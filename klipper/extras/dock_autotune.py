# Dock autotune for klipper-toolchanger (KTC) docks
#
# Finds, per axis, the window in which a tool still locks into the shuttle and puts
# params_park_x/y/z of that tool in the middle of it.
#
# One trial = SELECT_TOOL (pick the tool up from its dock, the verify at the lock position
# of the pickup path must see the tool) followed by UNSELECT_TOOL (put it back). Starting
# from the current value the offset is moved outwards in steps until a trial fails, on
# both sides. The window centre is the proposal. A failed trial is recovered by the
# toolchanger's own error_gcode (the shuttle leaves the dock along the dock's path); while
# a scan is running, `printer.dock_autotune.active` is True and that handler does not pause.
#
# Optionally the accelerometer of the tool is recorded during every trial (peak and RMS of
# the shock, written to the log). It only helps to judge a position and to stop a scan when
# something hits hard (abort_g); it never decides on its own.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import logging
import math
import os
import time

AXES = 'xyz'
FREEFALL = 9.80665          # m/s^2, Klipper reports accelerometer samples in m/s^2
RESTORE_NONE = "RESTORE_AXIS=''"   # no position restore: the scan does not need the old position


class AbortScan(Exception):
    """The scan is stopped on purpose (interrupt, unsafe state)"""


# ---------------------------------------------------------------------------
# Pure helpers (no Klipper needed, used by the tests)
# ---------------------------------------------------------------------------

def scan_axis(trial, step, limit, refine=True):
    """
    Search the window around offset 0 in which trial(offset) is True (offset 0 must work).

    Moves outwards in `step` increments on each side until a trial fails or `limit` is
    reached. With `refine` one more trial halves the step at the failing edge.
    Returns the last good offset on each side and whether the side is still open (no failure
    up to `limit`).
    """
    edges = {}
    for sign in (1, -1):
        good, bad, k = 0.0, None, 1
        while k * step <= limit + 1e-9:
            offset = sign * round(k * step, 6)
            if trial(offset):
                good = offset
                k += 1
            else:
                bad = offset
                break
        if bad is not None and refine:
            middle = round((good + bad) / 2.0, 6)
            if abs(middle - good) > 1e-9 and trial(middle):
                good = middle
        edges[sign] = (good, bad is None)
    return {'plus': edges[1][0], 'plus_open': edges[1][1],
            'minus': edges[-1][0], 'minus_open': edges[-1][1]}


def window_center(edges):
    """Centre of the window found by scan_axis, None if a side is still open (not reliable)"""
    if edges['plus_open'] or edges['minus_open']:
        return None
    return (edges['plus'] + edges['minus']) / 2.0


def shock_metrics(samples):
    """
    Shock numbers of accelerometer samples [(time, x, y, z), ...] in m/s^2, in g.
    The static part (gravity, orientation) is the median of each axis over the samples.
    peak_dev_g:   largest deviation from that median
    peak_delta_g: largest change between two consecutive samples (a knock, not a tilt)
    rms_g:        RMS of the deviation
    """
    if not samples or len(samples) < 3:
        return None
    columns = list(zip(*[(s[1], s[2], s[3]) for s in samples]))
    medians = [sorted(c)[len(c) // 2] for c in columns]
    dev = [math.sqrt(sum((v - m) ** 2 for v, m in zip(s[1:4], medians))) / FREEFALL for s in samples]
    delta = [math.sqrt(sum((a - b) ** 2 for a, b in zip(s[1:4], p[1:4]))) / FREEFALL
             for p, s in zip(samples, samples[1:])]
    return {'samples': len(samples), 'peak_dev_g': round(max(dev), 3),
            'peak_delta_g': round(max(delta), 3),
            'rms_g': round(math.sqrt(sum(d * d for d in dev) / len(dev)), 3)}


# ---------------------------------------------------------------------------
# Klipper object
# ---------------------------------------------------------------------------

class DockAutotune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.step = config.getfloat('step', 0.1, above=0.)
        self.range = {'x': config.getfloat('range_x', 0.6, above=0.),
                      'y': config.getfloat('range_y', 0.5, above=0.),
                      'z': config.getfloat('range_z', 0.5, above=0.)}
        self.repeats = config.getint('repeats', 2, minval=1)
        self.refine = config.getboolean('refine', True)
        # Slower than a normal tool change while the scan runs
        self.speed_factor = config.getfloat('speed_factor', 0.5, above=0., maxval=1.)
        self.accel = config.getfloat('accel', 2000., above=0.)
        self.settle = config.getfloat('settle', 0.3, minval=0.)
        self.max_cycles = config.getint('max_cycles', 400, minval=1)
        # Accelerometer of the tool: "adxl345 T{tool}" (the section must be named per tool)
        self.accel_chip = config.get('accelerometer', 'adxl345 T{tool}')
        self.abort_g = config.getfloat('abort_g', 0., minval=0.)
        self.log_file = os.path.expanduser(
            config.get('log_file', '~/printer_data/logs/dock_autotune.jsonl'))
        self.active = False
        self.abort = False
        self.use_adxl = False
        self.state = {'tool': None, 'axis': None, 'phase': 'idle', 'trial': 0, 'message': ''}
        self.result = {}
        self.cycles = 0
        self.gcode.register_command('DOCK_AUTOTUNE', self.cmd_DOCK_AUTOTUNE,
                                    desc=self.cmd_DOCK_AUTOTUNE_help)
        # 'INTERRUPT' in the name: Klipper runs it without waiting for the running scan
        self.gcode.register_command('DOCK_AUTOTUNE_INTERRUPT', self.cmd_INTERRUPT,
                                    desc="Stop a running DOCK_AUTOTUNE after the current trial")

    def get_status(self, eventtime=None):
        return {'active': self.active, 'tool': self.state['tool'], 'axis': self.state['axis'],
                'phase': self.state['phase'], 'trial': self.state['trial'],
                'message': self.state['message'], 'result': self.result}

    # -- helpers ------------------------------------------------------------

    def _now(self):
        return self.reactor.monotonic()

    def _toolchanger(self):
        return self.printer.lookup_object('toolchanger')

    def _tc_status(self):
        return self._toolchanger().get_status(self._now())

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _check_abort(self):
        if self.abort:
            raise AbortScan('stopped by DOCK_AUTOTUNE_INTERRUPT')

    def _log(self, record):
        record['time'] = time.strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(record) + '\n')
        except OSError:
            logging.exception("dock_autotune: cannot write %s", self.log_file)

    # -- accelerometer (optional) ----------------------------------------------

    def _adxl_start(self, tool_number):
        if not self.use_adxl:
            return None
        chip = self.printer.lookup_object(self.accel_chip.format(tool=tool_number), None)
        if chip is None or not hasattr(chip, 'start_internal_client'):
            return None
        try:
            return chip.start_internal_client()
        except Exception:
            logging.exception("dock_autotune: accelerometer of T%d not usable", tool_number)
            return None

    def _adxl_finish(self, client):
        if client is None:
            return None
        try:
            client.finish_measurements()
            return shock_metrics(client.get_samples())
        except Exception:
            logging.exception("dock_autotune: accelerometer read failed")
            return None

    # -- one pickup + dropoff ---------------------------------------------------

    def _cycle(self, tool_number):
        """One pickup and dropoff. Returns (picked, error message, shock metrics)."""
        self._check_abort()
        self.cycles += 1
        if self.cycles > self.max_cycles:
            raise AbortScan('max_cycles (%d) reached' % self.max_cycles)
        client = self._adxl_start(tool_number)
        picked, error = True, ''
        try:
            self._run("SELECT_TOOL T=%d %s" % (tool_number, RESTORE_NONE))
        except self.gcode.error as e:
            picked, error = False, str(e)
        if picked:
            status = self._tc_status()
            picked = status['status'] == 'ready' and status['tool_number'] == tool_number
            error = '' if picked else 'toolchanger %s, tool %s' % (status['status'], status['tool_number'])
        if picked:
            # An error while the tool is put back is not a trial result: the scan stops
            self._run("UNSELECT_TOOL %s" % RESTORE_NONE)
            self._wait()
        else:
            self._recover(tool_number)
        metrics = self._adxl_finish(client)
        if metrics and self.abort_g and metrics['peak_dev_g'] > self.abort_g:
            raise AbortScan('shock %.1f g is above abort_g %.1f g' % (metrics['peak_dev_g'], self.abort_g))
        return picked, error, metrics

    def _wait(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        if self.settle:
            toolhead.dwell(self.settle)
            toolhead.wait_moves()

    def _recover(self, tool_number):
        """After a failed pickup the toolchanger forgot its state: initialize it again from the pins
        and check that the shuttle really is empty and out of the dock"""
        self._wait()
        pause_resume = self.printer.lookup_object('pause_resume', None)
        if pause_resume is not None and pause_resume.get_status(self._now()).get('is_paused'):
            raise AbortScan('the failed pickup paused the print: the error handler of the toolchanger does not '
                            'know the scan mode (_TOOLCHANGE_ERROR_MOVE with printer.dock_autotune.active) - '
                            'update toolchanger-config.cfg, then RESUME or CLEAR_PAUSE')
        self._run('INITIALIZE_TOOLCHANGER')
        status = self._tc_status()
        if status['status'] != 'ready' or status['tool_number'] != -1 or status.get('detected_tool_number', -1) != -1:
            raise AbortScan('after the failed pickup the toolchanger reports %s, tool %s, detected %s - '
                            'check the shuttle and the dock' % (status['status'], status['tool_number'],
                                                                status.get('detected_tool_number')))
        tool = self.printer.lookup_object('tool T%d' % tool_number)
        safe_y = float(tool.params['params_safe_y'])
        position = self.printer.lookup_object('toolhead').get_status(self._now())['position']
        if position[1] < safe_y - 2.0:
            raise AbortScan('after the failed pickup the shuttle is at Y=%.1f, not at safe Y %.1f - '
                            'check the dock' % (position[1], safe_y))

    def _trial(self, tool, axis, value):
        """Set the park value and try pickup + dropoff `repeats` times"""
        tool.set_parameter('params_park_%s' % axis, round(value, 4))
        self.state['trial'] += 1
        shocks = []
        ok, error = True, ''
        for _ in range(self.repeats):
            picked, error, metrics = self._cycle(tool.tool_number)
            if metrics:
                shocks.append(metrics['peak_dev_g'])
            if not picked:
                ok = False
                break
        self._log({'tool': tool.tool_number, 'axis': axis, 'value': round(value, 4), 'ok': ok,
                   'peak_g': max(shocks) if shocks else None, 'error': error})
        return ok

    # -- the scan -----------------------------------------------------------------

    def _scan_tool(self, gcmd, tool, axes, limits, step):
        number = tool.tool_number
        base = {a: float(tool.params['params_park_%s' % a]) for a in axes}
        result = {}
        for axis in axes:
            self._check_abort()
            self.state['axis'] = axis
            self.state['phase'] = 'baseline %s' % axis
            gcmd.respond_info("dock_autotune T%d axis %s: baseline at %.3f" % (number, axis.upper(), base[axis]))
            if not self._trial(tool, axis, base[axis]):
                raise AbortScan("baseline %s=%.3f does not work (tool not in its dock, or the dock must be "
                                "adjusted by hand first)" % (axis.upper(), base[axis]))
            self.state['phase'] = 'scan %s' % axis
            edges = scan_axis(lambda offset, a=axis: self._trial(tool, a, base[a] + offset),
                              step, limits[axis], self.refine)
            center = window_center(edges)
            entry = {'start': base[axis], 'plus': edges['plus'], 'minus': edges['minus'],
                     'plus_open': edges['plus_open'], 'minus_open': edges['minus_open'],
                     'width': edges['plus'] - edges['minus'],
                     'proposal': None if center is None else round(base[axis] + center, 3)}
            result[axis] = entry
            if entry['proposal'] is None:
                # the window is wider than the range: keep the old value for this axis
                tool.set_parameter('params_park_%s' % axis, base[axis])
                gcmd.respond_info("dock_autotune T%d %s: no failure within +/-%.2f mm on one side, window "
                                  "not closed - value kept" % (number, axis.upper(), limits[axis]))
            else:
                self.state['phase'] = 'check %s' % axis
                if self._trial(tool, axis, entry['proposal']):
                    base[axis] = entry['proposal']
                    gcmd.respond_info("dock_autotune T%d %s: window %+.3f .. %+.3f mm (%.3f wide), centre -> %.3f"
                                      % (number, axis.upper(), edges['minus'], edges['plus'], entry['width'],
                                         entry['proposal']))
                else:
                    entry['proposal'] = None
                    tool.set_parameter('params_park_%s' % axis, base[axis])
                    gcmd.respond_info("dock_autotune T%d %s: the centre did not lock - value kept"
                                      % (number, axis.upper()))
        return base, result

    # -- commands -------------------------------------------------------------------

    cmd_DOCK_AUTOTUNE_help = ("Find the window in which a tool locks and centre its dock position: "
                              "DOCK_AUTOTUNE T=<n> [AXES=xyz] [STEP=0.1] [RANGE=<mm>] [REPEATS=2] "
                              "[SPEED=0.5] [ADXL=1] [DRY_RUN=1]")

    def cmd_DOCK_AUTOTUNE(self, gcmd):
        if self.active:
            raise gcmd.error("DOCK_AUTOTUNE is already running (DOCK_AUTOTUNE_INTERRUPT stops it)")
        number = gcmd.get_int('T', minval=0)
        axes = ''.join(a for a in gcmd.get('AXES', 'xyz').lower() if a in AXES)
        if not axes:
            raise gcmd.error("AXES must contain x, y or z")
        step = gcmd.get_float('STEP', self.step, above=0.)
        limits = dict(self.range)
        if gcmd.get('RANGE', None) is not None:
            limits = {a: gcmd.get_float('RANGE', above=0.) for a in AXES}
        for a in AXES:
            limits[a] = gcmd.get_float('RANGE_%s' % a.upper(), limits[a], above=0.)
        repeats = gcmd.get_int('REPEATS', self.repeats, minval=1)
        speed = gcmd.get_float('SPEED', self.speed_factor, above=0., maxval=1.)
        self.use_adxl = gcmd.get_int('ADXL', 0) == 1
        if self.use_adxl and self.printer.lookup_object(self.accel_chip.format(tool=number), None) is None:
            gcmd.respond_info("dock_autotune: ADXL=1 but there is no [%s] section - scanning without the "
                              "accelerometer" % self.accel_chip.format(tool=number))
        dry_run = gcmd.get_int('DRY_RUN', 0) == 1
        tool = self.printer.lookup_object('tool T%d' % number, None)
        if tool is None:
            raise gcmd.error("DOCK_AUTOTUNE: no tool T%d" % number)
        self._preconditions(gcmd, number)

        per_axis = {a: int(limits[a] / step + 1e-9) for a in axes}
        trials = sum(2 * n + (2 if self.refine else 0) + 2 for n in per_axis.values())
        gcmd.respond_info("dock_autotune T%d, axes %s, step %.3f mm, range %s, %d repeat(s): about %d trials "
                          "(%d pickups) at most, current dock position x=%.3f y=%.3f z=%.3f"
                          % (number, axes.upper(), step, ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                             repeats, trials, trials * repeats,
                             *[float(tool.params['params_park_%s' % a]) for a in 'xyz']))
        if dry_run:
            gcmd.respond_info("DRY_RUN: nothing moved")
            return
        self.repeats, saved_repeats = repeats, self.repeats
        self._start(number)
        toolhead = self.printer.lookup_object('toolhead')
        saved_accel = toolhead.get_status(self._now())['max_accel']
        saved_speed = float(tool.params['params_path_speed'])
        originals = {a: float(tool.params['params_park_%s' % a]) for a in AXES}
        proposals, base = {}, {}
        try:
            self._run("SET_VELOCITY_LIMIT ACCEL=%g" % min(self.accel, saved_accel))
            tool.set_parameter('params_path_speed', saved_speed * speed)
            if self._tc_status()['tool_number'] == number:
                self._run("UNSELECT_TOOL %s" % RESTORE_NONE)
                self._wait()
            base, result = self._scan_tool(gcmd, tool, axes, limits, step)
            proposals = {a: base[a] for a in axes if result[a]['proposal'] is not None}
            self.result = {'tool': number, 'axes': result}
            self._finish_ok(gcmd, number, originals, proposals, result)
        except AbortScan as e:
            self._abort_report(gcmd, number, e)
        except self.gcode.error as e:
            self._abort_report(gcmd, number, e)
            raise
        finally:
            # back to the old values except for the axes that were centred
            for a in AXES:
                if a in proposals:
                    tool.set_parameter('params_park_%s' % a, round(proposals[a], 3))
                else:
                    tool.set_parameter('params_park_%s' % a, originals[a])
            tool.set_parameter('params_path_speed', saved_speed)
            try:
                self._run("SET_VELOCITY_LIMIT ACCEL=%g" % saved_accel)
            except self.gcode.error:
                logging.exception("dock_autotune: could not restore the acceleration")
            self.repeats = saved_repeats
            self.active = False
            self.state['phase'] = 'idle'

    def _preconditions(self, gcmd, number):
        toolhead = self.printer.lookup_object('toolhead')
        if not all(a in toolhead.get_status(self._now())['homed_axes'] for a in 'xyz'):
            raise gcmd.error("DOCK_AUTOTUNE: home all axes first (same way as before a print: G28, QUAD_GANTRY_LEVEL, "
                             "touch home with T0)")
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(self._now()).get('state') in ('printing', 'paused'):
            raise gcmd.error("DOCK_AUTOTUNE: not while a print is running or paused")
        status = self._tc_status()
        if status['status'] != 'ready':
            raise gcmd.error("DOCK_AUTOTUNE: toolchanger is %s (INITIALIZE_TOOLCHANGER first)" % status['status'])
        if status['tool_number'] not in (-1, number):
            raise gcmd.error("DOCK_AUTOTUNE: T%d is on the shuttle, put it back first (UNSELECT_TOOL)"
                             % status['tool_number'])

    def _start(self, number):
        self.active = True
        self.abort = False
        self.cycles = 0
        self.result = {}
        self.state.update({'tool': number, 'axis': None, 'phase': 'start', 'trial': 0, 'message': ''})

    def _finish_ok(self, gcmd, number, originals, proposals, result):
        lines = ["dock_autotune T%d finished after %d pickups. Applied until the next restart:" % (number, self.cycles)]
        for axis in AXES:
            if axis in result:
                e = result[axis]
                if e['proposal'] is None:
                    lines.append("  params_park_%s: %.3f   (unchanged, window %s)" % (
                        axis, originals[axis], 'not closed' if e['plus_open'] or e['minus_open'] else 'centre failed'))
                else:
                    lines.append("  params_park_%s: %.3f   (was %.3f, window %.3f wide)" % (
                        axis, e['proposal'], originals[axis], e['width']))
        lines.append("Copy the changed lines into toolchanger/tools/T%d.cfg (or SAVE_TOOL_PARAMETER T=%d "
                     "PARAMETER=params_park_x ..., comment the line out in the cfg, SAVE_CONFIG)" % (number, number))
        gcmd.respond_info('\n'.join(lines))
        self._log({'tool': number, 'summary': {a: result[a] for a in result}})

    def _abort_report(self, gcmd, number, error):
        status = self._tc_status()
        gcmd.respond_info("dock_autotune T%d STOPPED: %s. Toolchanger: %s, tool %s. The park values are back at "
                          "their old values. Check the shuttle and the dock before you do anything else."
                          % (number, error, status['status'], status['tool_number']))
        self._log({'tool': number, 'aborted': str(error)})

    def cmd_INTERRUPT(self, gcmd):
        if self.active:
            self.abort = True
            gcmd.respond_info("dock_autotune: stopping after the current trial")
        else:
            gcmd.respond_info("dock_autotune: nothing is running")


def load_config(config):
    return DockAutotune(config)
