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
# Before the scan the machine can be brought into the state of a print start (bed temperature,
# homing, gantry levelling, touch homing with T0 for Cartographer / Beacon contact). The module
# detects the probe and offers that preparation in a prompt (Mainsail / Fluidd), see DOCK_AUTOTUNE.
#
# Commands: DOCK_AUTOTUNE_RUN (always) and DOCK_AUTOTUNE (the same, unless a macro of that name is
# configured, see config/dock_autotune_macro.cfg), DOCK_AUTOTUNE_INTERRUPT.
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
FREEFALL = 9806.65          # mm/s^2 = 1 g, Klipper reports accelerometer samples in mm/s^2
RESTORE_NONE = "RESTORE_AXIS=''"   # no position restore: the scan does not need the old position
PREPARE_CHOICES = {'ask': None, 'yes': 1, 'no': 0}      # option `prepare`: None = ask in the prompt


class AbortScan(Exception):
    """The scan is stopped on purpose (interrupt, unsafe state)"""


# ---------------------------------------------------------------------------
# Pure helpers (no Klipper needed, used by the tests)
# ---------------------------------------------------------------------------

def scan_axis(trial, step, limit, refine=True, resolution=None):
    """
    Search the window around offset 0 in which trial(offset) is True (offset 0 must work).

    Moves outwards in `step` increments on each side until a trial fails or `limit` is
    reached. With `refine` the interval between the last good and the first failing offset is
    halved until it is not larger than `resolution` (default: half a step, one extra trial).
    Returns the last good offset on each side and whether the side is still open (no failure
    up to `limit`).
    """
    if resolution is None:
        resolution = step / 2.0
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
        while bad is not None and refine and abs(bad - good) > resolution + 1e-9:
            middle = round((good + bad) / 2.0, 6)
            if trial(middle):
                good = middle
            else:
                bad = middle
        edges[sign] = (good, bad is None)
    return {'plus': edges[1][0], 'plus_open': edges[1][1],
            'minus': edges[-1][0], 'minus_open': edges[-1][1]}


def describe_edge(offset, is_open):
    """One side of the window for a message: the edge, or the last offset that still locked"""
    return '%+.2f mm (open, no failure up to here)' % offset if is_open else '%+.2f mm (edge)' % offset


def window_center(edges):
    """Centre of the window found by scan_axis, None if a side is still open (not reliable)"""
    if edges['plus_open'] or edges['minus_open']:
        return None
    return (edges['plus'] + edges['minus']) / 2.0


def shock_metrics(samples):
    """
    Shock numbers of accelerometer samples [(time, x, y, z), ...] in mm/s^2, in g.
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
# ADXL map: what the accelerometer of the tool sees while it is picked up (pure, used by the tests)
# ---------------------------------------------------------------------------

def residuals(samples, window=0.05):
    """
    Samples [(time, x, y, z), ...] in mm/s^2 -> (times, rate in Hz, [rx, ry, rz]) in g with the slow part
    removed: gravity, tilt and the smooth acceleration of the moves are the moving mean over `window` seconds,
    what is left is what happens fast (a knock, the click of the coupling).
    """
    n = len(samples)
    times = [s[0] for s in samples]
    rate = (n - 1) / (times[-1] - times[0]) if n > 1 and times[-1] > times[0] else 0.0
    half = max(1, int(round(rate * window)) // 2)
    columns = []
    for axis in (1, 2, 3):
        values = [s[axis] / FREEFALL for s in samples]
        prefix = [0.0]
        for v in values:
            prefix.append(prefix[-1] + v)
        out = []
        for i, v in enumerate(values):
            lo, hi = max(0, i - half), min(n, i + half + 1)
            out.append(v - (prefix[hi] - prefix[lo]) / (hi - lo))
        columns.append(out)
    return times, rate, columns


def knock_metrics(samples, t_from, t_to, window=0.05):
    """
    The strongest knock of the samples between t_from and t_to: peak (g, length of the residual vector), its
    time in s after t_from and its signed components x, y, z in g (axes of the chip after axes_map), the RMS of
    the residual and the sample rate. None if there are too few samples.
    """
    part = [s for s in samples if t_from <= s[0] <= t_to]
    if len(part) < 20:
        return None
    times, rate, (rx, ry, rz) = residuals(part, window)
    norms = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(rx, ry, rz)]
    i = max(range(len(norms)), key=norms.__getitem__)
    return {'n': len(part), 'rate_hz': round(rate), 'peak_g': round(norms[i], 3), 't_peak': round(times[i] - t_from, 3),
            'x': round(rx[i], 3), 'y': round(ry[i], 3), 'z': round(rz[i], 3),
            'rms_g': round(math.sqrt(sum(v * v for v in norms) / len(norms)), 4)}


def csv_snippet(samples, t_from, t_to, t_peak, half=0.25, window=0.05):
    """CSV lines (time after t_from, raw x, y, z in g, residual x, y, z in g) around the peak"""
    part = [s for s in samples if t_from <= s[0] <= t_to]
    if len(part) < 20:
        return []
    times, _, (rx, ry, rz) = residuals(part, window)
    lines = ['time_s,x_g,y_g,z_g,res_x_g,res_y_g,res_z_g']
    for i, s in enumerate(part):
        if abs(times[i] - t_from - t_peak) <= half:
            lines.append('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f' % (
                times[i] - t_from, s[1] / FREEFALL, s[2] / FREEFALL, s[3] / FREEFALL, rx[i], ry[i], rz[i]))
    return lines


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return None
    return values[n // 2] if n % 2 else (values[n // 2 - 1] + values[n // 2]) / 2.0


def zero_crossings(rows, key):
    """Offsets at which the median component `key` of the map changes sign (linear between two positions)"""
    points = [(r['offset'], r['pick'][key]) for r in rows if r['ok'] and r.get('pick')]
    points.sort()
    found = []
    for (o1, v1), (o2, v2) in zip(points, points[1:]):
        if v1 * v2 < 0:
            found.append(round(o1 + (o2 - o1) * v1 / (v1 - v2), 3))
    return found


def map_table(tool, axis, rows, window):
    """The map of one axis as text lines. rows: offset, value, locked, total, ok, pick, drop"""
    lines = ["ADXL map T%d %s: knock in g after a %d ms high-pass, signed components at the peak of the pickup "
             "(median of the repeats; x y z are the axes of the chip after axes_map)" % (
                 tool, axis.upper(), int(round(window * 1000))),
             " offset      value  locked   pickup      x      y      z  t_peak  dropoff"]
    for r in sorted(rows, key=lambda r: r['offset']):
        pick, drop = r.get('pick'), r.get('drop')
        lines.append("%+7.2f  %9.3f  %d/%d    %s  %s" % (
            r['offset'], r['value'], r['locked'], r['total'],
            ("%7.3f %6.2f %6.2f %6.2f %6.2f s" % (pick['peak_g'], pick['x'], pick['y'], pick['z'], pick['t_peak'])
             if pick else "      -      -      -      -       -"),
            ("%7.3f" % drop['peak_g']) if drop else "      -"))
    good = [r for r in rows if r['ok'] and r.get('pick')]
    if good:
        quiet = min(good, key=lambda r: r['pick']['peak_g'])
        lines.append("Quietest pickup: %+.2f mm (%.3f g)" % (quiet['offset'], quiet['pick']['peak_g']))
        for key in 'xyz':
            crossings = zero_crossings(rows, key)
            if crossings:
                lines.append("Component %s changes sign at %s mm" % (key, ', '.join('%+.2f' % c for c in crossings)))
    lines.append("Nothing is changed by a map. Locked = pickups that locked / tried.")
    return lines


def _park_result(axis, info):
    """(after value, text) of one axis for the report"""
    before = info['originals'][axis]
    entry = info.get('scan', {}).get(axis)
    if info['mode'] == 'seat':
        if axis == 'z':
            return before, 'height of the contact measured, not changed by the seat test'
        return before, ('measured, not changed by the seat test' if axis in info.get('seat', {}).get('rows', {})
                        else 'not measured')
    if info['mode'] == 'map':
        return before, ('measured, not changed by a map' if axis in info.get('map', {}) else 'not measured')
    if info.get('new', {}).get(axis) is not None:
        return info['new'][axis], 'centred, window %+.3f .. %+.3f mm (%.3f wide)' % (
            entry['minus'], entry['plus'], entry['width'])
    if entry is None:
        return before, 'not tuned' if not info['status'].startswith('STOPPED') else 'no result'
    if entry['plus_open'] or entry['minus_open']:
        return before, 'unchanged, window not closed (at least %.2f mm wide: %s .. %s)' % (
            entry['width'], describe_edge(entry['minus'], entry['minus_open']),
            describe_edge(entry['plus'], entry['plus_open']))
    return before, 'unchanged, the centre of the window did not lock'


def build_report(info):
    """
    The result file of one run as text: the park positions (before, after), the ADXL measurements or the trials of the
    scan, and all numbers as CSV. info: time, tool, mode, status, pickups, settings (list of text), originals, new,
    scan (result per axis), trials, map (rows per axis), seat (rows per axis and the height of the lift), window,
    raw_dir
    """
    tool = info['tool']
    lines = ["Dock autotune T%d - %s - %s" % (tool, {'map': 'ADXL map', 'seat': 'seat test'}.get(info['mode'], 'scan'), info['time']),
             "Status: %s (%d %s)" % (info['status'], info['pickups'], 'lifts' if info['mode'] == 'seat' else 'pickups'),
             "Settings: " + ', '.join(info['settings']), "",
             "Park positions of T%d" % tool,
             "  axis      before       after   result"]
    changed = []
    for axis in AXES:
        after, text = _park_result(axis, info)
        lines.append("  %-4s %10.3f  %10.3f   %s" % (axis, info['originals'][axis], after, text))
        if after != info['originals'][axis]:
            changed.append("params_park_%s: %.3f" % (axis, after))
    lines.append("")
    if changed:
        lines.append("Put these lines into toolchanger/tools/T%d.cfg (applied until the next restart):" % tool)
        lines.extend("  " + c for c in changed)
    else:
        lines.append("No park position was changed.")
    if info.get('seat'):
        lines.append("")
        lines.extend(seat_summary(info['seat']))
        for axis in AXES:
            if info['seat']['rows'].get(axis):
                lines.append("")
                lines.extend(seat_table(tool, axis, info['seat']['rows'][axis], info['window']))
    for axis in AXES:
        if info.get('map', {}).get(axis):
            lines.append("")
            lines.extend(map_table(tool, axis, info['map'][axis], info['window']))
    for axis in AXES:
        trials = [t for t in info.get('trials', []) if t['axis'] == axis]
        if trials:
            start = info['originals'][axis]
            lines.extend(["", "Scan trials %s in the order they were made (offset from the start value %.3f)"
                          % (axis.upper(), start), " offset      value  result  peak_g"])
            for t in trials:
                lines.append("%+7.3f  %9.3f  %-6s  %s" % (
                    t['value'] - start, t['value'], 'locked' if t['ok'] else 'FAILED',
                    '%6.3f' % t['peak_g'] if t['peak_g'] is not None else '     -'))
    lines.extend(["", "Data (CSV)"])
    if info['mode'] == 'seat':
        lines.append("tool,axis,offset_mm,value_mm,pin_ok,pin_total,events,z_first,z_last,peak_first_g,peak_last_g,"
                     "lateral_share,ix_mm_s,iy_mm_s,release_g")
        for axis in AXES:
            for r in sorted(info.get('seat', {}).get('rows', {}).get(axis, []), key=lambda r: r['offset']):
                lines.append(','.join('' if v is None else str(v) for v in (
                    tool, axis, r['offset'], r['value'], r['pin'], r['total'], r['events'], r['z_first'],
                    r['z_last'], r['peak_first'], r['peak_last'], r['lateral'], r['ix'], r['iy'], r['release_g'])))
    elif info['mode'] == 'map':
        lines.append("tool,axis,offset_mm,value_mm,locked,total,pick_peak_g,pick_x_g,pick_y_g,pick_z_g,pick_t_peak_s,"
                     "pick_rms_g,drop_peak_g")
        for axis in AXES:
            for r in sorted(info.get('map', {}).get(axis, []), key=lambda r: r['offset']):
                pick, drop = r.get('pick') or {}, r.get('drop') or {}
                lines.append(','.join(str(v) for v in (
                    tool, axis, r['offset'], r['value'], r['locked'], r['total'], pick.get('peak_g', ''),
                    pick.get('x', ''), pick.get('y', ''), pick.get('z', ''), pick.get('t_peak', ''),
                    pick.get('rms_g', ''), drop.get('peak_g', ''))))
    else:
        lines.append("tool,axis,offset_mm,value_mm,locked,peak_g")
        for t in info.get('trials', []):
            lines.append(','.join(str(v) for v in (
                tool, t['axis'], round(t['value'] - info['originals'][t['axis']], 4), t['value'],
                1 if t['ok'] else 0, '' if t['peak_g'] is None else t['peak_g'])))
    if info.get('raw_dir'):
        lines.extend(["", "Raw samples around the peak of every pickup (CSV): %s" % info['raw_dir']])
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------
# Seat test: the vertical lift into the shuttle, the tool stays in its dock (pure, used by the tests)
# ---------------------------------------------------------------------------

def parse_lift(path):
    """
    The part of a pickup path that the seat test uses: the waypoints up to the vertical lift into the verify
    waypoint. A waypoint without x, y or z keeps the value of the one before (all relative to the park position).
    Returns {'approach': waypoints up to the lowest point of the lift, 'low': that point, 'top': the verify waypoint},
    each {'x', 'y', 'z', 'f'}. Raises ValueError if the path has no such lift.
    """
    state = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    points = []
    for entry in path:
        for axis in AXES:
            if axis in entry:
                state[axis] = float(entry[axis])
        points.append({'x': state['x'], 'y': state['y'], 'z': state['z'], 'f': float(entry.get('f', 1.0)),
                       'verify': 'verify' in entry})
    index = next((i for i, p in enumerate(points) if p['verify']), None)
    if index is None or index == 0:
        raise ValueError("the pickup path has no waypoint with verify after a first waypoint")
    low, top = points[index - 1], points[index]
    if abs(low['x'] - top['x']) > 1e-6 or abs(low['y'] - top['y']) > 1e-6 or top['z'] <= low['z'] + 1e-6:
        raise ValueError("the move to the verify waypoint is not a vertical lift")
    return {'approach': points[:index], 'low': low, 'top': top}


def seat_enter_lines(geo, x, y, z, close_y, fast, path_speed):
    """G-code that takes the empty shuttle from safe Y to the lowest point of the lift, like the pickup_gcode"""
    first = geo['approach'][0]
    lines = ['G90', 'G0 Y%.3f F%g' % (close_y, fast),
             'G0 X%.4f Z%.4f F%g' % (x, z + first['z'], fast),
             'G0 Y%.4f F%g' % (y + first['y'], fast)]
    for p in geo['approach'][1:]:
        lines.append('G0 X%.4f Y%.4f Z%.4f F%g' % (x + p['x'], y + p['y'], z + p['z'], path_speed * p['f']))
    return lines


def seat_exit_lines(geo, x, y, z, safe_y, fast, path_speed):
    """G-code that takes the shuttle from the lowest point of the lift back out of the dock, like the dropoff path"""
    lines = ['G0 X%.4f Y%.4f Z%.4f F%g' % (x + p['x'], y + p['y'], z + p['z'], path_speed)
             for p in reversed(geo['approach'][:-1])]
    lines.append('G0 Y%.3f F%g' % (safe_y, fast))
    return lines


def move_duration(dist, speed, accel):
    """Duration of one move from rest to rest (mm, mm/s, mm/s^2)"""
    if dist <= 0.0:
        return 0.0
    if dist >= speed * speed / accel:
        return dist / speed + speed / accel
    return 2.0 * math.sqrt(dist / accel)


def height_at(t, segment):
    """Z at print time t during a rising move: the line of the cruise speed through the end of the move"""
    z = segment['z_end'] - segment['v'] * (segment['t_end'] - t) + segment['v'] ** 2 / (2.0 * segment['a'])
    return min(max(z, segment['z_start']), segment['z_end'])


def event_height(t, segments):
    """Z of the shuttle at print time t during a series of rising moves (in time order)"""
    for segment in segments:
        if segment['t_end'] - segment['dur'] - 0.002 <= t <= segment['t_end']:
            return height_at(t, segment)
    before = [sg for sg in segments if sg['t_end'] < t]
    return before[-1]['z_end'] if before else segments[0]['z_start']


def find_events(samples, t_from, t_to, t_quiet, window=0.05, threshold=0.0, gap=0.02):
    """
    The knocks in the samples between t_from and t_to: clusters (closer than `gap` s) of the residual (see
    residuals) above a threshold. The noise is the RMS of the residual before t_quiet, where the shuttle still
    stands; the threshold is 6 times the noise but at least 0.5 g, or `threshold` if that is given.
    Returns (events, noise in g, threshold in g). An event: t (first sample above the threshold), t_peak, peak_g,
    e_x, e_y, e_z (energy of the residual per axis, g^2 s) and i_x, i_y, i_z (net velocity change in mm/s).
    """
    part = [s for s in samples if t_from <= s[0] <= t_to]
    if len(part) < 40:
        return [], 0.0, threshold
    times, rate, (rx, ry, rz) = residuals(part, window)
    norms = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(rx, ry, rz)]
    quiet = [v for t, v in zip(times, norms) if t <= t_quiet]
    noise = math.sqrt(sum(v * v for v in quiet) / len(quiet)) if quiet else 0.0
    limit = threshold if threshold > 0 else max(0.5, 6.0 * noise)
    dt = 1.0 / rate if rate else 0.0
    clusters, current = [], None
    for i, (t, v) in enumerate(zip(times, norms)):
        if t <= t_quiet or v <= limit:
            continue
        if current is not None and t - times[current[1]] <= gap:
            current[1] = i
        else:
            current = [i, i]
            clusters.append(current)
    events = []
    for first, last in clusters:
        span = range(max(0, first - 5), min(len(times), last + 6))
        peak = max(range(first, last + 1), key=norms.__getitem__)
        event = {'t': times[first], 't_peak': times[peak], 'peak_g': norms[peak]}
        for name, column in (('x', rx), ('y', ry), ('z', rz)):
            event['e_' + name] = sum(column[i] ** 2 for i in span) * dt
            event['i_' + name] = sum(column[i] for i in span) * dt * FREEFALL
        events.append(event)
    return events, noise, limit


def seat_row(offset, value, hubs):
    """One position of the seat test: medians over its lifts. hubs: pin (seen at the top), events (z, rel, peak_g,
    e_x, e_y, e_z, i_x, i_y), release_g"""
    def med(values, digits=3):
        values = [v for v in values if v is not None]
        return round(median(values), digits) if values else None

    firsts = [min(h['events'], key=lambda e: e['z']) for h in hubs if h['events']]
    lasts = [max(h['events'], key=lambda e: e['z']) for h in hubs if h['events']]
    shares = []
    for h in hubs:
        lateral = sum(e['e_x'] + e['e_y'] for e in h['events'])
        total = lateral + sum(e['e_z'] for e in h['events'])
        if total > 0:
            shares.append(lateral / total)
    return {'offset': round(offset, 4), 'value': round(value, 4), 'pin': sum(1 for h in hubs if h['pin']),
            'total': len(hubs), 'ok': all(h['pin'] for h in hubs),
            'events': med([len(h['events']) for h in hubs], 1),
            'z_first': med([e['z'] for e in firsts]), 'z_last': med([e['z'] for e in lasts]),
            'peak_first': med([e['peak_g'] for e in firsts]), 'peak_last': med([e['peak_g'] for e in lasts]),
            'lateral': med(shares),
            'ix': med([sum(e['i_x'] for e in h['events']) for h in hubs if h['events']], 2),
            'iy': med([sum(e['i_y'] for e in h['events']) for h in hubs if h['events']], 2),
            'release_g': med([h['release_g'] for h in hubs])}


def seat_crossings(rows, key):
    """Offsets at which the value `key` of the seat rows changes sign (linear between two positions)"""
    points = sorted((r['offset'], r[key]) for r in rows if r.get(key) is not None)
    return [round(o1 + (o2 - o1) * v1 / (v1 - v2), 3)
            for (o1, v1), (o2, v2) in zip(points, points[1:]) if v1 * v2 < 0]


def seat_summary(seat):
    """Lines about the height of the lift: where it knocks, how far below the verify waypoint"""
    rows = [r for axis in AXES for r in seat['rows'].get(axis, [])]
    firsts = [r['z_first'] for r in rows if r['z_first'] is not None]
    lasts = [r['z_last'] for r in rows if r['z_last'] is not None]
    top = seat['verify_z']
    lines = ["Lift into the shuttle: park Z %.3f, verify waypoint at Z %.3f" % (seat['park_z'], top)]
    if lasts:
        lines.append("  first knock at Z %.3f (%.3f mm below the verify waypoint), spread over the positions "
                     "%.3f mm" % (median(firsts), top - median(firsts), max(firsts) - min(firsts)))
        lines.append("  last knock at Z %.3f (%.3f mm below the verify waypoint), spread %.3f mm"
                     % (median(lasts), top - median(lasts), max(lasts) - min(lasts)))
    coarse = seat.get('coarse')
    if coarse:
        lines.append("  first lift at path speed: %s" % (
            ', '.join('Z %.3f (%.1f g)' % (e['z'], e['peak_g']) for e in coarse['events']) or 'no knock'))
        if seat.get('slow_from_z') is not None:
            lines.append("  the slow part of the later lifts starts at Z %.3f (%g mm/s)"
                         % (seat['slow_from_z'], seat['speed']))
    lines.append("  the knock is the contact of the shuttle with the tool; the path continues until the verify "
                 "waypoint, so the last knock is that far below it")
    return lines


def seat_table(tool, axis, rows, window):
    """The seat test of one axis as text lines"""
    lines = ["Seat test T%d %s: knocks of the lift into the shuttle (g after a %d ms high-pass; z in mm; lateral = "
             "share of the knock energy in x and y; ix iy = net velocity change in x and y in mm/s; medians of "
             "the repeats)" % (tool, axis.upper(), int(round(window * 1000))),
             " offset  pin  events  z_first   z_last  peak1  peakN  lateral      ix      iy  release"]

    def num(v, fmt):
        return fmt % v if v is not None else ' ' * len(fmt % 0)
    for r in sorted(rows, key=lambda r: r['offset']):
        lines.append("%+7.2f  %d/%d  %s  %s  %s  %s  %s  %s  %s  %s  %s" % (
            r['offset'], r['pin'], r['total'], num(r['events'], '%6.1f'), num(r['z_first'], '%8.3f'),
            num(r['z_last'], '%8.3f'), num(r['peak_first'], '%5.2f'), num(r['peak_last'], '%5.2f'),
            num(r['lateral'], '%7.2f'), num(r['ix'], '%7.1f'), num(r['iy'], '%7.1f'), num(r['release_g'], '%7.2f')))
    done = [r for r in rows if r['lateral'] is not None]
    if done:
        low = min(done, key=lambda r: r['lateral'])
        lines.append("Lowest lateral share: %+.2f mm (%.2f)" % (low['offset'], low['lateral']))
        for key in ('ix', 'iy'):
            crossings = seat_crossings(rows, key)
            if crossings:
                lines.append("Net velocity change %s changes sign at %s mm" % (
                    key[1], ', '.join('%+.2f' % c for c in crossings)))
    lines.append("Nothing is changed by the seat test. Pin = lifts for which the tool detection was on at the top.")
    return lines


# ---------------------------------------------------------------------------
# Preparation: detection, plan, prompt (pure, used by the tests)
# ---------------------------------------------------------------------------

BED_MIN_BUTTON = 30.
EMPTY_SHUTTLE = '(put a tool that is on the shuttle back into its dock)'


def detect_setup(settings, commands):
    """
    What the machine has, from the parsed config (printer.configfile.settings) and the registered
    G-code commands. Decides if a preparation like a print start can be done with an empty shuttle.
    Returns a dict: probe (name or None), touch_cmd, level_cmd, has_bed, has_mesh, max_bed,
    can_prepare, reason (why not).
    """
    settings = {str(k).lower(): v for k, v in (settings or {}).items()}

    def has(name):
        return name in settings

    def has_prefix(prefix):
        return any(k == prefix or k.startswith(prefix + ' ') for k in settings)

    endstop = str((settings.get('stepper_z') or {}).get('endstop_pin', '')).strip().lower()
    probe, touch_cmd, reason = None, None, ''
    if has('cartographer'):
        probe = 'Cartographer'
        if 'CARTOGRAPHER_TOUCH_HOME' in commands:
            touch_cmd = 'CARTOGRAPHER_TOUCH_HOME'
    elif has('beacon'):
        probe = 'Beacon'
        # with home_method contact every G28 Z touches the bed with the nozzle of the mounted tool
        if str((settings.get('beacon') or {}).get('home_method', '')).strip().lower() == 'contact':
            touch_cmd = 'G28 Z'
    elif has_prefix('probe_eddy_current'):
        probe = 'Eddy current probe'
    elif has('scanner'):
        probe = 'Cartographer (legacy [scanner] module)'
        reason = ("the legacy [scanner] module is not recognised: its touch homing cannot be told apart "
                  "from the scan homing")
    elif has('probe') or has('bltouch'):
        probe = 'probe' if has('probe') else 'bltouch'
    if has_prefix('tool_probe') or endstop.startswith('tool_probe'):
        reason = "the Z probe sits on a tool (tool_probe): Z cannot be homed with an empty shuttle"
    elif not reason and endstop.startswith('probe:') and probe is None:
        reason = ("Z homes on a probe but no Cartographer, Beacon, eddy current or probe section was found: "
                  "Z cannot be homed with an empty shuttle")
    level = None
    if has('quad_gantry_level'):
        level = 'QUAD_GANTRY_LEVEL'
    elif has('z_tilt'):
        level = 'Z_TILT_ADJUST'
    try:
        max_bed = float((settings.get('heater_bed') or {}).get('max_temp'))
    except (TypeError, ValueError):
        max_bed = None
    return {'probe': probe, 'touch_cmd': touch_cmd, 'level_cmd': level, 'has_bed': has('heater_bed'),
            'has_mesh': has('bed_mesh'), 'max_bed': max_bed, 'can_prepare': not reason, 'reason': reason}


def build_plan(setup, bed, soak, touch_cmd, touch_tool, touch_temp, touch_repeats, before_touch=(),
               touch_restore='Z'):
    """The preparation as a list of G-code lines (the PRINT_START up to the preparation of the tools)"""
    plan = ['INITIALIZE_TOOLCHANGER', EMPTY_SHUTTLE]
    if bed > 0 and setup['has_bed']:
        plan.append('M190 S%g' % bed)
        if soak > 0:
            plan.append('G4 P%d' % int(round(soak * 1000)))
    plan.append('G28')
    if setup['level_cmd']:
        plan.append(setup['level_cmd'])
    if setup['has_mesh']:
        plan.append('BED_MESH_CLEAR')
    if touch_cmd:
        # like the T command in a PRINT_START: the pickup brings Z back to where it was before, so the touch homing
        # does not start at the height of the dock
        plan.append("SELECT_TOOL T=%d RESTORE_AXIS=%s" % (touch_tool, touch_restore.strip().upper() or "''"))
        if touch_temp > 0:
            plan.append('M104 S%g' % touch_temp)
        plan.extend(before_touch)
        if touch_temp > 0:
            plan.append('M109 S%g' % touch_temp)
        plan.extend([touch_cmd] * touch_repeats)
        if touch_temp > 0:
            plan.append('M104 S0')
        plan.append("UNSELECT_TOOL %s" % RESTORE_NONE)
    elif setup['level_cmd']:
        plan.append('G28 Z')        # levelling moved Z, the reference is taken again like in a print
    return plan


def command_line(params, **override):
    """DOCK_AUTOTUNE with the parameters of the call, some replaced or removed (value None)"""
    merged = {str(k).upper(): v for k, v in params.items()}
    for key, value in override.items():
        merged[key.upper()] = value
    parts = ['T=%s' % merged.pop('T')] if 'T' in merged else []
    parts += ['%s=%s' % (k, v) for k, v in merged.items() if v is not None]
    return 'DOCK_AUTOTUNE ' + ' '.join(parts)


def build_prompt(number, params, setup, plan, bed, touch_cmd):
    """Lines of a Mainsail / Fluidd action prompt that offers the preparation"""
    def line(text):
        return '// action:' + text.replace('\n', ' ')

    out = [line('prompt_begin Dock autotune T%d' % number)]
    close = line('prompt_footer_button Close|RESPOND TYPE=command MSG=action:prompt_end|info')
    if not setup['can_prepare']:
        out.append(line('prompt_text Preparation is not possible: %s.' % setup['reason']))
        out.append(line('prompt_text Home and prepare the machine by hand like before a print (bed temperature, '
                        'homing, gantry levelling, Z reference, shuttle empty), then continue.'))
        out.append(line('prompt_button_group_start'))
        out.append(line('prompt_button The machine is prepared - continue|%s|warning'
                        % command_line(params, PREPARE=0, BED=None)))
        out.append(line('prompt_button_group_end'))
        out.extend([close, line('prompt_show')])
        return out
    out.append(line('prompt_text Probe: %s. %s.' % (
        setup['probe'] or 'none (Z endstop)',
        ('Touch homing: %s with T%s' % (touch_cmd, plan_touch_tool(plan))) if touch_cmd else 'No touch homing')))
    steps = [p for p in plan if p not in ('INITIALIZE_TOOLCHANGER', EMPTY_SHUTTLE)]
    out.append(line('prompt_text Preparation like the PRINT_START: ' + ', '.join(steps)))
    out.append(line('prompt_text Bed temperature: %g C (change it with the buttons)' % bed))
    out.append(line('prompt_button_group_start'))
    top = setup['max_bed'] or 120.
    for delta in (-10, -5, 5, 10):
        new = min(max(bed + delta, BED_MIN_BUTTON), top)
        if new != bed:
            out.append(line('prompt_button %+d C|%s|secondary' % (delta, command_line(params, BED='%g' % new))))
    out.append(line('prompt_button_group_end'))
    out.append(line('prompt_button Prepare and start (bed %g C)|%s|primary'
                    % (bed, command_line(params, PREPARE=1, BED='%g' % bed))))
    out.append(line('prompt_button The machine is already prepared|%s|warning'
                    % command_line(params, PREPARE=0, BED=None)))
    out.extend([line('prompt_footer_button Cancel|RESPOND TYPE=command MSG=action:prompt_end|info'),
                line('prompt_show')])
    return out


def plan_touch_tool(plan):
    """Number of the tool the plan uses for the touch homing"""
    for entry in plan:
        if entry.startswith('SELECT_TOOL T='):
            return entry.split('T=')[1].split()[0]
    return '0'


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
        self.resolution = config.getfloat('resolution', 0.05, above=0.)
        # Slower than a normal tool change while the scan runs
        self.speed_factor = config.getfloat('speed_factor', 0.5, above=0., maxval=1.)
        self.accel = config.getfloat('accel', 2000., above=0.)
        self.settle = config.getfloat('settle', 0.3, minval=0.)
        self.max_cycles = config.getint('max_cycles', 400, minval=1)
        # Preparation like a print start (offered in a prompt)
        self.bed_temp = config.getfloat('bed_temp', 70., minval=0.)
        self.bed_soak = config.getfloat('bed_soak', 0., minval=0.)
        self.touch_command = config.get('touch_command', 'auto').strip()
        self.touch_tool = config.getint('touch_tool', 0, minval=0)
        self.touch_temp = config.getfloat('touch_temp', 150., minval=0.)
        self.touch_repeats = config.getint('touch_repeats', 2, minval=1)
        self.touch_restore = config.get('touch_restore_axis', 'Z')
        self.before_touch = [l.strip() for l in config.get('before_touch', '').splitlines() if l.strip()]
        # Accelerometer of the tool: "adxl345 T{tool}" (the section must be named per tool)
        self.accel_chip = config.get('accelerometer', 'adxl345 T{tool}')
        self.abort_g = config.getfloat('abort_g', 0., minval=0.)
        # ADXL map (MODE=map): high-pass window of the knock metric, step and repeats of the grid, how many
        # failed positions in a row end one side of the grid
        self.adxl_window = config.getfloat('adxl_window', 0.05, above=0.)
        self.map_step = config.getfloat('map_step', 0.25, above=0.)
        self.map_repeats = config.getint('map_repeats', 3, minval=1)
        self.map_fail_stop = config.getint('map_fail_stop', 2, minval=1)
        self.map_range = config.getfloat('map_range', 1.5, above=0.)
        # seat test (MODE=seat): lift into the shuttle without leaving the dock. seat_speed is the speed (mm/s)
        # of the part around the contact, which starts seat_zone mm below the first knock; the XY positions go
        # seat_range mm to both sides in seat_step steps with seat_repeats lifts each
        self.seat_speed = config.getfloat('seat_speed', 2., above=0.)
        self.seat_zone = config.getfloat('seat_zone', 1.5, minval=0.)
        self.seat_range = config.getfloat('seat_range', 1., above=0.)
        self.seat_step = config.getfloat('seat_step', 0.25, above=0.)
        self.seat_repeats = config.getint('seat_repeats', 3, minval=1)
        self.seat_threshold = config.getfloat('seat_threshold', 0., minval=0.)
        # defaults of the command line: DOCK_AUTOTUNE T=0 alone does what these say
        self.mode = config.get('mode', 'scan').strip().lower()
        if self.mode not in ('scan', 'map', 'seat'):
            raise config.error("dock_autotune: mode must be scan, map or seat")
        self.axes = ''.join(a for a in config.get('axes', AXES).lower() if a in AXES) or AXES
        self.adxl_default = config.getboolean('adxl', False)
        self.raw_default = config.getboolean('adxl_raw', False)
        prepare = config.get('prepare', 'ask').strip().lower()
        if prepare not in PREPARE_CHOICES:
            raise config.error("dock_autotune: prepare must be ask, yes or no")
        self.prepare_default = PREPARE_CHOICES[prepare]
        # the result file of every run (park positions + ADXL measurements), empty: no file
        self.results_dir = os.path.expanduser(
            config.get('results_dir', '~/printer_data/config/dock_autotune/results').strip())
        self.log_file = os.path.expanduser(
            config.get('log_file', '~/printer_data/logs/dock_autotune.jsonl'))
        self.raw_dir = os.path.join(os.path.dirname(self.log_file), 'dock_autotune_adxl')
        self.active = False
        self.abort = False
        self.use_adxl = False
        self.last_samples = None        # accelerometer samples of the last cycle, and when its pickup ended
        self.last_pick_end = None
        self.trial_rows = []            # data of the run for the result file
        self.map_rows = {}
        self.seat_rows = {}
        self.seat_info = {}
        self.scan_result = {}
        self.state = {'tool': None, 'axis': None, 'phase': 'idle', 'trial': 0, 'message': ''}
        self.result = {}
        self.cycles = 0
        # DOCK_AUTOTUNE_RUN always is the command of the module. DOCK_AUTOTUNE is it too, unless the config has
        # a [gcode_macro DOCK_AUTOTUNE] (dock_autotune_macro.cfg): Mainsail lists only macros, so that macro
        # carries the name and calls DOCK_AUTOTUNE_RUN
        self.gcode.register_command('DOCK_AUTOTUNE_RUN', self.cmd_DOCK_AUTOTUNE,
                                    desc=self.cmd_DOCK_AUTOTUNE_help)
        if not self._has_macro(config, 'DOCK_AUTOTUNE'):
            self.gcode.register_command('DOCK_AUTOTUNE', self.cmd_DOCK_AUTOTUNE,
                                        desc=self.cmd_DOCK_AUTOTUNE_help)
        # 'INTERRUPT' in the name: Klipper runs it without waiting for the running scan
        self.gcode.register_command('DOCK_AUTOTUNE_INTERRUPT', self.cmd_INTERRUPT,
                                    desc="Stop a running DOCK_AUTOTUNE after the current trial")

    @staticmethod
    def _has_macro(config, name):
        prefix = 'gcode_macro '
        return any(section.get_name()[len(prefix):].strip().upper() == name
                   for section in config.get_prefix_sections(prefix))

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
        self.last_samples = None
        if client is None:
            return None
        try:
            client.finish_measurements()
            self.last_samples = client.get_samples()
            return shock_metrics(self.last_samples)
        except Exception:
            logging.exception("dock_autotune: accelerometer read failed")
            return None

    # -- preparation like a print start ------------------------------------------

    def _detect(self):
        configfile = self.printer.lookup_object('configfile', None)
        settings = {}
        if configfile is not None:
            try:
                settings = configfile.get_status(self._now()).get('settings', {})
            except Exception:
                logging.exception("dock_autotune: cannot read the parsed config")
        return detect_setup(settings, set(getattr(self.gcode, 'ready_gcode_handlers', {})))

    def _make_plan(self, gcmd, setup, bed, soak):
        """Plan of the preparation. Touch homing as detected (TOUCH=0 turns it off, touch_command overrides)"""
        touch = setup['touch_cmd']
        if self.touch_command.lower() == 'none':
            touch = None
        elif self.touch_command.lower() != 'auto':
            touch = self.touch_command
        if gcmd.get_int('TOUCH', 1) == 0:
            touch = None
        if touch and self.printer.lookup_object('tool T%d' % self.touch_tool, None) is None:
            raise gcmd.error("DOCK_AUTOTUNE: touch homing needs tool T%d (touch_tool)" % self.touch_tool)
        return touch, build_plan(setup, bed, soak, touch, self.touch_tool, self.touch_temp,
                                 self.touch_repeats, self.before_touch, self.touch_restore)

    def _prepare(self, gcmd, plan):
        """Run the plan. A tool on the shuttle is put back first, which needs Z homed or a G28."""
        self.state['phase'] = 'prepare'
        gcmd.respond_info("dock_autotune: preparing like a print start:\n  " + '\n  '.join(
            p for p in plan if p != EMPTY_SHUTTLE))
        for entry in plan:
            self._check_abort()
            if entry == EMPTY_SHUTTLE:
                status = self._tc_status()
                if status['status'] != 'ready':
                    raise AbortScan('the toolchanger is %s after INITIALIZE_TOOLCHANGER - check the shuttle'
                                    % status['status'])
                if status['tool_number'] != -1:
                    toolhead = self.printer.lookup_object('toolhead')
                    if not all(a in toolhead.get_status(self._now())['homed_axes'] for a in 'xyz'):
                        self._run('G28')
                    self._run("UNSELECT_TOOL %s" % RESTORE_NONE)
                    self._wait()
                continue
            self.state['message'] = entry
            self._run(entry)
        self._wait()
        gcmd.respond_info("dock_autotune: preparation done")

    def _prompt(self, gcmd, number, setup, plan, bed, touch):
        for text in build_prompt(number, gcmd.get_command_parameters(), setup, plan, bed, touch):
            self.gcode.respond_raw(text)
        gcmd.respond_info("dock_autotune: choose in the prompt, or call DOCK_AUTOTUNE again with PREPARE=1 "
                          "(prepare) or PREPARE=0 (the machine is prepared)")

    # -- one pickup + dropoff ---------------------------------------------------

    def _cycle(self, tool_number):
        """One pickup and dropoff. Returns (picked, error message, shock metrics)."""
        self._check_abort()
        self.cycles += 1
        if self.cycles > self.max_cycles:
            raise AbortScan('max_cycles (%d) reached' % self.max_cycles)
        client = self._adxl_start(tool_number)
        self.last_pick_end = None
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
            if client is not None:
                # where the pickup ends and the dropoff begins, for the ADXL map
                toolhead = self.printer.lookup_object('toolhead')
                toolhead.wait_moves()
                self.last_pick_end = toolhead.get_last_move_time()
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
        peak = max(shocks) if shocks else None
        self._log({'tool': tool.tool_number, 'axis': axis, 'value': round(value, 4), 'ok': ok,
                   'peak_g': peak, 'error': error})
        self.trial_rows.append({'axis': axis, 'value': round(value, 4), 'ok': ok, 'peak_g': peak})
        return ok

    # -- the scan -----------------------------------------------------------------

    def _scan_tool(self, gcmd, tool, axes, limits, step, resolution):
        number = tool.tool_number
        base = {a: float(tool.params['params_park_%s' % a]) for a in axes}
        result = self.scan_result
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
                              step, limits[axis], self.refine, resolution)
            center = window_center(edges)
            entry = {'start': base[axis], 'plus': edges['plus'], 'minus': edges['minus'],
                     'plus_open': edges['plus_open'], 'minus_open': edges['minus_open'],
                     'width': edges['plus'] - edges['minus'],
                     'proposal': None if center is None else round(base[axis] + center, 3)}
            result[axis] = entry
            if entry['proposal'] is None:
                # the window is wider than the range: keep the old value for this axis
                tool.set_parameter('params_park_%s' % axis, base[axis])
                gcmd.respond_info("dock_autotune T%d %s: window not closed, value kept. %s side %s, %s side %s. Every "
                                  "trial up to the open side locked: the window is wider than the range, raise "
                                  "RANGE (and STEP, for example RANGE=1.2 STEP=0.2)"
                                  % (number, axis.upper(), '-', describe_edge(edges['minus'], edges['minus_open']),
                                     '+', describe_edge(edges['plus'], edges['plus_open'])))
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

    # -- the ADXL map ---------------------------------------------------------------

    def _map_cycle(self, tool_number):
        """One pickup and dropoff with the accelerometer. Returns (picked, error, pickup knock, dropoff knock,
        csv lines of the pickup)"""
        picked, error, _ = self._cycle(tool_number)
        samples = self.last_samples
        if not samples:
            return picked, error, None, None, []
        start, end = samples[0][0], samples[-1][0]
        mid = self.last_pick_end if (picked and self.last_pick_end is not None) else end
        pick = knock_metrics(samples, start, mid, self.adxl_window)
        drop = knock_metrics(samples, mid, end, self.adxl_window) if picked else None
        snippet = csv_snippet(samples, start, mid, pick['t_peak'], window=self.adxl_window) if pick else []
        return picked, error, pick, drop, snippet

    def _map_trial(self, tool, axis, offset, base, repeats, raw):
        """Set the park value and take `repeats` pickups with the accelerometer"""
        value = round(base + offset, 4)
        tool.set_parameter('params_park_%s' % axis, value)
        self.state['trial'] += 1
        picks, drops, locked, error = [], [], 0, ''
        for repeat in range(repeats):
            picked, error, pick, drop, snippet = self._map_cycle(tool.tool_number)
            if picked:
                locked += 1
            if pick:
                picks.append(pick)
            if drop:
                drops.append(drop)
            if raw and snippet:
                self._save_snippet(tool.tool_number, axis, offset, repeat + 1, snippet)
            if not picked:
                break
        row = {'offset': round(offset, 4), 'value': value, 'locked': locked, 'total': repeats,
               'ok': locked == repeats, 'error': error, 'pick': None, 'drop': None}
        for name, items in (('pick', picks), ('drop', drops)):
            if items:
                row[name] = {key: round(median([i[key] for i in items]), 3)
                             for key in ('peak_g', 'x', 'y', 'z', 't_peak', 'rms_g')}
        return row

    def _save_snippet(self, number, axis, offset, repeat, lines):
        try:
            os.makedirs(self.raw_dir, exist_ok=True)
            name = 'T%d_%s_%+.3f_r%d_pickup.csv' % (number, axis, offset, repeat)
            with open(os.path.join(self.raw_dir, name), 'w') as f:
                f.write('\n'.join(lines) + '\n')
        except OSError:
            logging.exception("dock_autotune: cannot write the ADXL snippet")

    def _map_tool(self, gcmd, tool, axes, limits, step, repeats, raw):
        """Map the knock of the pickup over a grid of positions: 0, +step, -step, +2 step, ... Positions beyond
        `map_fail_stop` failed ones in a row on one side are skipped. Nothing is applied."""
        number = tool.tool_number
        base = {a: float(tool.params['params_park_%s' % a]) for a in axes}
        result = self.map_rows
        for axis in axes:
            self._check_abort()
            self.state['axis'] = axis
            self.state['phase'] = 'map %s' % axis
            steps = int(limits[axis] / step + 1e-9)
            order = [0.0]
            for k in range(1, steps + 1):
                order.extend([round(k * step, 6), round(-k * step, 6)])
            rows, fails, stopped = [], {1: 0, -1: 0}, {1: False, -1: False}
            result[axis] = rows
            for offset in order:
                sign = 1 if offset > 0 else -1
                if offset != 0 and stopped[sign]:
                    continue
                self._check_abort()
                row = self._map_trial(tool, axis, offset, base[axis], repeats, raw)
                rows.append(row)
                self._log({'tool': number, 'axis': axis, 'map': row})
                if offset == 0 and not row['ok']:
                    raise AbortScan("baseline %s=%.3f does not work (tool not in its dock, or the dock must be "
                                    "adjusted by hand first)" % (axis.upper(), base[axis]))
                if offset != 0:
                    fails[sign] = 0 if row['ok'] else fails[sign] + 1
                    stopped[sign] = fails[sign] >= self.map_fail_stop
            tool.set_parameter('params_park_%s' % axis, base[axis])
            gcmd.respond_info('\n'.join(map_table(number, axis, rows, self.adxl_window)))
        return base, result

    # -- the seat test ----------------------------------------------------------------

    def _seat_params(self, tool, factor=1.0):
        """What the pickup_gcode uses: park position, speeds, safe and close Y, and the lift of the pickup path"""
        params = tool.params
        try:
            geo = parse_lift(params['params_pickup_path'])
        except (ValueError, TypeError, KeyError) as e:
            raise AbortScan("T%d: %s" % (tool.tool_number, e))
        try:
            return {'geo': geo, 'x': float(params['params_park_x']), 'y': float(params['params_park_y']),
                    'z': float(params['params_park_z']), 'fast': float(params['params_fast_speed']),
                    'close_y': float(params['params_close_y']), 'safe_y': float(params['params_safe_y']),
                    'path_speed': float(params['params_path_speed']) * factor}
        except (ValueError, TypeError, KeyError) as e:
            raise AbortScan("T%d: parameter %s is missing or not a number" % (tool.tool_number, e))

    def _seat_accel(self):
        """Acceleration of a Z move: the lower one of max_accel and max_z_accel"""
        toolhead = self.printer.lookup_object('toolhead')
        accel = float(toolhead.get_status(self._now()).get('max_accel', 500.))
        try:
            accel = min(accel, float(getattr(toolhead.get_kinematics(), 'max_z_accel', accel)))
        except Exception:
            pass
        return max(accel, 1.0)

    def _seat_plan(self, sp):
        enter = seat_enter_lines(sp['geo'], sp['x'], sp['y'], sp['z'], sp['close_y'], sp['fast'], sp['path_speed'])
        leave = seat_exit_lines(sp['geo'], sp['x'], sp['y'], sp['z'], sp['safe_y'], sp['fast'], sp['path_speed'])
        return enter, leave

    def _seat_hub(self, tool, sp, dx, dy, slow_from):
        """One lift into the shuttle up to the verify waypoint and back down with the accelerometer on. slow_from:
        height (relative to the park position) where the slow part starts, None: the whole lift at path speed.
        Returns pin (tool detected at the top), events (with their height), release_g (knock while it goes down)"""
        number = tool.tool_number
        toolhead = self.printer.lookup_object('toolhead')
        low, top = sp['geo']['low'], sp['geo']['top']
        self._check_abort()
        self.cycles += 1
        if self.cycles > self.max_cycles:
            raise AbortScan('max_cycles (%d) reached' % self.max_cycles)
        self._run('G0 X%.4f Y%.4f F%g' % (sp['x'] + dx + low['x'], sp['y'] + dy + low['y'], sp['path_speed']))
        self._wait()
        client = self._adxl_start(number)
        if client is None:
            raise AbortScan('the accelerometer of T%d does not start' % number)
        toolhead.dwell(0.3)
        accel = self._seat_accel()
        t_quiet = toolhead.get_last_move_time()
        segments, z_now = [], sp['z'] + low['z']

        def lift(z_rel, speed):
            nonlocal z_now
            target = sp['z'] + z_rel
            self._run('G0 Z%.4f F%g' % (target, speed))
            segments.append({'z_start': z_now, 'z_end': target, 'v': speed / 60.0, 'a': accel,
                             't_end': toolhead.get_last_move_time(),
                             'dur': move_duration(target - z_now, speed / 60.0, accel)})
            z_now = target

        coarse = sp['path_speed'] * top['f']
        if slow_from is None:
            lift(top['z'], coarse)
        else:
            if slow_from > low['z'] + 0.05:
                lift(slow_from, coarse)
            lift(top['z'], min(self.seat_speed * 60.0, coarse))
        toolhead.dwell(0.3)
        t_hold = toolhead.get_last_move_time()
        toolhead.wait_moves()
        pin = self._tc_status().get('detected_tool_number') == number
        self._run('G0 Z%.4f F%g' % (sp['z'] + low['z'], sp['path_speed']))
        t_low = toolhead.get_last_move_time()
        self._wait()
        self._adxl_finish(client)
        samples = self.last_samples
        if not samples:
            raise AbortScan('no accelerometer samples from T%d' % number)
        top_z = sp['z'] + top['z']
        events, noise, limit = find_events(samples, samples[0][0] + 0.1, t_hold, t_quiet, self.adxl_window,
                                           self.seat_threshold)
        events = [dict(e, z=event_height(e['t'], segments), rel=event_height(e['t'], segments) - top_z)
                  for e in events]
        release = knock_metrics(samples, t_hold, t_low, self.adxl_window)
        return {'pin': pin, 'events': events, 'noise_g': noise, 'limit_g': limit,
                'release_g': release['peak_g'] if release else None}

    def _seat_position(self, tool, sp, axis, offset, repeats, slow_from):
        dx, dy = (offset, 0.0) if axis == 'x' else (0.0, offset)
        hubs = [self._seat_hub(tool, sp, dx, dy, slow_from) for _ in range(repeats)]
        value = sp['x'] + offset if axis == 'x' else sp['y'] + offset
        row = seat_row(offset, value, hubs)
        self._log({'tool': tool.tool_number, 'axis': axis or 'z', 'seat': row})
        return row

    def _seat_start_check(self, sp):
        status = self._tc_status()
        if status.get('detected_tool_number', -1) != -1:
            raise AbortScan('a tool is detected on the shuttle (T%s): the shuttle has to be empty'
                            % status.get('detected_tool_number'))
        # anywhere outside of the dock is fine, the entry starts with the move to close_y like the pickup_gcode; inside
        # of the dock (nearer than the start of the pickup path) the first move could hit it
        position = self.printer.lookup_object('toolhead').get_status(self._now())['position']
        nearest = sp['y'] + sp['geo']['approach'][0]['y'] + 5.0
        if position[1] < nearest:
            raise AbortScan('the shuttle is at Y=%.1f, inside of the dock (the pickup path starts at Y=%.1f): move it '
                            'out to Y=%.1f first' % (position[1], nearest - 5.0, sp['safe_y']))

    def _seat_leave(self, sp):
        _, leave = self._seat_plan(sp)
        for line in leave:
            self._run(line)
        self.printer.lookup_object('toolhead').wait_moves()

    def _seat_tool(self, gcmd, tool, axes, limits, step, repeats):
        """The seat test: the shuttle goes under the tool, lifts into it up to the verify waypoint and back down,
        for the current dock position (first at path speed, finds the contact) and then for positions along X and
        Y, slowly through the contact. The tool never leaves its dock. Nothing is applied."""
        number = tool.tool_number
        sp = self._seat_params(tool)
        self._seat_start_check(sp)
        top_z = sp['z'] + sp['geo']['top']['z']
        info = self.seat_info
        info.update({'park_z': sp['z'], 'verify_z': top_z, 'speed': self.seat_speed, 'coarse': None,
                     'slow_from_z': None, 'rows': self.seat_rows})
        enter, _ = self._seat_plan(sp)
        for line in enter:
            self._run(line)
        self._wait()
        try:
            self.state['axis'] = None
            self.state['phase'] = 'seat coarse'
            coarse = self._seat_hub(tool, sp, 0.0, 0.0, None)
            info['coarse'] = coarse
            self._log({'tool': number, 'seat_coarse': {'pin': coarse['pin'], 'events': coarse['events'],
                                                       'noise_g': coarse['noise_g'], 'limit_g': coarse['limit_g']}})
            if not coarse['events']:
                raise AbortScan("no knock in the lift into the shuttle (threshold %.2f g, noise %.2f g): is T%d in "
                                "its dock, does its accelerometer work? (seat_threshold lowers the threshold)"
                                % (coarse['limit_g'], coarse['noise_g'], number))
            first_z = min(e['z'] for e in coarse['events'])
            slow_rel = max(sp['geo']['low']['z'], first_z - sp['z'] - self.seat_zone)
            info['slow_from_z'] = sp['z'] + slow_rel
            gcmd.respond_info("dock_autotune T%d: first lift at path speed knocks at Z %s; the slow part starts at "
                              "Z %.3f (%g mm/s)" % (number, ', '.join('%.3f' % e['z'] for e in coarse['events']),
                                                    info['slow_from_z'], self.seat_speed))
            self.state['phase'] = 'seat center'
            center = self._seat_position(tool, sp, None, 0.0, repeats, slow_rel)
            for axis in axes:
                self._check_abort()
                self.state['axis'] = axis
                self.state['phase'] = 'seat %s' % axis
                rows = self.seat_rows.setdefault(axis, [])
                rows.append(dict(center, value=sp[axis]))
                steps = int(limits[axis] / step + 1e-9)
                order = []
                for k in range(1, steps + 1):
                    order.extend([round(k * step, 6), round(-k * step, 6)])
                fails, stopped = {1: 0, -1: 0}, {1: False, -1: False}
                for offset in order:
                    sign = 1 if offset > 0 else -1
                    if stopped[sign]:
                        continue
                    self._check_abort()
                    row = self._seat_position(tool, sp, axis, offset, repeats, slow_rel)
                    rows.append(row)
                    fails[sign] = 0 if row['ok'] else fails[sign] + 1
                    stopped[sign] = fails[sign] >= self.map_fail_stop
                gcmd.respond_info('\n'.join(seat_table(number, axis, rows, self.adxl_window)))
            if not axes:
                self.seat_rows['z'] = [center]
        except AbortScan:
            try:
                self._seat_leave(sp)
            except self.gcode.error:
                logging.exception("dock_autotune: the shuttle could not leave the dock")
            raise
        self._seat_leave(sp)
        gcmd.respond_info('\n'.join(seat_summary(info)))
        return sp

    # -- commands -------------------------------------------------------------------

    cmd_DOCK_AUTOTUNE_help = ("Find the window in which a tool locks and centre its dock position: "
                              "DOCK_AUTOTUNE T=N [AXES=xyz] [STEP=0.1] [RANGE=MM] [REPEATS=2] "
                              "[RESOLUTION=0.05] [SPEED=0.5] [ADXL=1] [MODE=scan|map|seat] [RAW=1] [PREPARE=0|1] "
                              "[BED=70] [SOAK=SECONDS] [TOUCH=0] [DRY_RUN=1]")

    def cmd_DOCK_AUTOTUNE(self, gcmd):
        if self.active:
            raise gcmd.error("DOCK_AUTOTUNE is already running (DOCK_AUTOTUNE_INTERRUPT stops it)")
        number = gcmd.get_int('T', minval=0)
        axes = ''.join(a for a in gcmd.get('AXES', self.axes).lower() if a in AXES)
        mode = gcmd.get('MODE', self.mode).strip().lower()
        if mode not in ('scan', 'map', 'seat'):
            raise gcmd.error("MODE must be scan, map or seat")
        if mode == 'seat':
            # the height of the contact is measured at every position; only X and Y are moved
            axes = ''.join(a for a in axes if a in 'xy')
        elif not axes:
            raise gcmd.error("AXES must contain x, y or z")
        mode_map = mode == 'map'
        step = gcmd.get_float('STEP', {'map': self.map_step, 'seat': self.seat_step}.get(mode, self.step), above=0.)
        resolution = gcmd.get_float('RESOLUTION', self.resolution, above=0.)
        limits = {'map': {a: self.map_range for a in AXES},
                  'seat': {a: self.seat_range for a in AXES}}.get(mode, dict(self.range))
        if gcmd.get('RANGE', None) is not None:
            limits = {a: gcmd.get_float('RANGE', above=0.) for a in AXES}
        for a in AXES:
            limits[a] = gcmd.get_float('RANGE_%s' % a.upper(), limits[a], above=0.)
        repeats = gcmd.get_int('REPEATS', {'map': self.map_repeats, 'seat': self.seat_repeats}.get(mode, self.repeats),
                               minval=1)
        speed = gcmd.get_float('SPEED', self.speed_factor, above=0., maxval=1.)
        raw = gcmd.get_int('RAW', 1 if self.raw_default else 0) == 1
        self.use_adxl = gcmd.get_int('ADXL', 1 if self.adxl_default else 0) == 1 or mode in ('map', 'seat')
        if self.use_adxl and self.printer.lookup_object(self.accel_chip.format(tool=number), None) is None:
            if mode in ('map', 'seat'):
                raise gcmd.error("DOCK_AUTOTUNE: MODE=%s needs the accelerometer [%s]"
                                 % (mode, self.accel_chip.format(tool=number)))
            gcmd.respond_info("dock_autotune: ADXL=1 but there is no [%s] section - scanning without the "
                              "accelerometer" % self.accel_chip.format(tool=number))
        dry_run = gcmd.get_int('DRY_RUN', 0) == 1
        tool = self.printer.lookup_object('tool T%d' % number, None)
        if tool is None:
            raise gcmd.error("DOCK_AUTOTUNE: no tool T%d" % number)
        self._check_not_printing(gcmd)

        # PREPARE=1: bring the machine into the state of a print start first, PREPARE=0: it is
        # prepared already, without PREPARE the prompt asks (not for DRY_RUN)
        prepare = gcmd.get_int('PREPARE', self.prepare_default)
        bed = gcmd.get_float('BED', self.bed_temp, minval=0.)
        soak = gcmd.get_float('SOAK', self.bed_soak, minval=0.)
        plan = []
        if prepare != 0:
            setup = self._detect()
            if setup['max_bed'] and bed > setup['max_bed']:
                raise gcmd.error("DOCK_AUTOTUNE: BED=%g is above max_temp %g of the heater_bed"
                                 % (bed, setup['max_bed']))
            touch, plan = self._make_plan(gcmd, setup, bed, soak)
            if prepare is None and not dry_run:
                self._prompt(gcmd, number, setup, plan, bed, touch)
                return
            if prepare == 1 and not setup['can_prepare']:
                raise gcmd.error("DOCK_AUTOTUNE: preparation is not possible: %s. Prepare the machine by hand "
                                 "and use PREPARE=0" % setup['reason'])
            if not setup['can_prepare']:        # DRY_RUN only
                gcmd.respond_info("dock_autotune: preparation is not possible: %s" % setup['reason'])
                plan = []
            else:
                gcmd.respond_info("dock_autotune: probe %s, touch homing %s, levelling %s"
                                  % (setup['probe'] or 'none (Z endstop)', touch or 'none',
                                     setup['level_cmd'] or 'none'))
        prepare = prepare == 1
        if not prepare and not (dry_run and plan):
            self._check_state(gcmd, number)

        per_axis = {a: int(limits[a] / step + 1e-9) for a in axes}
        bisect = max(0, int(math.ceil(math.log(step / resolution, 2) - 1e-9))) if self.refine else 0
        if mode == 'seat':
            positions = 1 + sum(2 * n for n in per_axis.values())
            gcmd.respond_info("dock_autotune T%d seat test, axes %s, step %.3f mm, range %s, %d repeat(s): up to "
                              "%d lifts at %d positions, the tool stays in its dock; current dock position "
                              "x=%.3f y=%.3f z=%.3f"
                              % (number, axes.upper() or '-', step, ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                                 repeats, 1 + positions * repeats, positions,
                                 *[float(tool.params['params_park_%s' % a]) for a in 'xyz']))
        else:
            if mode == 'map':
                trials = sum(2 * n + 1 for n in per_axis.values())
            else:
                trials = sum(2 * n + 2 * bisect + 2 for n in per_axis.values())
            gcmd.respond_info("dock_autotune T%d %s, axes %s, step %.3f mm, range %s, %d repeat(s): about %d trials "
                              "(%d pickups) at most, current dock position x=%.3f y=%.3f z=%.3f"
                              % (number, 'ADXL map' if mode == 'map' else 'scan', axes.upper(), step,
                                 ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                                 repeats, trials, trials * repeats,
                                 *[float(tool.params['params_park_%s' % a]) for a in 'xyz']))
        if dry_run:
            if plan:
                gcmd.respond_info("DRY_RUN: PREPARE=1 would run first:\n  " + '\n  '.join(
                    p for p in plan if p != EMPTY_SHUTTLE))
            if mode == 'seat':
                try:
                    sp = self._seat_params(tool, speed)
                except AbortScan as e:
                    raise gcmd.error("DOCK_AUTOTUNE: %s" % e)
                enter, leave = self._seat_plan(sp)
                low, top = sp['geo']['low'], sp['geo']['top']
                gcmd.respond_info(
                    "DRY_RUN: the empty shuttle moves like this (G-code, like the pickup_gcode up to the verify "
                    "waypoint):\n  " + '\n  '.join(enter) +
                    "\nat every position (X or Y shifted at the lowest point), the accelerometer on:\n"
                    "  G0 Z%.4f F%g   lift to the verify waypoint (the first lift; the later ones go at this speed up to "
                    "%g mm below the first knock, then at %g mm/s)\n"
                    "  hold 0.3 s, read the tool detection, then G0 Z%.4f F%g   down again\n"
                    "and at the end:\n  " % (sp['z'] + top['z'], sp['path_speed'] * top['f'], self.seat_zone,
                                             self.seat_speed, sp['z'] + low['z'], sp['path_speed']) +
                    '\n  '.join(leave))
            gcmd.respond_info("DRY_RUN: nothing moved")
            return
        self.repeats, saved_repeats = repeats, self.repeats
        self._start(number)
        toolhead = self.printer.lookup_object('toolhead')
        saved_accel = toolhead.get_status(self._now())['max_accel']
        saved_speed = float(tool.params['params_path_speed'])
        originals = {a: float(tool.params['params_park_%s' % a]) for a in AXES}
        proposals, base = {}, {}
        status = 'stopped'
        settings = ['step %.3f mm' % step, 'range ' + ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                    '%d repeat(s)' % repeats, 'speed factor %g' % speed,
                    'acceleration %g' % min(self.accel, saved_accel)]
        if mode in ('map', 'seat'):
            settings.append('high-pass %d ms' % int(round(self.adxl_window * 1000)))
        if mode == 'seat':
            settings.append('speed at the contact %g mm/s' % self.seat_speed)
        settings.append('machine prepared by DOCK_AUTOTUNE with bed %g C' % bed if prepare
                        else 'machine was prepared before')
        try:
            if prepare:
                self._prepare(gcmd, plan)
                self._check_state(gcmd, number)
            self.state['phase'] = 'start'
            self._run("SET_VELOCITY_LIMIT ACCEL=%g" % min(self.accel, saved_accel))
            tool.set_parameter('params_path_speed', saved_speed * speed)
            if self._tc_status()['tool_number'] == number:
                self._run("UNSELECT_TOOL %s" % RESTORE_NONE)
                self._wait()
            if mode == 'map':
                base, result = self._map_tool(gcmd, tool, axes, limits, step, repeats, raw)
                self.result = {'tool': number, 'map': result}
                gcmd.respond_info("dock_autotune T%d ADXL map finished after %d pickups, the park values are "
                                  "unchanged%s" % (number, self.cycles,
                                                   ", CSV snippets in %s" % self.raw_dir if raw else ""))
                status = 'finished'
            elif mode == 'seat':
                self._seat_tool(gcmd, tool, axes, limits, step, repeats)
                self.result = {'tool': number, 'seat': self.seat_rows}
                gcmd.respond_info("dock_autotune T%d seat test finished after %d lifts, the park values are "
                                  "unchanged" % (number, self.cycles))
                status = 'finished'
            else:
                base, result = self._scan_tool(gcmd, tool, axes, limits, step, resolution)
                proposals = {a: base[a] for a in axes if result[a]['proposal'] is not None}
                self.result = {'tool': number, 'axes': result}
                self._finish_ok(gcmd, number, originals, proposals, result)
                status = 'finished'
        except AbortScan as e:
            status = 'STOPPED: %s' % e
            self._abort_report(gcmd, number, e)
        except self.gcode.error as e:
            status = 'STOPPED: %s' % e
            self._abort_report(gcmd, number, e)
            raise
        finally:
            path = self._write_report(number, mode, status, originals, proposals, settings,
                                      self.raw_dir if raw and mode_map else None)
            if path:
                gcmd.respond_info("dock_autotune: result file %s" % path)
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

    def _write_report(self, number, mode, status, originals, proposals, settings, raw_dir):
        """Writes the result file (park positions, ADXL measurements, CSV) to results_dir, returns its path"""
        seat = self.seat_info if (self.seat_rows or self.seat_info.get('coarse')) else None
        if not self.results_dir or not (self.trial_rows or any(self.map_rows.values()) or seat):
            return None
        info = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'tool': number, 'mode': mode, 'status': status,
                'pickups': self.cycles, 'settings': settings, 'originals': originals, 'new': proposals,
                'scan': self.scan_result, 'trials': self.trial_rows, 'map': self.map_rows, 'seat': seat,
                'window': self.adxl_window, 'raw_dir': raw_dir}
        path = os.path.join(self.results_dir, 'T%d_%s_%s.txt' % (number, mode, time.strftime('%Y%m%d_%H%M%S')))
        try:
            os.makedirs(self.results_dir, exist_ok=True)
            with open(path, 'w') as f:
                f.write(build_report(info))
        except (OSError, KeyError, TypeError, ValueError):
            logging.exception("dock_autotune: cannot write the result file %s", path)
            return None
        return path

    def _check_not_printing(self, gcmd):
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(self._now()).get('state') in ('printing', 'paused'):
            raise gcmd.error("DOCK_AUTOTUNE: not while a print is running or paused")

    def _check_state(self, gcmd, number):
        toolhead = self.printer.lookup_object('toolhead')
        if not all(a in toolhead.get_status(self._now())['homed_axes'] for a in 'xyz'):
            raise gcmd.error("DOCK_AUTOTUNE: home all axes first (same way as before a print), or let "
                             "DOCK_AUTOTUNE prepare the machine (PREPARE=1)")
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
        self.trial_rows, self.map_rows, self.scan_result = [], {}, {}
        self.seat_rows, self.seat_info = {}, {}
        self.state.update({'tool': number, 'axis': None, 'phase': 'start', 'trial': 0, 'message': ''})

    def _finish_ok(self, gcmd, number, originals, proposals, result):
        lines = ["dock_autotune T%d finished after %d pickups. %s" % (
            number, self.cycles, "Applied until the next restart:" if proposals else "Nothing was changed:")]
        for axis in AXES:
            if axis in result:
                e = result[axis]
                if e['proposal'] is None:
                    if e['plus_open'] or e['minus_open']:
                        reason = 'window not closed, at least %.2f mm wide: %s .. %s' % (
                            e['width'], describe_edge(e['minus'], e['minus_open']),
                            describe_edge(e['plus'], e['plus_open']))
                    else:
                        reason = 'the centre of the window did not lock'
                    lines.append("  params_park_%s: %.3f   (unchanged, %s)" % (axis, originals[axis], reason))
                else:
                    lines.append("  params_park_%s: %.3f   (was %.3f, window %.3f wide)" % (
                        axis, e['proposal'], originals[axis], e['width']))
        if proposals:
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
