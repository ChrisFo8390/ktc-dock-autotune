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
# MODE=seat (the default) does not pick the tool up: the empty shuttle lifts into the tool in its dock up to
# the verify waypoint and down again, the accelerometer of the tool measures where the contact is and how it
# is pushed sideways. MODE=scan looks for the window of positions in which the tool locks and centres it.
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


# ---------------------------------------------------------------------------
# Accelerometer: what the tool sees while the shuttle takes it up (pure, used by the tests)
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


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return None
    return values[n // 2] if n % 2 else (values[n // 2 - 1] + values[n // 2]) / 2.0


def _park_result(axis, info):
    """(after value, text) of one axis for the report"""
    before = info['originals'][axis]
    entry = info.get('scan', {}).get(axis)
    if info['mode'] == 'seat':
        if axis == 'z':
            return before, 'height of the contact measured, not changed by the seat test'
        proposal = (info.get('seat') or {}).get('proposals', {}).get(axis)
        if proposal and proposal['value'] is not None:
            return before, 'not changed, the quiet window suggests %.3f' % proposal['value']
        return before, ('measured, not changed by the seat test' if axis in info.get('seat', {}).get('rows', {})
                        else 'not measured')
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
    The result file of one run as text: the park positions (before, after), the measurements of the seat test or the
    trials of the scan, and all numbers as CSV. info: time, tool, mode, status, pickups, settings (list of text),
    originals, new, scan (result per axis), trials, seat (rows per axis and the height of the lift), window
    """
    tool = info['tool']
    lines = ["Dock autotune T%d - %s - %s" % (tool, 'seat test' if info['mode'] == 'seat' else 'scan', info['time']),
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
        suggestions = seat_suggestions(info['seat'], tool)
        if suggestions:
            lines.append("")
            lines.extend(suggestions)
        for axis in AXES:
            if info['seat']['rows'].get(axis):
                lines.append("")
                lines.extend(seat_table(tool, axis, info['seat']['rows'][axis], info['window']))
    for axis in AXES:
        trials = [t for t in info.get('trials', []) if t['axis'] == axis]
        if trials:
            start = info['originals'][axis]
            lines.extend(["", "Scan trials %s in the order they were made (offset from the start value %.3f)"
                          % (axis.upper(), start), " offset      value  result"])
            for t in trials:
                lines.append("%+7.3f  %9.3f  %s" % (t['value'] - start, t['value'], 'locked' if t['ok'] else 'FAILED'))
    lines.extend(["", "Data (CSV)"])
    if info['mode'] == 'seat':
        lines.append("tool,axis,offset_mm,value_mm,pin_ok,pin_total,events,z_main,peak_all_g,energy_g2s,z_first,z_last,"
                     "peak_first_g,peak_last_g,lateral_share,ix_mm_s,iy_mm_s,release_g")
        for axis in AXES:
            for r in sorted(info.get('seat', {}).get('rows', {}).get(axis, []), key=lambda r: r['offset']):
                lines.append(','.join('' if v is None else str(v) for v in (
                    tool, axis, r['offset'], r['value'], r['pin'], r['total'], r['events'], r.get('z_main'),
                    r.get('peak_all'), r.get('energy'), r['z_first'], r['z_last'], r['peak_first'], r['peak_last'],
                    r['lateral'], r['ix'], r['iy'], r['release_g'])))
    else:
        lines.append("tool,axis,offset_mm,value_mm,locked")
        for t in info.get('trials', []):
            lines.append(','.join(str(v) for v in (
                tool, t['axis'], round(t['value'] - info['originals'][t['axis']], 4), t['value'],
                1 if t['ok'] else 0)))
    return '\n'.join(lines) + '\n'


def parse_tool_list(text, known=None):
    """'0,2', '0-4' or 'all' (every number in `known`) -> tool numbers in the order given, without repeats"""
    text = text.strip().lower()
    if text == 'all':
        if not known:
            raise ValueError("TOOLS=all: the toolchanger does not know its tools")
        return sorted(known)
    numbers = []
    for item in text.replace(' ', '').split(','):
        if not item:
            continue
        try:
            if '-' in item:
                low, high = (int(v) for v in item.split('-', 1))
                if high < low:
                    raise ValueError(item)
                values = range(low, high + 1)
            else:
                values = [int(item)]
        except ValueError:
            raise ValueError("cannot read '%s' in TOOLS (use 0,1,2 or 0-4 or all)" % item)
        for value in values:
            if value not in numbers:
                numbers.append(value)
    if not numbers:
        raise ValueError("TOOLS is empty (use 0,1,2 or 0-4 or all)")
    return numbers


def build_multi_report(runs, numbers, mode, when):
    """The result file of a run over several tools: how each tool ended, the park positions of all of them (and
    the lines for the tool files), then the full report of every tool. runs: number, status, info (or None)"""
    names = {'seat': 'seat test'}
    lines = ["Dock autotune %s - %s - %s" % (', '.join('T%d' % n for n in numbers), names.get(mode, 'scan'), when),
             "Tools:"]
    done = {r['number']: r for r in runs}
    for n in numbers:
        lines.append("  T%-2d %s" % (n, done[n]['status'] if n in done else 'not run'))
    lines.extend(["", "Park positions" + (" (applied until the next restart)" if mode == 'scan' else ""),
                  "  tool  axis      before       after   result"])
    changed = {}
    for r in runs:
        info = r.get('info')
        if not info:
            lines.append("  T%-3d no result (%s)" % (r['number'], r['status']))
            continue
        for axis in AXES:
            after, text = _park_result(axis, info)
            lines.append("  T%-3d %-4s %10.3f  %10.3f   %s" % (r['number'], axis, info['originals'][axis], after, text))
            if after != info['originals'][axis]:
                changed.setdefault(r['number'], []).append("params_park_%s: %.3f" % (axis, after))
    lines.append("")
    if changed:
        lines.append("Put these lines into the tool files (toolchanger/tools/T0.cfg, T1.cfg, ...):")
        for n in sorted(changed):
            lines.append("  T%d:" % n)
            lines.extend("    " + c for c in changed[n])
    else:
        lines.append("No park position was changed.")
    suggested = [(r['number'], r['info']['seat']) for r in runs
                 if r.get('info') and r['info'].get('seat') and r['info']['seat'].get('proposals')]
    if suggested:
        lines.extend(["", "Suggested by the quiet windows of the seat test (nothing is applied, run the seat test again "
                          "around a new position):"])
        for n, seat in suggested:
            for axis in 'xy':
                p = seat['proposals'].get(axis)
                if p and p['value'] is not None:
                    lines.append("  T%d params_park_%s: %.3f   (%s)" % (n, axis, p['value'], p['text']))
                elif p:
                    lines.append("  T%d %s: %s" % (n, axis, p['text']))
    for r in runs:
        if r.get('info'):
            lines.extend(["", "=" * 78, ""] + build_report(r['info']).rstrip('\n').splitlines())
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
    e_x, e_y, e_z, i_x, i_y), release_g, energy_g2s and lift_peak_g (of the whole lift, also below the threshold)"""
    def med(values, digits=3):
        values = [v for v in values if v is not None]
        return round(median(values), digits) if values else None

    firsts = [min(h['events'], key=lambda e: e['z']) for h in hubs if h['events']]
    lasts = [max(h['events'], key=lambda e: e['z']) for h in hubs if h['events']]
    mains = [max(h['events'], key=lambda e: e['peak_g']) for h in hubs if h['events']]
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
            'z_main': med([e['z'] for e in mains]), 'peak_main': med([e['peak_g'] for e in mains]),
            'peak_first': med([e['peak_g'] for e in firsts]), 'peak_last': med([e['peak_g'] for e in lasts]),
            'energy': med([h.get('energy_g2s') for h in hubs], 4), 'peak_all': med([h.get('lift_peak_g') for h in hubs]),
            'lateral': med(shares),
            'ix': med([sum(e['i_x'] for e in h['events']) for h in hubs if h['events']], 2),
            'iy': med([sum(e['i_y'] for e in h['events']) for h in hubs if h['events']], 2),
            'release_g': med([h['release_g'] for h in hubs])}


def seat_crossings(rows, key):
    """Offsets at which the value `key` of the seat rows changes sign (linear between two positions)"""
    points = sorted((r['offset'], r[key]) for r in rows if r.get(key) is not None)
    return [round(o1 + (o2 - o1) * v1 / (v1 - v2), 3)
            for (o1, v1), (o2, v2) in zip(points, points[1:]) if v1 * v2 < 0]


def seat_proposal(axis, rows, quiet_part=0.25):
    """
    The quiet window along one axis: the positions where the lift is clearly calmer than at the others. The measure
    is the energy of the whole lift (every knock, also the small ones). A position is quiet if its energy is in the
    lowest quarter between the lowest and the highest one. The window is a run of quiet positions with a louder one on
    both sides; its centre is suggested. Returns state (ok, open, flat, no data), offset and value of the centre,
    the edges, and a text.
    """
    points = sorted((r['offset'], r['energy'], r['value']) for r in rows if r.get('energy') is not None)
    if len(points) < 4:
        return {'state': 'no data', 'text': 'too few positions measured', 'value': None}
    energies = [e for _, e, _ in points]
    low, high = min(energies), max(energies)
    if high < 2.0 * low:
        return {'state': 'flat', 'value': None,
                'text': 'no clear difference between the positions (energy %.3f .. %.3f)' % (low, high)}
    level = low + quiet_part * (high - low)
    runs, start = [], None
    for i, (_, e, _) in enumerate(points + [(0, high + 1, 0)]):
        if e <= level and start is None:
            start = i
        elif e > level and start is not None:
            runs.append((start, i - 1))
            start = None
    park = median([v - o for o, _, v in points])

    def edge(i, j):
        return (points[i][0] + points[j][0]) / 2.0

    closed = [(a, b) for a, b in runs if a > 0 and b < len(points) - 1]
    if closed:
        a, b = max(closed, key=lambda run: (points[run[1]][0] - points[run[0]][0], -sum(points[k][1] for k in range(run[0], run[1] + 1))))
        minus, plus = edge(a - 1, a), edge(b, b + 1)
        centre = (minus + plus) / 2.0
        return {'state': 'ok', 'offset': round(centre, 3), 'value': round(park + centre, 3), 'minus': round(minus, 3),
                'plus': round(plus, 3), 'text': 'quiet from %+.2f to %+.2f mm around the park position (%.2f mm wide), '
                                                 'centre %+.2f mm' % (minus, plus, plus - minus, centre)}
    a, b = max(runs, key=lambda run: points[run[1]][0] - points[run[0]][0])
    sides = []
    if a == 0:
        sides.append('minus')
    if b == len(points) - 1:
        sides.append('plus')
    return {'state': 'open', 'value': None, 'minus': points[a][0], 'plus': points[b][0],
            'text': 'quiet from %+.2f to %+.2f mm but no louder position on the %s side within the range: raise '
                    'seat_range' % (points[a][0], points[b][0], ' and '.join(sides))}


def seat_suggestions(seat, tool):
    """Lines with the park positions that the quiet windows suggest, and how to try them (nothing is applied)"""
    proposals = seat.get('proposals') or {}
    lines = []
    for axis in 'xy':
        p = proposals.get(axis)
        if p:
            lines.append("  %s: %s%s" % (axis, p['text'], ('  ->  params_park_%s: %.3f' % (axis, p['value']))
                                         if p['value'] is not None else ''))
    ok = [a for a in 'xy' if proposals.get(a) and proposals[a]['value'] is not None]
    if not lines:
        return []
    out = ["Suggested park positions (nothing is applied). The quiet window is the range of positions without a "
           "clear knock:"] + lines
    if ok:
        out.append("To try them without editing the tool file, then run the seat test again around the new position:")
        out.extend("  SET_TOOL_PARAMETER T=%d PARAMETER=params_park_%s VALUE=%.3f" % (tool, a, proposals[a]['value'])
                   for a in ok)
    return out


def seat_summary(seat):
    """Lines about the height of the lift: where it knocks, how far below the verify waypoint"""
    rows = [r for axis in AXES for r in seat['rows'].get(axis, [])]
    mains = [r['z_main'] for r in rows if r.get('z_main') is not None]
    top = seat['verify_z']
    lines = ["Lift into the shuttle: park Z %.3f, verify waypoint at Z %.3f" % (seat['park_z'], top)]
    coarse = seat.get('coarse')
    if coarse:
        if coarse['events']:
            main = max(coarse['events'], key=lambda e: e['peak_g'])
            lines.append("  first lift at path speed: the strongest knock is at Z %.3f (%.1f g), %.3f mm below the "
                         "verify waypoint" % (main['z'], main['peak_g'], top - main['z']))
            lines.append("  all knocks of the first lift: %s" % ', '.join(
                'Z %.3f (%.1f g)' % (e['z'], e['peak_g']) for e in coarse['events']))
        else:
            lines.append("  first lift at path speed: no knock")
        if seat.get('slow_from_z') is not None:
            lines.append("  the slow part of the later lifts starts at Z %.3f (%g mm/s)"
                         % (seat['slow_from_z'], seat['speed']))
    if mains:
        lines.append("  strongest knock of the later lifts: Z %.3f (median), %.3f .. %.3f over the positions"
                     % (median(mains), min(mains), max(mains)))
    return lines


def seat_table(tool, axis, rows, window):
    """The seat test of one axis as text lines"""
    lines = ["Seat test T%d %s: the lift into the shuttle at positions along the axis (g after a %d ms high-pass; "
             "z of the strongest knock in mm; energy = of the whole lift, g^2 s; lateral = share of the knock energy "
             "in x and y; ix iy = net velocity change in x and y in mm/s; release = knock while it goes down; "
             "medians of the repeats)" % (tool, axis.upper(), int(round(window * 1000))),
             " offset  pin  events   z_main   peak  energy  lateral      ix      iy  release"]

    def num(v, fmt):
        return fmt % v if v is not None else ' ' * len(fmt % 0)
    for r in sorted(rows, key=lambda r: r['offset']):
        lines.append("%+7.2f  %d/%d  %s  %s  %s  %s  %s  %s  %s  %s" % (
            r['offset'], r['pin'], r['total'], num(r['events'], '%6.1f'), num(r.get('z_main'), '%8.3f'),
            num(r.get('peak_all'), '%5.2f'), num(r.get('energy'), '%7.3f'), num(r['lateral'], '%7.2f'),
            num(r['ix'], '%7.1f'), num(r['iy'], '%7.1f'), num(r['release_g'], '%7.2f')))
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
            plan.append('M109 S%g' % touch_temp)       # heat up and wait first, the brush works on a hot nozzle
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

    out = [line('prompt_begin Dock autotune %s' % (number if isinstance(number, str) else 'T%d' % number))]
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
    out.append(line('prompt_button Prepare and start (bed %g C)|%s|info'
                    % (bed, command_line(params, PREPARE=1, BED='%g' % bed))))
    out.append(line('prompt_button The machine is already prepared|%s|warning'
                    % command_line(params, PREPARE=0, BED=None)))
    out.extend([line('prompt_footer_button Cancel|RESPOND TYPE=command MSG=action:prompt_end|secondary'),
                line('prompt_show')])
    return out


MODE_TEXTS = {'seat': 'seat test (the shuttle lifts into the tool without taking it out of its dock, nothing is '
                      'changed)',
              'scan': 'scan (finds the window in which the tool locks and centres the dock position)'}


def build_tool_prompt(params, numbers, mode='seat'):
    """Lines of a Mainsail / Fluidd action prompt that asks which tool to measure: one of the tools, or all"""
    def line(text):
        return '// action:' + text.replace('\n', ' ')

    out = [line('prompt_begin Dock autotune'),
           line('prompt_text Which tool should be measured? Mode: %s.' % MODE_TEXTS.get(mode, mode)),
           line('prompt_button_group_start')]
    for number in numbers:
        out.append(line('prompt_button T%d|%s|info' % (number, command_line(params, T=number))))
    out.append(line('prompt_button_group_end'))
    out.append(line('prompt_button All tools (%s)|%s|warning' % (', '.join('T%d' % n for n in numbers),
                                                                  command_line(params, TOOLS='all'))))
    out.extend([line('prompt_footer_button Cancel|RESPOND TYPE=command MSG=action:prompt_end|secondary'),
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
        self.touch_repeats = config.getint('touch_repeats', 1, minval=1)
        self.touch_restore = config.get('touch_restore_axis', 'Z')
        self.before_touch = [l.strip() for l in config.get('before_touch', '').splitlines() if l.strip()]
        # Accelerometer of the tool: "adxl345 T{tool}" (the section must be named per tool)
        self.accel_chip = config.get('accelerometer', 'adxl345 T{tool}')
        # seat test (MODE=seat): lift into the shuttle without leaving the dock. adxl_window is the window (s) of the
        # high-pass that removes gravity and the smooth acceleration of the moves, seat_speed the speed (mm/s) of the
        # part around the contact, which starts seat_zone mm below the first knock; the XY positions go seat_range mm
        # to both sides in seat_step steps with seat_repeats lifts each, a side ends after seat_fail_stop positions
        # in a row where the tool detection was off at the top
        self.adxl_window = config.getfloat('adxl_window', 0.05, above=0.)
        self.seat_fail_stop = config.getint('seat_fail_stop', 2, minval=1)
        self.seat_speed = config.getfloat('seat_speed', 2., above=0.)
        self.seat_zone = config.getfloat('seat_zone', 1.5, minval=0.)
        self.seat_range = config.getfloat('seat_range', 1., above=0.)
        self.seat_step = config.getfloat('seat_step', 0.25, above=0.)
        self.seat_repeats = config.getint('seat_repeats', 3, minval=1)
        self.seat_threshold = config.getfloat('seat_threshold', 0., minval=0.)
        # defaults of the command line: DOCK_AUTOTUNE T=0 alone does what these say
        self.mode = config.get('mode', 'seat').strip().lower()
        if self.mode not in ('scan', 'seat'):
            raise config.error("dock_autotune: mode must be seat or scan")
        self.axes = ''.join(a for a in config.get('axes', AXES).lower() if a in AXES) or AXES
        self.tools_default = config.get('tools', '').strip()
        prepare = config.get('prepare', 'ask').strip().lower()
        if prepare not in PREPARE_CHOICES:
            raise config.error("dock_autotune: prepare must be ask, yes or no")
        self.prepare_default = PREPARE_CHOICES[prepare]
        # the result file of every run (park positions + ADXL measurements), empty: no file
        self.results_dir = os.path.expanduser(
            config.get('results_dir', '~/printer_data/config/dock_autotune/results').strip())
        self.log_file = os.path.expanduser(
            config.get('log_file', '~/printer_data/logs/dock_autotune.jsonl'))
        self.active = False
        self.abort = False
        self.last_samples = None        # accelerometer samples of the last lift
        self.trial_rows = []            # data of the run for the result file
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

    # -- accelerometer (seat test) ------------------------------------------------

    def _adxl_start(self, tool_number):
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
            return
        try:
            client.finish_measurements()
            self.last_samples = client.get_samples()
        except Exception:
            logging.exception("dock_autotune: accelerometer read failed")

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
        """One pickup and dropoff. Returns (picked, error message)."""
        self._check_abort()
        self.cycles += 1
        if self.cycles > self.max_cycles:
            raise AbortScan('max_cycles (%d) reached' % self.max_cycles)
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
        return picked, error

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
        ok, error = True, ''
        for _ in range(self.repeats):
            picked, error = self._cycle(tool.tool_number)
            if not picked:
                ok = False
                break
        self._log({'tool': tool.tool_number, 'axis': axis, 'value': round(value, 4), 'ok': ok, 'error': error})
        self.trial_rows.append({'axis': axis, 'value': round(value, 4), 'ok': ok})
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
        energy, lift_peak = 0.0, 0.0
        part = [sm for sm in samples if t_quiet <= sm[0] <= t_hold]
        if len(part) >= 40:
            _, rate, (rx, ry, rz) = residuals(part, self.adxl_window)
            squares = [a * a + b * b + c * c for a, b, c in zip(rx, ry, rz)]
            energy, lift_peak = sum(squares) / rate, math.sqrt(max(squares))
        return {'pin': pin, 'events': events, 'noise_g': noise, 'limit_g': limit, 'energy_g2s': energy,
                'lift_peak_g': lift_peak, 'release_g': release['peak_g'] if release else None}

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
                    stopped[sign] = fails[sign] >= self.seat_fail_stop
                proposal = seat_proposal(axis, rows)
                info.setdefault('proposals', {})[axis] = proposal
                gcmd.respond_info('\n'.join(seat_table(number, axis, rows, self.adxl_window) +
                                            ["Quiet window %s: %s" % (axis.upper(), proposal['text'])]))
            if not axes:
                self.seat_rows['z'] = [center]
        except AbortScan:
            try:
                self._seat_leave(sp)
            except self.gcode.error:
                logging.exception("dock_autotune: the shuttle could not leave the dock")
            raise
        self._seat_leave(sp)
        gcmd.respond_info('\n'.join(seat_summary(info) + seat_suggestions(info, number)))
        return sp

    # -- commands -------------------------------------------------------------------

    cmd_DOCK_AUTOTUNE_help = ("Find the window in which a tool locks and centre its dock position: "
                              "DOCK_AUTOTUNE T=N [AXES=xyz] [STEP=0.1] [RANGE=MM] [REPEATS=2] "
                              "[RESOLUTION=0.05] [SPEED=0.5] [MODE=seat|scan] [PREPARE=0|1] "
                              "[BED=70] [SOAK=SECONDS] [TOUCH=0] [DRY_RUN=1]")

    def _tool_numbers(self, gcmd):
        """The tools of the call: T=n, or TOOLS=0,1,2 / 0-4 / all (or `tools` of the config)"""
        text, single = gcmd.get('TOOLS', None), gcmd.get('T', None)
        if text is not None and single is not None:
            raise gcmd.error("DOCK_AUTOTUNE: use T=n for one tool or TOOLS=0,1,2 for several, not both")
        if text is None and single is not None:
            return [gcmd.get_int('T', minval=0)]
        text = text if text is not None else self.tools_default
        if not text.strip():
            raise gcmd.error("DOCK_AUTOTUNE: T=n (one tool) or TOOLS=0,1,2 / 0-4 / all is required, for example "
                             "DOCK_AUTOTUNE T=1 DRY_RUN=1")
        try:
            return parse_tool_list(text, self._tc_status().get('tool_numbers'))
        except ValueError as e:
            raise gcmd.error("DOCK_AUTOTUNE: %s" % e)

    def _known_tools(self):
        numbers = self._tc_status().get('tool_numbers') or []
        if not numbers:
            numbers = [n for n in range(16) if self.printer.lookup_object('tool T%d' % n, None) is not None]
        return sorted(numbers)

    def _prompt_tools(self, gcmd):
        numbers = self._known_tools()
        if not numbers:
            raise gcmd.error("DOCK_AUTOTUNE: the toolchanger has no tools")
        mode = gcmd.get('MODE', self.mode).strip().lower()
        for text in build_tool_prompt(gcmd.get_command_parameters(), numbers, mode):
            self.gcode.respond_raw(text)
        gcmd.respond_info("dock_autotune: choose the tool in the prompt, or call DOCK_AUTOTUNE with T=n (one tool) "
                          "or TOOLS=0,1,2 / 0-4 / all")

    def cmd_DOCK_AUTOTUNE(self, gcmd):
        if self.active:
            raise gcmd.error("DOCK_AUTOTUNE is already running (DOCK_AUTOTUNE_INTERRUPT stops it)")
        if (gcmd.get('T', None) is None and gcmd.get('TOOLS', None) is None and not self.tools_default
                and gcmd.get_int('DRY_RUN', 0) != 1):
            # called from the macro list without a tool: ask in a prompt
            self._check_not_printing(gcmd)
            self._prompt_tools(gcmd)
            return
        numbers = self._tool_numbers(gcmd)
        multi = len(numbers) > 1
        label = numbers[0] if not multi else ', '.join('T%d' % n for n in numbers)
        axes = ''.join(a for a in gcmd.get('AXES', self.axes).lower() if a in AXES)
        mode = gcmd.get('MODE', self.mode).strip().lower()
        if mode not in ('scan', 'seat'):
            raise gcmd.error("MODE must be seat or scan")
        if mode == 'seat':
            # the height of the contact is measured at every position; only X and Y are moved
            axes = ''.join(a for a in axes if a in 'xy')
        elif not axes:
            raise gcmd.error("AXES must contain x, y or z")
        step = gcmd.get_float('STEP', self.seat_step if mode == 'seat' else self.step, above=0.)
        resolution = gcmd.get_float('RESOLUTION', self.resolution, above=0.)
        limits = {a: self.seat_range for a in AXES} if mode == 'seat' else dict(self.range)
        if gcmd.get('RANGE', None) is not None:
            limits = {a: gcmd.get_float('RANGE', above=0.) for a in AXES}
        for a in AXES:
            limits[a] = gcmd.get_float('RANGE_%s' % a.upper(), limits[a], above=0.)
        repeats = gcmd.get_int('REPEATS', self.seat_repeats if mode == 'seat' else self.repeats, minval=1)
        speed = gcmd.get_float('SPEED', self.speed_factor, above=0., maxval=1.)
        dry_run = gcmd.get_int('DRY_RUN', 0) == 1
        tools = []
        for number in numbers:
            tool = self.printer.lookup_object('tool T%d' % number, None)
            if tool is None:
                raise gcmd.error("DOCK_AUTOTUNE: no tool T%d" % number)
            chip = self.accel_chip.format(tool=number)
            if mode == 'seat' and self.printer.lookup_object(chip, None) is None:
                raise gcmd.error("DOCK_AUTOTUNE: MODE=seat needs the accelerometer [%s] (MODE=scan does not)" % chip)
            tools.append(tool)
        self._check_not_printing(gcmd)

        # PREPARE=1: bring the machine into the state of a print start first, PREPARE=0: it is
        # prepared already, without PREPARE the prompt asks (not for DRY_RUN)
        prepare = gcmd.get_int('PREPARE', self.prepare_default)
        if prepare is not None and not dry_run:
            self.gcode.respond_raw("// action:prompt_end")      # the buttons of the prompt call this with PREPARE
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
                self._prompt(gcmd, label, setup, plan, bed, touch)
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
            self._check_state(gcmd, numbers[0])

        per_axis = {a: int(limits[a] / step + 1e-9) for a in axes}
        bisect = max(0, int(math.ceil(math.log(step / resolution, 2) - 1e-9))) if self.refine else 0
        for number, tool in zip(numbers, tools):
            if mode == 'seat':
                positions = 1 + sum(2 * n for n in per_axis.values())
                gcmd.respond_info("dock_autotune T%d seat test, axes %s, step %.3f mm, range %s, %d repeat(s): up to "
                                  "%d lifts at %d positions, the tool stays in its dock; current dock position "
                                  "x=%.3f y=%.3f z=%.3f"
                                  % (number, axes.upper() or '-', step,
                                     ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                                     repeats, 1 + positions * repeats, positions,
                                     *[float(tool.params['params_park_%s' % a]) for a in 'xyz']))
            else:
                trials = sum(2 * n + 2 * bisect + 2 for n in per_axis.values())
                gcmd.respond_info("dock_autotune T%d scan, axes %s, step %.3f mm, range %s, %d repeat(s): about %d "
                                  "trials (%d pickups) at most, current dock position x=%.3f y=%.3f z=%.3f"
                                  % (number, axes.upper(), step,
                                     ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                                     repeats, trials, trials * repeats,
                                     *[float(tool.params['params_park_%s' % a]) for a in 'xyz']))
        if dry_run:
            if plan:
                gcmd.respond_info("DRY_RUN: PREPARE=1 would run first:\n  " + '\n  '.join(
                    p for p in plan if p != EMPTY_SHUTTLE))
            if mode == 'seat':
                try:
                    sp = self._seat_params(tools[0], speed)
                except AbortScan as e:
                    raise gcmd.error("DOCK_AUTOTUNE: %s" % e)
                enter, leave = self._seat_plan(sp)
                low, top = sp['geo']['low'], sp['geo']['top']
                gcmd.respond_info(
                    "DRY_RUN: the empty shuttle moves like this%s (G-code, like the pickup_gcode up to the verify "
                    "waypoint):\n  " % (" for T%d, the other tools with their own park positions" % numbers[0]
                                         if multi else "") + '\n  '.join(enter) +
                    "\nat every position (X or Y shifted at the lowest point), the accelerometer on:\n"
                    "  G0 Z%.4f F%g   lift to the verify waypoint (the first lift; the later ones go at this speed up to "
                    "%g mm below the first knock, then at %g mm/s)\n"
                    "  hold 0.3 s, read the tool detection, then G0 Z%.4f F%g   down again\n"
                    "and at the end:\n  " % (sp['z'] + top['z'], sp['path_speed'] * top['f'], self.seat_zone,
                                             self.seat_speed, sp['z'] + low['z'], sp['path_speed']) +
                    '\n  '.join(leave))
            if multi:
                gcmd.respond_info("DRY_RUN: the tools %s would be done one after the other (each one starts at safe "
                                  "Y); then the shuttle goes back to where the toolhead stood, and one result file "
                                  "holds the park positions of all tools" % label)
            gcmd.respond_info("DRY_RUN: nothing moved")
            return
        self.repeats, saved_repeats = repeats, self.repeats
        self._start(numbers[0])
        toolhead = self.printer.lookup_object('toolhead')
        saved_accel = toolhead.get_status(self._now())['max_accel']
        start_position = self._start_position() if multi else None
        runs = []
        try:
            if prepare:
                self._prepare(gcmd, plan)
            self.state['phase'] = 'start'
            self._run("SET_VELOCITY_LIMIT ACCEL=%g" % min(self.accel, saved_accel))
            for index, (number, tool) in enumerate(zip(numbers, tools)):
                self._check_abort()
                settings = ['step %.3f mm' % step, 'range ' + ' '.join('%s+/-%.2f' % (a, limits[a]) for a in axes),
                            '%d repeat(s)' % repeats, 'speed factor %g' % speed,
                            'acceleration %g' % min(self.accel, saved_accel)]
                if mode == 'seat':
                    settings.append('high-pass %d ms' % int(round(self.adxl_window * 1000)))
                    settings.append('speed at the contact %g mm/s' % self.seat_speed)
                settings.append('machine prepared by DOCK_AUTOTUNE with bed %g C' % bed if prepare
                                else 'machine was prepared before')
                run = self._run_tool(gcmd, tool, mode, axes, limits, step, repeats, resolution, speed, settings,
                                     check=prepare or index > 0, runs=runs, save=not multi)
                if run['status'] != 'finished':
                    break
            if multi and len(runs) == len(numbers) and all(r['status'] == 'finished' for r in runs):
                self.result = {'tools': {r['number']: r['result'] for r in runs}}
                self._return_to_start(gcmd, start_position, tools[-1])
        except AbortScan as e:
            self._abort_report(gcmd, self.state['tool'] if self.state['tool'] is not None else label, e)
        except self.gcode.error as e:
            self._abort_report(gcmd, self.state['tool'] if self.state['tool'] is not None else label, e)
            raise
        finally:
            if multi and runs:
                path = self._save_report('tools_%s_%s.txt' % (mode, time.strftime('%Y%m%d_%H%M%S')),
                                         lambda: build_multi_report(runs, numbers, mode,
                                                                    time.strftime('%Y-%m-%d %H:%M:%S')))
                if path:
                    gcmd.respond_info("dock_autotune: result file %s" % path)
            try:
                self._run("SET_VELOCITY_LIMIT ACCEL=%g" % saved_accel)
            except self.gcode.error:
                logging.exception("dock_autotune: could not restore the acceleration")
            self.repeats = saved_repeats
            self.active = False
            self.state['phase'] = 'idle'

    def _run_tool(self, gcmd, tool, mode, axes, limits, step, repeats, resolution, speed, settings, check,
                  runs, save):
        """The measurement of one tool (seat test or scan) with the park values and the path speed put back
        afterwards. Appends its run (number, status, info, result) to `runs`; an AbortScan ends this tool only."""
        number = tool.tool_number
        interrupted = self.abort        # an interrupt between two tools is not forgotten
        self._start(number)
        self.abort = interrupted
        saved_speed = float(tool.params['params_path_speed'])
        originals = {a: float(tool.params['params_park_%s' % a]) for a in AXES}
        proposals = {}
        status = 'stopped'
        run = {'number': number, 'status': status, 'info': None, 'result': {}}
        runs.append(run)
        try:
            if check:
                self._check_state(gcmd, number)
            tool.set_parameter('params_path_speed', saved_speed * speed)
            if self._tc_status()['tool_number'] == number:
                self._run("UNSELECT_TOOL %s" % RESTORE_NONE)
                self._wait()
            if mode == 'seat':
                self._seat_tool(gcmd, tool, axes, limits, step, repeats)
                self.result = {'tool': number, 'seat': self.seat_rows}
                gcmd.respond_info("dock_autotune T%d seat test finished after %d lifts, the park values are "
                                  "unchanged" % (number, self.cycles))
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
            raise
        finally:
            info = self._report_info(number, mode, status, originals, proposals, settings)
            run.update(status=status, info=info, result=self.result)
            if save and info:
                path = self._save_report('T%d_%s_%s.txt' % (number, mode, time.strftime('%Y%m%d_%H%M%S')),
                                         lambda: build_report(info))
                if path:
                    gcmd.respond_info("dock_autotune: result file %s" % path)
            # back to the old values except for the axes that were centred
            for a in AXES:
                if a in proposals:
                    tool.set_parameter('params_park_%s' % a, round(proposals[a], 3))
                else:
                    tool.set_parameter('params_park_%s' % a, originals[a])
            tool.set_parameter('params_path_speed', saved_speed)
        return run

    def _report_info(self, number, mode, status, originals, proposals, settings):
        """What the result file of one tool is made of, None if nothing was measured"""
        seat = self.seat_info if (self.seat_rows or self.seat_info.get('coarse')) else None
        if not (self.trial_rows or seat):
            return None
        return {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'tool': number, 'mode': mode, 'status': status,
                'pickups': self.cycles, 'settings': settings, 'originals': originals, 'new': proposals,
                'scan': self.scan_result, 'trials': self.trial_rows, 'seat': seat,
                'window': self.adxl_window}

    def _save_report(self, name, render):
        """Writes the text that render() makes to results_dir, returns the path (None: no folder or not writable)"""
        if not self.results_dir:
            return None
        path = os.path.join(self.results_dir, name)
        try:
            text = render()
            os.makedirs(self.results_dir, exist_ok=True)
            with open(path, 'w') as f:
                f.write(text)
        except (OSError, KeyError, TypeError, ValueError):
            logging.exception("dock_autotune: cannot write the result file %s", path)
            return None
        return path

    def _start_position(self):
        """Where the toolhead stands, None if the machine is not homed"""
        status = self.printer.lookup_object('toolhead').get_status(self._now())
        if all(a in status['homed_axes'] for a in 'xyz'):
            return list(status['position'][:3])
        return None

    def _return_to_start(self, gcmd, position, tool):
        """Without a tool on the shuttle back to where the toolhead stood before the run: sideways at the height of
        the docks first, then down (not nearer to the docks than close Y)"""
        if position is None:
            gcmd.respond_info("dock_autotune: the machine was not homed before the run, the shuttle stays at safe Y")
            return
        close_y = float(tool.params.get('params_close_y', 0.0))
        fast = float(tool.params.get('params_fast_speed', 6000.0))
        self._run('G90')
        self._run('G0 X%.3f Y%.3f F%g' % (position[0], max(position[1], close_y), fast))
        self._run('G0 Z%.3f F%g' % (position[2], 3000.0))
        self.printer.lookup_object('toolhead').wait_moves()
        gcmd.respond_info("dock_autotune: all tools done, the shuttle is back at X=%.1f Y=%.1f Z=%.1f"
                          % (position[0], max(position[1], close_y), position[2]))

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
        self.trial_rows, self.scan_result = [], {}
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
        gcmd.respond_info("dock_autotune %s STOPPED: %s. Toolchanger: %s, tool %s. The park values are back at "
                          "their old values. Check the shuttle and the dock before you do anything else."
                          % (number if isinstance(number, str) else 'T%d' % number, error, status['status'],
                             status['tool_number']))
        self._log({'tool': number, 'aborted': str(error)})

    def cmd_INTERRUPT(self, gcmd):
        if self.active:
            self.abort = True
            gcmd.respond_info("dock_autotune: stopping after the current trial")
        else:
            gcmd.respond_info("dock_autotune: nothing is running")


def load_config(config):
    return DockAutotune(config)
