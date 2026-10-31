import math
import os
import random
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'klipper', 'extras'))

import dock_autotune  # noqa: E402
from dock_autotune import (event_height, find_events, height_at, move_duration, parse_lift,  # noqa: E402
                           seat_crossings, seat_enter_lines, seat_exit_lines, seat_proposal, seat_row)
from test_dock_autotune import (CommandError, FakeConfig, FakeDock, FakeGcmd, FakePrinter, FakePrintStats,  # noqa: E402
                                FakeTool, FakeGcode, FakeToolhead)

G = dock_autotune.FREEFALL
RATE = 3200.0
PATH = [{'z': -10, 'y': 16}, {'z': -10, 'y': 0}, {'z': 0, 'y': 0, 'f': 0.7, 'verify': 1}, {'y': 0, 'z': 5},
        {'y': 9.5, 'z': 5}, {'y': 9.5, 'z': 5}]


# --- pure helpers -------------------------------------------------------------------------

class ParseLiftTests(unittest.TestCase):
    def test_the_path_of_a_stealthchanger(self):
        geo = parse_lift(PATH)
        self.assertEqual(len(geo['approach']), 2)
        self.assertEqual((geo['low']['y'], geo['low']['z']), (0.0, -10.0))
        self.assertEqual((geo['top']['y'], geo['top']['z'], geo['top']['f']), (0.0, 0.0, 0.7))
        self.assertEqual(geo['approach'][0]['y'], 16.0)

    def test_missing_keys_keep_the_value_before(self):
        geo = parse_lift([{'x': 1, 'y': 8, 'z': -5}, {'y': 0}, {'z': 0, 'verify': 1}])
        self.assertEqual((geo['low']['x'], geo['low']['y'], geo['low']['z']), (1.0, 0.0, -5.0))
        self.assertEqual((geo['top']['x'], geo['top']['y'], geo['top']['z']), (1.0, 0.0, 0.0))

    def test_paths_without_a_vertical_lift(self):
        for path in ([], [{'z': 0, 'y': 0, 'verify': 1}], [{'z': -5, 'y': 5}, {'z': 0, 'y': 0, 'verify': 1}],
                     [{'z': -5, 'y': 0}, {'z': -5, 'y': 0, 'verify': 1}], [{'z': 5, 'y': 0}, {'z': 0, 'y': 0, 'verify': 1}],
                     [{'z': 0, 'y': 0}]):
            with self.assertRaises(ValueError):
                parse_lift(path)

    def test_lines_like_the_pickup_gcode(self):
        geo = parse_lift(PATH)
        enter = seat_enter_lines(geo, 2.6, -30.1, 266.2, 22.6, 13200, 750)
        self.assertEqual(enter, ['G90', 'G0 Y22.600 F13200', 'G0 X2.6000 Z256.2000 F13200', 'G0 Y-14.1000 F13200',
                                 'G0 X2.6000 Y-30.1000 Z256.2000 F750'])
        leave = seat_exit_lines(geo, 2.6, -30.1, 266.2, 100, 13200, 750)
        self.assertEqual(leave, ['G0 X2.6000 Y-14.1000 Z256.2000 F750', 'G0 Y100.000 F13200'])


class MotionTests(unittest.TestCase):
    def test_move_duration(self):
        self.assertAlmostEqual(move_duration(10, 10, 500), 1.0 + 0.02)
        self.assertAlmostEqual(move_duration(0.1, 10, 500), 2 * math.sqrt(0.1 / 500))
        self.assertEqual(move_duration(0, 10, 500), 0.0)

    def test_height_at_follows_the_cruise_line(self):
        seg = {'z_start': 100.0, 'z_end': 110.0, 'v': 5.0, 'a': 500.0, 't_end': 20.0, 'dur': move_duration(10, 5, 500)}
        self.assertAlmostEqual(height_at(20.0 - seg['dur'] / 2, seg), 105.0, places=2)
        self.assertEqual(height_at(5.0, seg), 100.0)            # before the move
        self.assertEqual(height_at(25.0, seg), 110.0)

    def test_event_height_between_moves_and_after_them(self):
        a = {'z_start': 0.0, 'z_end': 4.0, 'v': 8.0, 'a': 500.0, 't_end': 1.0, 'dur': move_duration(4, 8, 500)}
        b = {'z_start': 4.0, 'z_end': 6.0, 'v': 2.0, 'a': 500.0, 't_end': 3.0, 'dur': move_duration(2, 2, 500)}
        self.assertEqual(event_height(1.4, [a, b]), 4.0)
        self.assertEqual(event_height(3.5, [a, b]), 6.0)
        self.assertAlmostEqual(event_height(2.0, [a, b]), 4.0 + 2.0 * (2.0 - (3.0 - b['dur'])) - 0.0, places=1)


def burst(t0, amp, rate=RATE, length=0.03):
    """samples of a damped 800 Hz knock starting at t0: [(t, ax, ay, az) in mm/s^2], amp = (x, y, z) in g"""
    out = []
    for i in range(int(length * rate)):
        t = i / rate
        w = math.sin(2 * math.pi * 800 * t) * math.exp(-t / 0.003)
        out.append((t0 + t, amp[0] * w * G, amp[1] * w * G, (amp[2] * w - 1.0) * G))
    return out


def quiet(t_from, t_to, noise=0.03, seed=3):
    rnd = random.Random(seed)
    n = int((t_to - t_from) * RATE)
    return [(t_from + i / RATE, rnd.gauss(0, noise) * G, rnd.gauss(0, noise) * G, (-1.0 + rnd.gauss(0, noise)) * G)
            for i in range(n)]


class FindEventsTests(unittest.TestCase):
    def series(self, knocks, length=2.0):
        samples = quiet(0.0, length)
        for t0, amp in knocks:
            for k, (t, x, y, z) in enumerate(burst(t0, amp)):
                i = int(round((t - 0.0) * RATE))
                if i < len(samples):
                    s = samples[i]
                    samples[i] = (s[0], s[1] + x, s[2] + y, s[3] + z + G)
        return samples

    def test_two_knocks(self):
        samples = self.series([(0.8, (0.5, 0.0, 6.0)), (1.3, (-0.4, 0.3, 4.0))])
        events, noise, limit = find_events(samples, 0.1, 2.0, 0.5)
        self.assertEqual(len(events), 2)
        self.assertAlmostEqual(events[0]['t'], 0.8, delta=0.003)
        self.assertAlmostEqual(events[1]['t'], 1.3, delta=0.003)
        self.assertGreater(events[0]['peak_g'], events[1]['peak_g'])
        self.assertLess(noise, 0.2)
        self.assertEqual(limit, 0.5)

    def test_lateral_impulse_has_the_sign_of_the_push(self):
        events = find_events(self.series([(0.8, (1.0, -1.5, 6.0))]), 0.1, 2.0, 0.5)[0]
        self.assertGreater(events[0]['i_x'], 0)
        self.assertLess(events[0]['i_y'], 0)
        self.assertGreater(events[0]['e_z'], events[0]['e_x'])

    def test_nothing_happens(self):
        self.assertEqual(find_events(self.series([]), 0.1, 2.0, 0.5)[0], [])

    def test_knocks_before_the_quiet_time_are_ignored(self):
        events = find_events(self.series([(0.3, (0, 0, 6.0)), (1.2, (0, 0, 6.0))]), 0.1, 2.0, 0.6)[0]
        self.assertEqual(len(events), 1)

    def test_threshold_can_be_given(self):
        samples = self.series([(0.8, (0, 0, 0.4))])
        self.assertEqual(find_events(samples, 0.1, 2.0, 0.5)[0], [])
        self.assertEqual(len(find_events(samples, 0.1, 2.0, 0.5, threshold=0.2)[0]), 1)

    def test_too_few_samples(self):
        self.assertEqual(find_events(quiet(0, 0.005), 0, 1, 0.5)[0], [])


class SeatRowTests(unittest.TestCase):
    def hub(self, z, push=1.0, pin=True):
        return {'pin': pin, 'release_g': 1.0, 'events': [
            {'z': z - 1.0, 'rel': -1.0, 'peak_g': 5.0, 'e_x': 0.1, 'e_y': 0.1, 'e_z': 0.8, 'i_x': push, 'i_y': -push},
            {'z': z, 'rel': 0.0, 'peak_g': 3.0, 'e_x': 0.1, 'e_y': 0.1, 'e_z': 0.8, 'i_x': push, 'i_y': 0.0}]}

    def test_medians(self):
        row = seat_row(0.25, 150.0, [self.hub(10.0), self.hub(10.02), self.hub(10.04, pin=False)])
        self.assertEqual((row['pin'], row['total'], row['ok']), (2, 3, False))
        self.assertEqual(row['z_first'], 9.02)
        self.assertEqual(row['z_last'], 10.02)
        self.assertEqual(row['events'], 2.0)
        self.assertAlmostEqual(row['lateral'], 0.2, places=3)
        self.assertEqual(row['ix'], 2.0)

    def test_no_events(self):
        row = seat_row(0.0, 1.0, [{'pin': False, 'events': [], 'release_g': None}])
        self.assertIsNone(row['z_first'])
        self.assertIsNone(row['lateral'])

    def test_crossings(self):
        rows = [{'offset': -0.25, 'ix': -2.0}, {'offset': 0.0, 'ix': -1.0}, {'offset': 0.25, 'ix': 1.0},
                {'offset': 0.5, 'ix': None}]
        self.assertEqual(seat_crossings(rows, 'ix'), [0.125])


# --- the command with a simulated shuttle --------------------------------------------------

TRUTH = {'x': 149.8, 'y': -29.55}                  # where the dock really is (park x 149.6, y -29.4)
CONTACTS = (264.9, 265.6)                           # the pins touch, the magnets snap (park z 266.2)
PIN_LIMIT = 1.15                                    # the tool detection is on while the error is smaller


class SeatGcode(FakeGcode):
    def run_script_from_command(self, script):
        word = script.split()[0]
        if word in ('G0', 'G1'):
            self.dock.rig.move(script)
            self.dock.scripts.append(script)
        elif word == 'G90':
            self.dock.scripts.append(script)
        else:
            super().run_script_from_command(script)


class SeatToolhead(FakeToolhead):
    def __init__(self, dock):
        super().__init__(dock)

    def get_status(self, eventtime):
        s = super().get_status(eventtime)
        s['position'] = tuple(self.dock.rig.pos) + (0.0,)
        return s

    def wait_moves(self):
        self.waits += 1

    def dwell(self, delay):
        self.clock += delay


class Rig:
    """The shuttle: trapezoid moves, and a knock when it rises through the contact heights of the dock"""

    def __init__(self, dock, toolhead, contacts=CONTACTS, truth=TRUTH, gain=2.0, knocks=(6.0, 4.0), accel=500.0,
                 quiet_at_truth=False, wobble=0.0):
        self.dock, self.toolhead, self.contacts, self.truth, self.gain, self.knocks = dock, toolhead, contacts, truth, gain, knocks
        self.accel = accel
        self.quiet_at_truth = quiet_at_truth        # the whole knock gets weaker the nearer the shuttle is
        self.wobble = wobble                        # g per mm of error: the tool swings sideways (6 Hz) during a lift
        self.pos = [0.0, 100.0, 300.0]
        self.moves = []
        self.mounted = False

    def lateral(self, pos):
        return pos[0] - self.truth['x'], pos[1] - self.truth['y']

    def move(self, script):
        target, speed = list(self.pos), None
        for word in script.split()[1:]:
            if word[0] == 'X':
                target[0] = float(word[1:])
            elif word[0] == 'Y':
                target[1] = float(word[1:])
            elif word[0] == 'Z':
                target[2] = float(word[1:])
            elif word[0] == 'F':
                speed = float(word[1:]) / 60.0
        dist = math.dist(self.pos, target)
        t0 = self.toolhead.clock
        if dist > 1e-9:
            peak = speed if dist >= speed * speed / self.accel else math.sqrt(self.accel * dist)
            duration = dock_autotune.move_duration(dist, speed, self.accel)
            self.moves.append({'t0': t0, 't1': t0 + duration, 'p0': list(self.pos), 'p1': target, 'peak': peak,
                               'dist': dist})
            self.toolhead.clock = t0 + duration
        self.pos = target
        # the tool detection: on while the shuttle is up in the magnets and the error is small
        ex, ey = self.lateral(self.pos)
        high = self.pos[2] >= self.contacts[1] - 1e-9
        self.dock.detected = self.dock.tool.tool_number if (high and abs(ex) <= PIN_LIMIT and abs(ey) <= PIN_LIMIT) else -1

    def travelled(self, move, t):
        """distance along the move at time t"""
        a, peak, dist = self.accel, move['peak'], move['dist']
        ta = peak / a
        total = move['t1'] - move['t0']
        t -= move['t0']
        if t <= 0:
            return 0.0
        if t >= total:
            return dist
        if t < ta:
            return 0.5 * a * t * t
        if t > total - ta:
            return dist - 0.5 * a * (total - t) ** 2
        return 0.5 * a * ta * ta + peak * (t - ta)

    def time_at_height(self, move, z):
        if not (move['p0'][2] < z < move['p1'][2]) or math.dist(move['p0'][:2], move['p1'][:2]) > 1e-9:
            return None
        lo, hi = move['t0'], move['t1']
        want = (z - move['p0'][2])
        for _ in range(60):
            mid = (lo + hi) / 2
            if self.travelled(move, mid) < want:
                lo = mid
            else:
                hi = mid
        return hi

    def samples(self, start, end):
        rnd = random.Random(5)
        n = int((end - start) * RATE)
        out = [[start + i / RATE, rnd.gauss(0, 0.03), rnd.gauss(0, 0.03), -1.0 + rnd.gauss(0, 0.03)] for i in range(n)]
        for move in self.moves:
            if self.wobble and move['p1'][2] > move['p0'][2] and math.dist(move['p0'][:2], move['p1'][:2]) < 1e-9:
                ex, ey = self.lateral(move['p0'])
                for i in range(n):
                    t = start + i / RATE
                    if move['t0'] <= t <= move['t1']:
                        out[i][1] += self.wobble * math.hypot(ex, ey) * math.sin(2 * math.pi * 6 * (t - move['t0']))
            for k, z in enumerate(self.contacts):
                t = self.time_at_height(move, z)
                if t is None or not start <= t <= end:
                    continue
                ex, ey = self.lateral(move['p0'])
                scale = 0.1 + math.hypot(ex, ey) if self.quiet_at_truth else 1.0
                amp = (self.gain * ex, self.gain * ey, self.knocks[k] * scale)
                for j in range(int(0.03 * RATE)):
                    i = int(round((t - start) * RATE)) + j
                    if 0 <= i < n:
                        w = math.sin(2 * math.pi * 800 * j / RATE) * math.exp(-j / RATE / 0.003)
                        for axis in range(3):
                            out[i][axis + 1] += amp[axis] * w
        return [(s[0], s[1] * G, s[2] * G, s[3] * G) for s in out]


class SeatChip:
    def __init__(self, rig, silent=False):
        self.rig, self.silent = rig, silent

    def start_internal_client(self):
        chip, start = self, self.rig.toolhead.clock

        class Client:
            def finish_measurements(self):
                pass

            def get_samples(self):
                end = chip.rig.toolhead.clock
                if chip.silent:
                    chip_rig = chip.rig
                    saved = chip_rig.contacts
                    chip_rig.contacts = (-1.0, -2.0)
                    try:
                        return chip_rig.samples(start, end)
                    finally:
                        chip_rig.contacts = saved
                return chip.rig.samples(start, end)
        return Client()


def energy_rows(energies, step=0.25, first=None):
    first = -step * (len(energies) // 2) if first is None else first
    return [{'offset': round(first + i * step, 4), 'value': round(10.0 + first + i * step, 4), 'score': e}
            for i, e in enumerate(energies)]


class SeatProposalTests(unittest.TestCase):
    def test_the_centre_of_the_quiet_window(self):
        rows = energy_rows([1.0, 0.9, 0.1, 0.05, 0.1, 0.8, 1.0])           # -0.75 .. +0.75
        p = seat_proposal('x', rows)
        self.assertEqual(p['state'], 'ok')
        self.assertAlmostEqual(p['offset'], 0.0, places=3)
        self.assertAlmostEqual(p['value'], 10.0, places=3)
        self.assertEqual((p['minus'], p['plus']), (-0.375, 0.375))
        self.assertIn('quiet from -0.38 to +0.38 mm', p['text'])

    def test_the_window_is_not_in_the_middle(self):
        p = seat_proposal('y', energy_rows([1.0, 0.1, 0.05, 0.1, 0.9, 1.0, 1.0]))
        self.assertEqual(p['state'], 'ok')
        self.assertAlmostEqual(p['offset'], -0.25 + 0.0, delta=0.13)

    def test_open_window_asks_for_a_larger_range(self):
        p = seat_proposal('y', energy_rows([0.05, 0.05, 0.06, 0.07, 0.9, 1.0, 1.0]))      # quiet to the minus end
        self.assertEqual(p['state'], 'open')
        self.assertIsNone(p['value'])
        self.assertIn('minus side', p['text'])
        self.assertIn('raise seat_range', p['text'])

    def test_the_widest_closed_run_wins(self):
        p = seat_proposal('x', energy_rows([1.0, 0.05, 1.0, 0.9, 0.05, 0.06, 0.05, 0.1, 1.0, 1.0], first=-1.0))
        self.assertEqual(p['state'], 'ok')
        self.assertGreater(p['offset'], 0.0)

    def test_nothing_to_see(self):
        self.assertEqual(seat_proposal('x', energy_rows([0.10, 0.12, 0.09, 0.11, 0.10]))['state'], 'flat')
        self.assertEqual(seat_proposal('x', energy_rows([0.1, 0.9]))['state'], 'no data')
        self.assertEqual(seat_proposal('x', [{'offset': 0.0, 'value': 1.0}] * 5)['state'], 'no data')


class SeatCommandTests(unittest.TestCase):
    def make(self, silent=False, contacts=CONTACTS, **options):
        self.tool = FakeTool(2, 149.6, -29.4, 266.2)
        self.tool.params.update({'params_close_y': 22.6, 'params_fast_speed': 13200.0, 'params_path_speed': 1500.0,
                                 'params_pickup_path': PATH})
        self.dock = FakeDock(self.tool, {'x': 0, 'y': 0, 'z': 0}, {'x': (-1, 1), 'y': (-1, 1), 'z': (-1, 1)})
        self.dock.accel = 500.0
        self.printer = FakePrinter(self.dock, self.tool)
        gcode = self.printer.objects['gcode'] = SeatGcode(self.dock)
        toolhead = self.printer.objects['toolhead'] = SeatToolhead(self.dock)
        self.rig = self.dock.rig = Rig(self.dock, toolhead, contacts=contacts)
        self.printer.objects['adxl345 T2'] = SeatChip(self.rig, silent)
        self.printer.objects['print_stats'] = FakePrintStats()
        options.setdefault('log_file', os.path.join(tempfile.mkdtemp(), 'dock_autotune.jsonl'))
        options.setdefault('results_dir', os.path.join(tempfile.mkdtemp(), 'results'))
        self.module = dock_autotune.DockAutotune(FakeConfig(self.printer, **options))
        self.gcode = gcode
        return self.module

    def seat(self, **params):
        params.setdefault('PREPARE', 0)
        params.setdefault('MODE', 'seat')
        gcmd = FakeGcmd(**params)
        self.module.cmd_DOCK_AUTOTUNE(gcmd)
        return gcmd

    def report(self):
        files = sorted(os.listdir(self.module.results_dir))
        self.assertEqual(len(files), 1)
        with open(os.path.join(self.module.results_dir, files[0])) as f:
            return files[0], f.read()

    def test_the_contact_height_comes_out_in_millimetres(self):
        self.make()
        gcmd = self.seat(T=2, AXES='x', RANGE=0.5, STEP=0.25, REPEATS=2)
        self.assertNotIn('STOPPED', gcmd.text)
        seat = self.module.seat_info
        rows = seat['rows']['x']
        self.assertEqual(len(rows), 5)
        for row in rows:
            self.assertAlmostEqual(row['z_first'], CONTACTS[0], delta=0.03)
            self.assertAlmostEqual(row['z_last'], CONTACTS[1], delta=0.03)
            self.assertEqual(row['events'], 2.0)
        self.assertAlmostEqual(seat['coarse']['events'][0]['z'], CONTACTS[0], delta=0.05)

    def test_the_tool_stays_in_the_dock_and_nothing_is_changed(self):
        self.make()
        self.seat(T=2, AXES='x', RANGE=0.25, STEP=0.25, REPEATS=1)
        self.assertFalse([s for s in self.dock.scripts if s.startswith(('SELECT_TOOL', 'UNSELECT_TOOL'))])
        self.assertEqual((self.tool.params['params_park_x'], self.tool.params['params_park_y'],
                          self.tool.params['params_park_z']), (149.6, -29.4, 266.2))
        self.assertEqual(self.tool.params['params_path_speed'], 1500.0)
        self.assertEqual(self.dock.mounted, -1)

    def test_same_moves_as_the_pickup_and_back_out_at_the_end(self):
        self.make()
        self.seat(T=2, AXES='', REPEATS=1)
        lines = [s for s in self.dock.scripts if s.startswith('G')]
        self.assertEqual(lines[:4], ['G90', 'G0 Y22.600 F13200', 'G0 X149.6000 Z256.2000 F13200',
                                     'G0 Y-13.4000 F13200'])
        self.assertEqual(lines[-1], 'G0 Y100.000 F13200')
        self.assertEqual(self.rig.pos[1], 100.0)
        self.assertGreater(max(m['p1'][2] for m in self.rig.moves), 265.9)       # up to the verify waypoint, not higher

    def test_never_higher_than_the_verify_waypoint(self):
        self.make()
        self.seat(T=2, AXES='x', RANGE=0.25, STEP=0.25, REPEATS=2)
        lifts = [m['p1'][2] for m in self.rig.moves if m['p1'][1] < 50 and abs(m['p1'][2] - m['p0'][2]) > 0.01]
        self.assertEqual(max(lifts), 266.2)

    def test_the_slow_part_starts_below_the_first_knock(self):
        self.make()
        gcmd = self.seat(T=2, AXES='x', RANGE=0.25, STEP=0.25, REPEATS=1)
        self.assertAlmostEqual(self.module.seat_info['slow_from_z'], CONTACTS[0] - 1.5, delta=0.06)
        slow = [m for m in self.rig.moves if abs(m['peak'] - 2.0) < 1e-6]
        self.assertEqual(len(slow), 3)                               # centre, +0.25, -0.25
        self.assertTrue(all(abs(m['p0'][2] - self.module.seat_info['slow_from_z']) < 1e-3 for m in slow))
        self.assertIn('the slow part starts at Z', gcmd.text)

    def test_lateral_push_changes_sign_at_the_true_position(self):
        self.make()
        self.seat(T=2, AXES='xy', RANGE=0.5, STEP=0.25, REPEATS=2)
        rows = self.module.seat_info['rows']
        self.assertEqual([r['offset'] for r in sorted(rows['x'], key=lambda r: r['offset'])], [-0.5, -0.25, 0.0, 0.25, 0.5])
        # the dock is 0.2 mm in +x and 0.15 mm in -y from the park position
        self.assertAlmostEqual(seat_crossings(rows['x'], 'ix')[0], 0.2, delta=0.06)
        self.assertAlmostEqual(seat_crossings(rows['y'], 'iy')[0], -0.15, delta=0.06)
        best = min(rows['x'], key=lambda r: r['lateral'])
        self.assertEqual(best['offset'], 0.25)

    def test_positions_where_the_tool_is_not_detected_end_a_side(self):
        # the tool detection is off from an error of 1.15 mm on, and the dock is 0.2 mm off: +1.5 and +2.0 fail, so
        # +2.5 and +3.0 are skipped; on the other side -1.0 and -1.5 fail, so -2.0 and further are skipped
        self.make(seat_fail_stop='2')
        self.seat(T=2, AXES='x', RANGE=3.0, STEP=0.5, REPEATS=1)
        rows = self.module.seat_info['rows']['x']
        offsets = sorted(r['offset'] for r in rows)
        self.assertEqual(offsets, [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0])
        self.assertEqual(sorted(r['offset'] for r in rows if not r['ok']), [-1.5, -1.0, 1.5, 2.0])

    def test_the_quiet_window_gives_a_suggestion_and_applies_nothing(self):
        self.make()
        self.rig.quiet_at_truth = True
        gcmd = self.seat(T=2, AXES='x', RANGE=1.0, STEP=0.25, REPEATS=1)
        self.assertNotIn('STOPPED', gcmd.text)
        p = self.module.seat_info['proposals']['x']
        self.assertEqual(p['state'], 'ok')
        self.assertAlmostEqual(p['offset'], 0.2, delta=0.12)        # the dock is 0.2 mm off in x
        self.assertAlmostEqual(p['value'], 149.6 + p['offset'], places=3)
        self.assertIn('Quiet window X: quiet from', gcmd.text)
        self.assertIn('SET_TOOL_PARAMETER T=2 PARAMETER=params_park_x VALUE=%.3f' % p['value'], gcmd.text)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)
        text = self.report()[1]
        self.assertIn('Suggested new park positions (nothing is applied', text)
        self.assertIn('  params_park_x: %.3f' % p['value'], text)
        self.assertIn('%10.3f   quiet from' % p['value'], text)
        self.assertNotIn('changed', text)

    def test_the_wobble_of_the_tool_is_measured_and_counts(self):
        # no knock gets weaker, but the tool swings sideways in proportion to the error: the score sees it
        self.make()
        self.rig.wobble = 1.0
        self.seat(T=2, AXES='x', RANGE=0.5, STEP=0.25, REPEATS=1)
        rows = {r['offset']: r for r in self.module.seat_info['rows']['x']}
        self.assertGreater(rows[-0.5]['wobble'], 5 * rows[0.25]['wobble'])        # the dock is 0.2 mm in +x
        self.assertGreater(rows[-0.5]['score'], rows[-0.5]['energy'])
        self.assertIn('wobble', self.report()[1])

    def test_a_window_outside_of_the_range_is_not_suggested(self):
        self.make()
        self.rig.quiet_at_truth = True
        self.rig.truth = {'x': 151.6, 'y': TRUTH['y']}               # 2 mm away: the range ends before the window
        self.rig.contacts = CONTACTS
        gcmd = self.seat(T=2, AXES='x', RANGE=1.0, STEP=0.25, REPEATS=1)
        p = self.module.seat_info['proposals']['x']
        self.assertIn(p['state'], ('open', 'flat'))
        self.assertIsNone(p['value'])
        self.assertNotIn('SET_TOOL_PARAMETER', gcmd.text)

    def test_z_only(self):
        self.make()
        gcmd = self.seat(T=2, AXES='z', REPEATS=2)
        self.assertNotIn('STOPPED', gcmd.text)
        self.assertEqual(len(self.module.seat_info['rows']['z']), 1)
        self.assertAlmostEqual(self.module.seat_info['rows']['z'][0]['z_last'], CONTACTS[1], delta=0.03)

    def test_stops_if_nothing_knocks(self):
        self.make(silent=True)
        gcmd = self.seat(T=2, AXES='x', RANGE=0.25, STEP=0.25)
        self.assertIn('STOPPED', gcmd.text)
        self.assertIn('no knock in the lift', gcmd.text)
        self.assertEqual(self.rig.pos[1], 100.0)            # the shuttle left the dock anyway

    def test_the_shuttle_must_be_outside_and_empty(self):
        self.make()
        self.rig.pos = [149.6, -20.0, 266.2]
        self.assertIn('inside of the dock', self.seat(T=2, AXES='x', RANGE=0.25).text)
        self.make()
        self.dock.detected = 1
        self.assertIn('a tool is detected on the shuttle', self.seat(T=2, AXES='x', RANGE=0.25).text)
        self.assertEqual([s for s in self.dock.scripts if s.startswith('G0')], [])

    def test_the_shuttle_may_be_anywhere_outside_of_the_dock(self):
        # after a preparation without touch homing it stands where the levelling left it
        self.make()
        self.rig.pos = [150.0, 10.0, 200.0]
        gcmd = self.seat(T=2, AXES='', REPEATS=1)
        self.assertNotIn('STOPPED', gcmd.text)
        self.assertEqual(self.rig.pos[1], 100.0)

    def test_interrupt_leaves_the_dock(self):
        self.make()
        real = self.module._seat_hub
        calls = []

        def hub(*a, **kw):
            calls.append(1)
            if len(calls) == 3:
                self.module.abort = True
            return real(*a, **kw)
        self.module._seat_hub = hub
        gcmd = self.seat(T=2, AXES='x', RANGE=1.0, STEP=0.25, REPEATS=1)
        self.assertIn('STOPPED', gcmd.text)
        self.assertEqual(self.rig.pos[1], 100.0)
        self.assertIn('Status: STOPPED: stopped by DOCK_AUTOTUNE_INTERRUPT', self.report()[1])

    def test_dry_run_shows_the_moves(self):
        self.make()
        gcmd = self.seat(T=2, AXES='x', DRY_RUN=1)
        self.assertIn('DRY_RUN: the empty shuttle moves like this', gcmd.text)
        self.assertIn('G0 Y22.600 F13200', gcmd.text)
        self.assertIn('G0 Y100.000 F13200', gcmd.text)
        self.assertIn('nothing moved', gcmd.text)
        self.assertEqual(self.dock.scripts, [])

    def test_a_path_without_a_lift_is_reported(self):
        self.make()
        self.tool.params['params_pickup_path'] = [{'z': 0, 'y': 0}]
        with self.assertRaises(CommandError):
            self.seat(T=2, AXES='x', DRY_RUN=1)
        gcmd = self.seat(T=2, AXES='x')
        self.assertIn('STOPPED', gcmd.text)
        self.assertIn('no waypoint with verify', gcmd.text)

    def test_needs_the_accelerometer(self):
        self.make()
        del self.printer.objects['adxl345 T2']
        with self.assertRaises(CommandError) as ctx:
            self.seat(T=2)
        self.assertIn('MODE=seat needs the accelerometer', str(ctx.exception))

    def test_result_file_with_heights_and_table(self):
        self.make()
        gcmd = self.seat(T=2, AXES='x', RANGE=0.25, STEP=0.25, REPEATS=1)
        name, text = self.report()
        self.assertTrue(name.startswith('T2_seat_'))
        self.assertIn('result file', gcmd.text)
        self.assertIn('seat test', text.splitlines()[0])
        self.assertIn('Lift into the shuttle: park Z 266.200, verify waypoint at Z 266.200', text)
        self.assertIn('the strongest knock is at Z 264.9', text)
        self.assertIn('Seat test T2 X', text)
        self.assertIn('the contact height is given below', text)
        self.assertIn('No new park position can be suggested.', text)
        self.assertNotIn('changed', text)
        csv = text.split('Data (CSV)')[1].strip().splitlines()
        self.assertTrue(csv[0].startswith('tool,axis,offset_mm,value_mm,pin_ok'))
        self.assertEqual(len(csv), 1 + 3)

    def test_config_defaults(self):
        self.make(mode='seat', axes='xyz', seat_range='0.5', seat_step='0.25', seat_repeats='1', seat_speed='1')
        gcmd = self.seat(T=2, MODE='seat')
        self.assertIn('seat test, axes XY', gcmd.text)
        self.assertEqual(sorted({r['offset'] for r in self.module.seat_info['rows']['x']}), [-0.5, -0.25, 0.0, 0.25, 0.5])
        slow = [m for m in self.rig.moves if abs(m['peak'] - 1.0) < 1e-6]
        self.assertTrue(slow)

    def test_unknown_mode_is_still_refused(self):
        self.make()
        with self.assertRaises(CommandError):
            self.seat(T=2, MODE='both')


if __name__ == '__main__':
    unittest.main()
