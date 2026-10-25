import os
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'klipper', 'extras'))

import dock_autotune  # noqa: E402
from dock_autotune import scan_axis, shock_metrics, window_center  # noqa: E402


# --- pure helpers ----------------------------------------------------------

def window_trial(low, high):
    calls = []

    def trial(offset):
        calls.append(offset)
        return low - 1e-9 <= offset <= high + 1e-9
    trial.calls = calls
    return trial


class ScanAxisTests(unittest.TestCase):
    def test_finds_and_refines_the_edges(self):
        edges = scan_axis(window_trial(-0.35, 0.25), 0.1, 0.8)
        self.assertEqual((edges['minus'], edges['plus']), (-0.35, 0.25))
        self.assertFalse(edges['minus_open'] or edges['plus_open'])
        self.assertAlmostEqual(window_center(edges), -0.05)

    def test_without_refine_the_edge_is_the_last_step(self):
        edges = scan_axis(window_trial(-0.35, 0.25), 0.1, 0.8, refine=False)
        self.assertEqual((edges['minus'], edges['plus']), (-0.3, 0.2))

    def test_open_side_gives_no_centre(self):
        edges = scan_axis(window_trial(-9, 0.25), 0.1, 0.5)
        self.assertTrue(edges['minus_open'])
        self.assertEqual(edges['minus'], -0.5)
        self.assertIsNone(window_center(edges))

    def test_stops_at_the_first_failure_and_goes_outwards_only(self):
        trial = window_trial(-0.2, 0.2)
        scan_axis(trial, 0.1, 0.8, refine=False)
        self.assertEqual(trial.calls, [0.1, 0.2, 0.3, -0.1, -0.2, -0.3])

    def test_never_tries_beyond_the_limit(self):
        trial = window_trial(-9, 9)
        scan_axis(trial, 0.1, 0.35)
        self.assertEqual(max(abs(x) for x in trial.calls), 0.3)

    def test_zero_width_window(self):
        edges = scan_axis(window_trial(0, 0), 0.1, 0.5)
        self.assertEqual((edges['minus'], edges['plus']), (0.0, 0.0))
        self.assertEqual(window_center(edges), 0.0)


class ShockMetricsTests(unittest.TestCase):
    G = dock_autotune.FREEFALL

    def samples(self, values):
        return [(i * 0.001, 0.0, 0.0, v * self.G) for i, v in enumerate(values)]

    def test_too_few_samples(self):
        self.assertIsNone(shock_metrics([]))
        self.assertIsNone(shock_metrics(self.samples([1, 1])))

    def test_constant_gravity_is_no_shock(self):
        m = shock_metrics(self.samples([1.0] * 50))
        self.assertEqual((m['peak_dev_g'], m['peak_delta_g'], m['rms_g']), (0.0, 0.0, 0.0))

    def test_knock(self):
        values = [1.0] * 20 + [4.0] + [1.0] * 20
        m = shock_metrics(self.samples(values))
        self.assertEqual(m['peak_dev_g'], 3.0)
        self.assertEqual(m['peak_delta_g'], 3.0)
        self.assertGreater(m['rms_g'], 0.0)

    def test_tilt_is_not_a_knock(self):
        # slow change of the orientation: large deviation, but small steps between samples
        m = shock_metrics(self.samples([1.0 + 0.01 * i for i in range(100)]))
        self.assertLess(m['peak_delta_g'], 0.02)
        self.assertGreater(m['peak_dev_g'], 0.4)


# --- the command with a fake printer ----------------------------------------------

class CommandError(Exception):
    pass


class FakeTool:
    def __init__(self, number, x, y, z):
        self.tool_number = number
        self.params = {'params_park_x': x, 'params_park_y': y, 'params_park_z': z,
                       'params_safe_y': 100.0, 'params_path_speed': 2100.0}
        self.original = {}

    def set_parameter(self, name, value):
        self.original.setdefault(name, self.params[name])
        self.params[name] = value

    def reset_parameter(self, name):
        self.params[name] = self.original[name]


class FakeDock:
    """Tool 2 sits in its dock. It locks if the park values are inside the window around the true position."""

    def __init__(self, tool, truth, window):
        self.tool, self.truth, self.window = tool, truth, window
        self.status, self.mounted, self.detected = 'ready', -1, -1
        self.scripts, self.pickups, self.y = [], 0, 100.0
        self.fail_unselect = False
        self.skip_recovery = False
        self.accel = 12000.0

    def locks(self):
        for axis in 'xyz':
            rel = self.tool.params['params_park_' + axis] - self.truth[axis]
            if not self.window[axis][0] - 1e-9 <= rel <= self.window[axis][1] + 1e-9:
                return False
        return True


class FakeGcode:
    error = CommandError

    def __init__(self, dock):
        self.dock, self.info, self.commands = dock, [], {}

    def register_command(self, name, func, when_not_ready=False, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)

    def run_script_from_command(self, script):
        d = self.dock
        d.scripts.append(script)
        word = script.split()[0]
        if word == 'SELECT_TOOL':
            assert "RESTORE_AXIS=''" in script
            d.pickups += 1
            if d.locks():
                d.status, d.mounted, d.detected = 'ready', d.tool.tool_number, d.tool.tool_number
            else:
                d.status, d.mounted, d.detected = 'uninitialized', -1, -1
                d.y = 100.0 if not d.skip_recovery else -20.0
                raise CommandError('Expected tool but active is None')
        elif word == 'UNSELECT_TOOL':
            if d.fail_unselect:
                d.status = 'uninitialized'
                raise CommandError('dropoff failed')
            d.mounted, d.detected = -1, -1
        elif word == 'INITIALIZE_TOOLCHANGER':
            d.status = 'ready'
        elif word == 'SET_VELOCITY_LIMIT':
            d.accel = float(script.split('ACCEL=')[1])


class FakeToolchanger:
    def __init__(self, dock):
        self.dock = dock

    def get_status(self, eventtime):
        d = self.dock
        return {'status': d.status, 'tool_number': d.mounted, 'detected_tool_number': d.detected}


class FakeToolhead:
    def __init__(self, dock):
        self.dock, self.homed = dock, 'xyz'
        self.waits = 0

    def get_status(self, eventtime):
        return {'homed_axes': self.homed, 'position': (0.0, self.dock.y, 0.0, 0.0),
                'max_accel': self.dock.accel}

    def wait_moves(self):
        self.waits += 1

    def dwell(self, delay):
        pass


class FakeReactor:
    def monotonic(self):
        return 0.0


class FakePrintStats:
    state = 'standby'

    def get_status(self, eventtime):
        return {'state': self.state}


class FakeChip:
    """ADXL: every client gets a quiet signal with an optional knock"""

    def __init__(self, knock_g=0.0):
        self.knock_g = knock_g

    def start_internal_client(self):
        chip = self

        class Client:
            def finish_measurements(self):
                pass

            def get_samples(self):
                g = dock_autotune.FREEFALL
                values = [g] * 30 + [g * (1 + chip.knock_g)] + [g] * 30
                return [(i * 0.001, 0.0, 0.0, v) for i, v in enumerate(values)]
        return Client()


class FakePrinter:
    def __init__(self, dock, tool):
        self.objects = {'gcode': FakeGcode(dock), 'toolchanger': FakeToolchanger(dock),
                        'toolhead': FakeToolhead(dock), 'tool T2': tool}
        self.reactor = FakeReactor()

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class FakeConfig:
    def __init__(self, printer, **options):
        self.printer, self.options = printer, options

    def get_printer(self):
        return self.printer

    def _get(self, name, default, cast):
        return cast(self.options.get(name, default))

    def get(self, name, default=None):
        return self._get(name, default, str)

    def getfloat(self, name, default=None, **kw):
        return self._get(name, default, float)

    def getint(self, name, default=None, **kw):
        return self._get(name, default, int)

    def getboolean(self, name, default=None):
        return self.options.get(name, default) in (True, 'True', 'true', '1')


class FakeGcmd:
    error = CommandError

    def __init__(self, **params):
        self.params = {k: str(v) for k, v in params.items()}
        self.responses = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, minval=None):
        return int(self.params[name]) if name in self.params else default

    def get_float(self, name, default=None, above=None, maxval=None, minval=None):
        return float(self.params[name]) if name in self.params else default

    def respond_info(self, msg):
        self.responses.append(msg)

    @property
    def text(self):
        return '\n'.join(self.responses)


WINDOW = {'x': (-0.35, 0.35), 'y': (-0.45, 0.25), 'z': (-0.30, 0.20)}


class CommandTests(unittest.TestCase):
    def make(self, hidden=(0.2, -0.15, 0.1), window=WINDOW, chip=None, **options):
        self.tool = FakeTool(2, 149.6, -29.4, 266.2)
        truth = dict(zip('xyz', (149.6 + hidden[0], -29.4 + hidden[1], 266.2 + hidden[2])))
        self.dock = FakeDock(self.tool, truth, window)
        self.printer = FakePrinter(self.dock, self.tool)
        if chip is not None:
            self.printer.objects['adxl345 T2'] = chip
        self.printer.objects['print_stats'] = self.stats = FakePrintStats()
        options.setdefault('log_file', os.path.join(tempfile.mkdtemp(), 'dock_autotune.jsonl'))
        self.module = dock_autotune.DockAutotune(FakeConfig(self.printer, **options))
        self.truth = truth
        return self.module

    def scan(self, **params):
        gcmd = FakeGcmd(**params)
        self.module.cmd_DOCK_AUTOTUNE(gcmd)
        return gcmd

    def centre(self, axis):
        low, high = WINDOW[axis]
        return self.truth[axis] + (low + high) / 2

    def test_commands_are_registered(self):
        self.make()
        commands = self.printer.objects['gcode'].commands
        self.assertIn('DOCK_AUTOTUNE', commands)
        # 'INTERRUPT' in the name makes Klipper run it while the scan is running
        self.assertIn('DOCK_AUTOTUNE_INTERRUPT', commands)

    def test_scan_centres_the_window(self):
        self.make()
        gcmd = self.scan(T=2, RANGE=0.8)
        for axis in 'xyz':
            self.assertAlmostEqual(self.tool.params['params_park_' + axis], self.centre(axis), places=3)
        self.assertIn('params_park_x: 149.800', gcmd.text)
        # tool is back in its dock, shuttle empty, everything restored
        self.assertEqual((self.dock.mounted, self.dock.status), (-1, 'ready'))
        self.assertEqual(self.tool.params['params_path_speed'], 2100.0)
        self.assertEqual(self.dock.accel, 12000.0)
        self.assertFalse(self.module.active)

    def test_runs_slower_while_scanning(self):
        self.make(speed_factor='0.5', accel='2000')
        speeds, accels = [], []
        real = self.printer.objects['gcode'].run_script_from_command

        def spy(script):
            speeds.append(self.tool.params['params_path_speed'])
            accels.append(self.dock.accel)
            return real(script)
        self.printer.objects['gcode'].run_script_from_command = spy
        self.scan(T=2, AXES='x', RANGE=0.3)
        pickups = [i for i, s in enumerate(self.dock.scripts) if s.startswith('SELECT_TOOL')]
        self.assertTrue(pickups)
        self.assertTrue(all(s == 1050.0 for s in speeds[1:-1]), set(speeds))
        self.assertTrue(all(a == 2000.0 for a in accels[1:-1]), set(accels))

    def test_axes_not_asked_for_are_not_touched(self):
        self.make()
        before = {a: self.tool.params['params_park_' + a] for a in 'yz'}
        self.scan(T=2, AXES='x', RANGE=0.8)
        self.assertAlmostEqual(self.tool.params['params_park_x'], self.centre('x'), places=3)
        self.assertEqual({a: self.tool.params['params_park_' + a] for a in 'yz'}, before)

    def test_window_wider_than_the_range_keeps_the_value(self):
        self.make(window={'x': (-3, 3), 'y': WINDOW['y'], 'z': WINDOW['z']})
        gcmd = self.scan(T=2, AXES='x', RANGE=0.5)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)
        self.assertIn('unchanged', gcmd.text)

    def test_failing_baseline_changes_nothing(self):
        self.make(hidden=(1.5, 0, 0))
        gcmd = self.scan(T=2, RANGE=0.8)
        self.assertIn('STOPPED', gcmd.text)
        self.assertIn('baseline X', gcmd.text)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)
        self.assertEqual(self.dock.pickups, 1)

    def test_interrupt_stops_and_restores(self):
        self.make()
        real = self.module._cycle

        def cycle(tool_number):
            result = real(tool_number)
            if self.module.cycles == 6:
                self.module.cmd_INTERRUPT(FakeGcmd())
            return result
        self.module._cycle = cycle
        gcmd = self.scan(T=2, RANGE=0.8)
        self.assertIn('stopped by DOCK_AUTOTUNE_INTERRUPT', gcmd.text)
        self.assertEqual(self.dock.pickups, 6)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)
        self.assertFalse(self.module.active)

    def test_failed_dropoff_is_not_a_trial_result(self):
        self.make()
        self.dock.fail_unselect = True
        with self.assertRaises(CommandError):
            self.scan(T=2, RANGE=0.8)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)
        self.assertFalse(self.module.active)

    def test_shuttle_not_out_of_the_dock_after_a_failure_aborts(self):
        self.make(hidden=(0.0, 0.0, 0.0), window={'x': (-0.15, 0.15), 'y': WINDOW['y'], 'z': WINDOW['z']})
        self.dock.skip_recovery = True      # the error handler did not get the shuttle out
        gcmd = self.scan(T=2, AXES='x', RANGE=0.8)
        self.assertIn('not at safe Y', gcmd.text)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)

    def test_refuses_without_homing_with_a_tool_on_the_shuttle_and_while_printing(self):
        self.make()
        self.printer.objects['toolhead'].homed = 'xy'
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2)
        self.assertIn('home all axes', str(ctx.exception))
        self.printer.objects['toolhead'].homed = 'xyz'
        self.stats.state = 'printing'
        with self.assertRaises(CommandError):
            self.scan(T=2)
        self.stats.state = 'standby'
        self.dock.mounted = 1
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2)
        self.assertIn('T1 is on the shuttle', str(ctx.exception))
        self.assertEqual(self.dock.pickups, 0)

    def test_tool_of_the_scan_already_on_the_shuttle_is_put_back_first(self):
        self.make()
        self.dock.mounted = 2
        self.scan(T=2, AXES='x', RANGE=0.3)
        self.assertTrue(self.dock.scripts[1].startswith('UNSELECT_TOOL'))

    def test_dry_run_moves_nothing(self):
        self.make()
        gcmd = self.scan(T=2, DRY_RUN=1)
        self.assertIn('DRY_RUN', gcmd.text)
        self.assertEqual(self.dock.scripts, [])
        self.assertEqual(self.dock.pickups, 0)

    def test_max_cycles(self):
        self.make(max_cycles='5')
        gcmd = self.scan(T=2, RANGE=0.8)
        self.assertIn('max_cycles', gcmd.text)
        self.assertEqual(self.dock.pickups, 5)

    def test_status_while_idle(self):
        self.make()
        status = self.module.get_status()
        self.assertFalse(status['active'])
        self.assertEqual(status['phase'], 'idle')

    def test_log_file(self):
        self.make()
        self.scan(T=2, AXES='x', RANGE=0.3)
        with open(self.module.log_file) as f:
            lines = f.read().strip().splitlines()
        self.assertGreater(len(lines), 3)
        self.assertIn('"summary"', lines[-1])

    def test_accelerometer_is_recorded_and_can_stop_the_scan(self):
        self.make(chip=FakeChip(knock_g=2.0), abort_g='5')
        self.scan(T=2, AXES='x', RANGE=0.3, ADXL=1)
        with open(self.module.log_file) as f:
            self.assertIn('"peak_g": 2.0', f.read())
        self.make(chip=FakeChip(knock_g=8.0), abort_g='5')
        gcmd = self.scan(T=2, AXES='x', RANGE=0.3, ADXL=1)
        self.assertIn('above abort_g', gcmd.text)
        self.assertEqual(self.dock.pickups, 1)

    def test_missing_accelerometer_does_not_stop_the_scan(self):
        self.make()
        gcmd = self.scan(T=2, AXES='x', RANGE=0.3, ADXL=1)
        self.assertNotIn('STOPPED', gcmd.text)
        self.assertIn('there is no [adxl345 T2] section', gcmd.text)

    def test_old_error_handler_that_pauses_stops_the_scan(self):
        self.make(hidden=(0.0, 0.0, 0.0), window={'x': (-0.15, 0.15), 'y': WINDOW['y'], 'z': WINDOW['z']})

        class Paused:
            def get_status(self, eventtime):
                return {'is_paused': True}
        # the first failing pickup pauses the print, as the stock error_gcode would
        self.printer.objects['pause_resume'] = Paused()
        gcmd = self.scan(T=2, AXES='x', RANGE=0.8)
        self.assertIn('does not know the scan mode', gcmd.text)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)


if __name__ == '__main__':
    unittest.main()
