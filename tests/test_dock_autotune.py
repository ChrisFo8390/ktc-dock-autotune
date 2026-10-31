import os
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'klipper', 'extras'))

import dock_autotune  # noqa: E402
from dock_autotune import (build_plan, build_prompt, command_line, detect_setup, scan_axis,  # noqa: E402
                           shock_metrics, window_center)


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


# --- detection, plan, prompt (pure) ------------------------------------------------

CARTO = {'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}, 'cartographer': {}, 'quad_gantry_level': {},
         'bed_mesh': {}, 'heater_bed': {'max_temp': '130'}}
TOUCH = {'CARTOGRAPHER_TOUCH_HOME', 'G28'}


def plan_of(setup, bed=70., soak=0., touch='CARTOGRAPHER_TOUCH_HOME', temp=150., repeats=2, before=()):
    return build_plan(setup, bed, soak, touch, 0, temp, repeats, before)


class DetectSetupTests(unittest.TestCase):
    def test_cartographer_with_touch(self):
        setup = detect_setup(CARTO, TOUCH)
        self.assertEqual((setup['probe'], setup['touch_cmd'], setup['level_cmd']),
                         ('Cartographer', 'CARTOGRAPHER_TOUCH_HOME', 'QUAD_GANTRY_LEVEL'))
        self.assertTrue(setup['can_prepare'])
        self.assertEqual(setup['max_bed'], 130.0)

    def test_cartographer_without_the_touch_command(self):
        self.assertIsNone(detect_setup(CARTO, {'G28'})['touch_cmd'])

    def test_beacon_touches_only_with_home_method_contact(self):
        base = {'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}}
        self.assertEqual(detect_setup(dict(base, beacon={'home_method': 'contact'}), set())['touch_cmd'], 'G28 Z')
        self.assertIsNone(detect_setup(dict(base, beacon={'home_method': 'proximity'}), set())['touch_cmd'])
        self.assertIsNone(detect_setup(dict(base, beacon={}), set())['touch_cmd'])

    def test_eddy_current_has_no_touch(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'},
                              'probe_eddy_current btt': {}, 'z_tilt': {}}, TOUCH)
        self.assertEqual((setup['probe'], setup['touch_cmd'], setup['level_cmd']),
                         ('Eddy current probe', None, 'Z_TILT_ADJUST'))
        self.assertTrue(setup['can_prepare'])

    def test_physical_endstop_homes_without_a_probe(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'PG10'}}, set())
        self.assertTrue(setup['can_prepare'])
        self.assertIsNone(setup['probe'])
        self.assertFalse(setup['has_bed'])

    def test_probe_endstop_without_a_probe_cannot_home_without_a_tool(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}}, set())
        self.assertFalse(setup['can_prepare'])
        self.assertIn('Z cannot be homed with an empty shuttle', setup['reason'])

    def test_probe_on_a_tool_cannot_home_without_a_tool(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'tool_probe_endstop:z'}, 'tool_probe t0': {}}, set())
        self.assertFalse(setup['can_prepare'])
        self.assertIn('tool_probe', setup['reason'])

    def test_legacy_scanner_is_not_guessed(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}, 'scanner': {}}, set())
        self.assertFalse(setup['can_prepare'])
        self.assertIn('scanner', setup['reason'])

    def test_section_names_are_case_insensitive(self):
        self.assertEqual(detect_setup({'stepper_z': {'endstop_pin': 'probe:x'}, 'Cartographer': {}}, set())['probe'],
                         'Cartographer')


class PlanTests(unittest.TestCase):
    def test_plan_follows_the_print_start(self):
        setup = detect_setup(CARTO, TOUCH)
        plan = plan_of(setup, before=('BRUSH',))
        self.assertEqual(plan, [
            'INITIALIZE_TOOLCHANGER', dock_autotune.EMPTY_SHUTTLE, 'M190 S70', 'G28', 'QUAD_GANTRY_LEVEL',
            'BED_MESH_CLEAR', "SELECT_TOOL T=0 RESTORE_AXIS=''", 'M104 S150', 'BRUSH', 'M109 S150',
            'CARTOGRAPHER_TOUCH_HOME', 'CARTOGRAPHER_TOUCH_HOME', 'M104 S0', "UNSELECT_TOOL RESTORE_AXIS=''"])

    def test_without_touch_the_z_reference_is_taken_again_after_levelling(self):
        plan = plan_of(detect_setup(CARTO, set()), touch=None)
        self.assertEqual(plan[-3:], ['QUAD_GANTRY_LEVEL', 'BED_MESH_CLEAR', 'G28 Z'])
        self.assertFalse(any('SELECT_TOOL' in p for p in plan))

    def test_soak_and_no_bed(self):
        setup = detect_setup(CARTO, TOUCH)
        self.assertIn('G4 P120000', plan_of(setup, soak=120))
        self.assertFalse(any(p.startswith('M190') for p in plan_of(setup, bed=0)))

    def test_machine_without_levelling_and_mesh(self):
        plan = plan_of(detect_setup({'stepper_z': {'endstop_pin': 'PG10'}}, set()), touch=None)
        self.assertEqual(plan, ['INITIALIZE_TOOLCHANGER', dock_autotune.EMPTY_SHUTTLE, 'G28'])


class PromptTests(unittest.TestCase):
    def test_command_line(self):
        params = {'T': '1', 'AXES': 'z', 'BED': '60', 'PREPARE': '0'}
        self.assertEqual(command_line(params, PREPARE=1, BED='75'), 'DOCK_AUTOTUNE T=1 AXES=z BED=75 PREPARE=1')
        self.assertEqual(command_line(params, PREPARE=None, BED=None), 'DOCK_AUTOTUNE T=1 AXES=z')

    def test_prompt_offers_preparation_and_keeps_the_parameters(self):
        setup = detect_setup(CARTO, TOUCH)
        text = '\n'.join(build_prompt(1, {'T': '1', 'AXES': 'z'}, setup, plan_of(setup), 70.,
                                      'CARTOGRAPHER_TOUCH_HOME'))
        self.assertTrue(text.startswith('// action:prompt_begin Dock autotune T1'))
        self.assertTrue(text.rstrip().endswith('// action:prompt_show'))
        self.assertIn('Cartographer', text)
        self.assertIn('Prepare and start (bed 70 C)|DOCK_AUTOTUNE T=1 AXES=z PREPARE=1 BED=70|primary', text)
        self.assertIn('+5 C|DOCK_AUTOTUNE T=1 AXES=z BED=75|', text)
        self.assertIn('-10 C|DOCK_AUTOTUNE T=1 AXES=z BED=60|', text)
        self.assertIn('already prepared|DOCK_AUTOTUNE T=1 AXES=z PREPARE=0|', text)
        self.assertIn('action:prompt_end', text)

    def test_bed_buttons_stay_in_range(self):
        setup = detect_setup(CARTO, TOUCH)
        text = '\n'.join(build_prompt(1, {'T': '1'}, setup, plan_of(setup), 125., None))
        self.assertIn('BED=130|', text)
        self.assertNotIn('BED=135', text)
        low = '\n'.join(build_prompt(1, {'T': '1'}, setup, plan_of(setup), 35., None))
        self.assertIn('-10 C|DOCK_AUTOTUNE T=1 BED=30|', low)

    def test_prompt_without_preparation_possible_only_continues_or_closes(self):
        setup = detect_setup({'stepper_z': {'endstop_pin': 'tool_probe_endstop:z'}}, set())
        text = '\n'.join(build_prompt(3, {'T': '3', 'BED': '80'}, setup, [], 70., None))
        self.assertIn('Preparation is not possible', text)
        self.assertIn('continue|DOCK_AUTOTUNE T=3 PREPARE=0|', text)
        self.assertNotIn('PREPARE=1', text)
        self.assertNotIn('M190', text)


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
        self.dock, self.info, self.commands, self.raw = dock, [], {}, []
        self.ready_gcode_handlers = {}

    def register_command(self, name, func, when_not_ready=False, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)

    def respond_raw(self, msg):
        self.raw.append(msg)

    def run_script_from_command(self, script):
        d = self.dock
        d.scripts.append(script)
        word = script.split()[0]
        if word == 'SELECT_TOOL' and int(script.split('T=')[1].split()[0]) != d.tool.tool_number:
            d.mounted = d.detected = int(script.split('T=')[1].split()[0])     # another tool: not part of the model
        elif word == 'SELECT_TOOL':
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
        elif word == 'G28':
            d.homed = True


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
        if getattr(self.dock, 'homed', False):
            self.homed = 'xyz'
        return {'homed_axes': self.homed, 'position': (0.0, self.dock.y, 0.0, 0.0),
                'max_accel': self.dock.accel}

    def wait_moves(self):
        self.waits += 1

    def dwell(self, delay):
        pass


class FakeReactor:
    def monotonic(self):
        return 0.0


class FakeConfigfile:
    def __init__(self, settings):
        self.settings = settings

    def get_status(self, eventtime):
        return {'settings': self.settings}


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

    def get_prefix_sections(self, prefix):
        class Section:
            def __init__(self, name):
                self.name = name

            def get_name(self):
                return self.name
        return [Section(n) for n in self.options.get('sections', ()) if n.startswith(prefix)]

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

    def get_command_parameters(self):
        return dict(self.params)

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
        params.setdefault('PREPARE', 0)         # the prompt is tested separately
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
        self.assertIn('DOCK_AUTOTUNE_RUN', commands)
        # 'INTERRUPT' in the name makes Klipper run it while the scan is running
        self.assertIn('DOCK_AUTOTUNE_INTERRUPT', commands)

    def test_a_macro_of_that_name_gets_the_name_and_calls_the_run_command(self):
        self.make(sections=['gcode_macro dock_autotune'])
        commands = self.printer.objects['gcode'].commands
        self.assertNotIn('DOCK_AUTOTUNE', commands)
        self.assertIn('DOCK_AUTOTUNE_RUN', commands)
        self.assertIn('DOCK_AUTOTUNE_INTERRUPT', commands)

    def test_other_macros_do_not_take_the_name(self):
        self.make(sections=['gcode_macro DOCK_TUNER', 'gcode_macro DOCK_AUTOTUNE_START'])
        self.assertIn('DOCK_AUTOTUNE', self.printer.objects['gcode'].commands)

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

    # -- preparation like a print start ---------------------------------------------------

    def carto(self, settings=None, commands=TOUCH):
        self.printer.objects['configfile'] = FakeConfigfile(CARTO if settings is None else settings)
        self.printer.objects['gcode'].ready_gcode_handlers = {c: None for c in commands}
        self.printer.objects['tool T0'] = FakeTool(0, 10.0, -29.0, 266.0)

    def test_without_prepare_the_prompt_is_shown_and_nothing_moves(self):
        self.make()
        self.carto()
        self.module.cmd_DOCK_AUTOTUNE(FakeGcmd(T=2, AXES='z'))
        raw = self.printer.objects['gcode'].raw
        self.assertEqual(raw[0], '// action:prompt_begin Dock autotune T2')
        self.assertEqual(raw[-1], '// action:prompt_show')
        self.assertTrue(any('PREPARE=1 BED=70' in r for r in raw))
        self.assertTrue(any('AXES=z' in r for r in raw))
        self.assertEqual(self.dock.scripts, [])
        self.assertFalse(self.module.active)

    def test_prepare_runs_like_the_print_start_and_then_scans(self):
        self.make()
        self.carto()
        self.printer.objects['toolhead'].homed = ''
        self.dock.homed = False
        gcmd = self.scan(T=2, AXES='z', RANGE=0.8, PREPARE=1, BED=75)
        self.assertNotIn('STOPPED', gcmd.text)
        self.assertEqual(self.dock.scripts[:13], [
            'INITIALIZE_TOOLCHANGER', 'M190 S75', 'G28', 'QUAD_GANTRY_LEVEL', 'BED_MESH_CLEAR',
            "SELECT_TOOL T=0 RESTORE_AXIS=''", 'M104 S150', 'M109 S150', 'CARTOGRAPHER_TOUCH_HOME',
            'CARTOGRAPHER_TOUCH_HOME', 'M104 S0', "UNSELECT_TOOL RESTORE_AXIS=''",
            'SET_VELOCITY_LIMIT ACCEL=2000'])
        self.assertTrue(self.dock.scripts[13].startswith('SELECT_TOOL T=2'))
        self.assertAlmostEqual(self.tool.params['params_park_z'], self.centre('z'), places=3)
        self.assertEqual((self.dock.mounted, self.dock.status), (-1, 'ready'))

    def test_prepare_puts_a_tool_on_the_shuttle_back_first(self):
        self.make()
        self.carto()
        self.dock.mounted = 1
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertEqual(self.dock.scripts[:3], ['INITIALIZE_TOOLCHANGER', "UNSELECT_TOOL RESTORE_AXIS=''", 'M190 S70'])

    def test_prepare_homes_before_it_puts_the_tool_back_when_nothing_is_homed(self):
        self.make()
        self.carto()
        self.dock.mounted = 1
        self.printer.objects['toolhead'].homed = ''
        self.dock.homed = False
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertEqual(self.dock.scripts[:4], ['INITIALIZE_TOOLCHANGER', 'G28', "UNSELECT_TOOL RESTORE_AXIS=''",
                                                 'M190 S70'])

    def test_touch_can_be_turned_off(self):
        self.make()
        self.carto()
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1, TOUCH=0)
        self.assertNotIn('CARTOGRAPHER_TOUCH_HOME', self.dock.scripts)
        self.assertIn('G28 Z', self.dock.scripts)

    def test_touch_settings_from_the_config(self):
        self.make(touch_temp='148', touch_repeats='1', before_touch='BRUSH')
        self.carto()
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertEqual(self.dock.scripts[5:11], ["SELECT_TOOL T=0 RESTORE_AXIS=''", 'M104 S148', 'BRUSH',
                                                   'M109 S148', 'CARTOGRAPHER_TOUCH_HOME', 'M104 S0'])

    def test_beacon_contact_homing_is_the_touch(self):
        self.make()
        self.carto({'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}, 'beacon': {'home_method': 'contact'},
                    'z_tilt': {}}, commands=())
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertEqual(self.dock.scripts[:9], ['INITIALIZE_TOOLCHANGER', 'G28', 'Z_TILT_ADJUST',
                                                 "SELECT_TOOL T=0 RESTORE_AXIS=''", 'M104 S150', 'M109 S150',
                                                 'G28 Z', 'G28 Z', 'M104 S0'])

    def test_failure_in_the_preparation_stops_with_a_report(self):
        self.make()
        self.carto()
        real = self.printer.objects['gcode'].run_script_from_command

        def run(script):
            if script == 'QUAD_GANTRY_LEVEL':
                raise CommandError('Probing failed')
            return real(script)
        self.printer.objects['gcode'].run_script_from_command = run
        gcmd = FakeGcmd(T=2, PREPARE=1)
        with self.assertRaises(CommandError):
            self.module.cmd_DOCK_AUTOTUNE(gcmd)
        self.assertEqual(self.dock.pickups, 0)
        self.assertFalse(self.module.active)
        self.assertIn('STOPPED', gcmd.text)

    def test_explicit_prepare_without_a_way_to_home_is_refused(self):
        self.make()
        self.carto({'stepper_z': {'endstop_pin': 'tool_probe_endstop:z'}, 'tool_probe t0': {}})
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2, PREPARE=1)
        self.assertIn('not possible', str(ctx.exception))
        self.assertEqual(self.dock.scripts, [])

    def test_prompt_without_a_way_to_home_offers_only_continue(self):
        self.make()
        self.carto({'stepper_z': {'endstop_pin': 'tool_probe_endstop:z'}, 'tool_probe t0': {}})
        self.module.cmd_DOCK_AUTOTUNE(FakeGcmd(T=2))
        text = '\n'.join(self.printer.objects['gcode'].raw)
        self.assertIn('Preparation is not possible', text)
        self.assertNotIn('PREPARE=1', text)
        self.assertEqual(self.dock.scripts, [])

    def test_prepare_0_needs_a_prepared_machine(self):
        self.make()
        self.carto()
        self.printer.objects['toolhead'].homed = 'xy'
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2, PREPARE=0)
        self.assertIn('PREPARE=1', str(ctx.exception))

    def test_no_prompt_and_no_preparation_while_printing(self):
        self.make()
        self.carto()
        self.stats.state = 'printing'
        for params in ({'T': 2}, {'T': 2, 'PREPARE': 1}):
            with self.assertRaises(CommandError):
                self.module.cmd_DOCK_AUTOTUNE(FakeGcmd(**params))
        self.assertEqual(self.printer.objects['gcode'].raw, [])
        self.assertEqual(self.dock.scripts, [])

    def test_bed_above_the_heater_limit_is_refused(self):
        self.make()
        self.carto()
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2, PREPARE=1, BED=200)
        self.assertIn('max_temp', str(ctx.exception))

    def test_dry_run_shows_the_preparation_and_moves_nothing(self):
        self.make()
        self.carto()
        self.printer.objects['toolhead'].homed = ''
        gcmd = FakeGcmd(T=2, DRY_RUN=1)
        self.module.cmd_DOCK_AUTOTUNE(gcmd)
        self.assertIn('M190 S70', gcmd.text)
        self.assertIn('Cartographer', gcmd.text)
        self.assertIn('nothing moved', gcmd.text)
        self.assertEqual((self.dock.scripts, self.printer.objects['gcode'].raw), ([], []))


if __name__ == '__main__':
    unittest.main()
