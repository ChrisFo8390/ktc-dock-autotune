import os
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'klipper', 'extras'))

import dock_autotune  # noqa: E402
from dock_autotune import (build_plan, build_prompt, command_line, csv_snippet, detect_setup,  # noqa: E402
                           knock_metrics, map_table, median, scan_axis, shock_metrics, window_center,
                           zero_crossings)


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

    def test_bisects_down_to_the_resolution(self):
        trial = window_trial(-0.35, 0.25)
        edges = scan_axis(trial, 0.2, 1.0, resolution=0.05)
        self.assertEqual((edges['minus'], edges['plus']), (-0.35, 0.25))
        # coarse steps 0.2, 0.4 (bad) then 0.3 (bad), 0.25 (good): 4 trials on the plus side
        self.assertEqual([o for o in trial.calls if o > 0], [0.2, 0.4, 0.3, 0.25])

    def test_a_coarse_step_without_a_smaller_resolution_halves_once(self):
        trial = window_trial(-0.35, 0.25)
        edges = scan_axis(trial, 0.2, 1.0)
        self.assertEqual(edges['plus'], 0.2)
        self.assertEqual([o for o in trial.calls if o > 0], [0.2, 0.4, 0.3])

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


# --- ADXL map (pure) -----------------------------------------------------------------

G = dock_autotune.FREEFALL


def gravity_samples(seconds=2.0, rate=1000, knock_at=None, knock=(0.0, 0.0, 0.0), ramp=0.0, t0=0.0):
    """Samples in mm/s^2: -1 g on z (the chip is upside down), a slow ramp on x, one knock (g) at a time"""
    out = []
    for i in range(int(seconds * rate)):
        t = i / rate
        x, y, z = ramp * t, 0.0, -1.0
        if knock_at is not None and abs(t - knock_at) < 0.5 / rate:
            x, y, z = x + knock[0], y + knock[1], z + knock[2]
        out.append((t0 + t, x * G, y * G, z * G))
    return out


class KnockMetricsTests(unittest.TestCase):
    def test_a_g_is_9806_65_mm_per_s2(self):
        # Klipper reports mm/s^2: 1 g is 9806.65, not 9.80665
        self.assertAlmostEqual(G, 9806.65)
        metrics = shock_metrics([(i * 0.001, 0.0, 0.0, -G * (1 + (i == 30))) for i in range(60)])
        self.assertAlmostEqual(metrics['peak_dev_g'], 1.0, places=2)

    def test_gravity_and_slow_motion_are_not_a_knock(self):
        metrics = knock_metrics(gravity_samples(ramp=0.2), 0.0, 2.0)
        self.assertLess(metrics['peak_g'], 0.01)

    def test_finds_the_knock_with_time_and_signed_components(self):
        metrics = knock_metrics(gravity_samples(knock_at=1.0, knock=(0.3, -0.1, 0.5)), 0.0, 2.0)
        self.assertAlmostEqual(metrics['t_peak'], 1.0, places=2)
        self.assertAlmostEqual(metrics['x'], 0.3, delta=0.02)
        self.assertAlmostEqual(metrics['y'], -0.1, delta=0.02)
        self.assertAlmostEqual(metrics['z'], 0.5, delta=0.02)
        self.assertAlmostEqual(metrics['peak_g'], 0.59, delta=0.03)
        self.assertEqual(metrics['rate_hz'], 1000)

    def test_the_sign_follows_the_direction(self):
        metrics = knock_metrics(gravity_samples(knock_at=0.5, knock=(-0.3, 0.0, -0.4)), 0.0, 2.0)
        self.assertLess(metrics['x'], 0)
        self.assertLess(metrics['z'], 0)

    def test_only_the_window_is_looked_at(self):
        samples = gravity_samples(knock_at=1.5, knock=(0.0, 0.0, 0.5))
        self.assertLess(knock_metrics(samples, 0.0, 1.0)['peak_g'], 0.01)
        self.assertGreater(knock_metrics(samples, 1.0, 2.0)['peak_g'], 0.4)
        self.assertAlmostEqual(knock_metrics(samples, 1.0, 2.0)['t_peak'], 0.5, places=2)

    def test_too_few_samples(self):
        self.assertIsNone(knock_metrics(gravity_samples(seconds=0.01), 0.0, 1.0))
        self.assertEqual(csv_snippet(gravity_samples(seconds=0.01), 0.0, 1.0, 0.0), [])

    def test_csv_snippet_is_a_window_around_the_peak(self):
        samples = gravity_samples(knock_at=1.0, knock=(0.0, 0.0, 0.5))
        lines = csv_snippet(samples, 0.0, 2.0, 1.0, half=0.1)
        self.assertEqual(lines[0], 'time_s,x_g,y_g,z_g,res_x_g,res_y_g,res_z_g')
        times = [float(l.split(',')[0]) for l in lines[1:]]
        self.assertAlmostEqual(min(times), 0.9, places=2)
        self.assertAlmostEqual(max(times), 1.1, places=2)
        self.assertAlmostEqual(float(lines[1].split(',')[3]), -1.0, places=3)

    def test_median(self):
        self.assertEqual(median([3, 1, 2]), 2)
        self.assertEqual(median([1, 2, 3, 4]), 2.5)
        self.assertIsNone(median([]))


def map_row(offset, ok=True, z=0.0, x=0.0, peak=0.2):
    return {'offset': offset, 'value': 266.2 + offset, 'locked': 3 if ok else 0, 'total': 3, 'ok': ok,
            'pick': {'peak_g': peak, 'x': x, 'y': 0.0, 'z': z, 't_peak': 0.4, 'rms_g': 0.01} if ok else None,
            'drop': {'peak_g': 0.1} if ok else None}


class MapTableTests(unittest.TestCase):
    def test_zero_crossing_is_interpolated(self):
        rows = [map_row(-0.5, z=-0.4), map_row(0.0, z=-0.1), map_row(0.25, z=0.15), map_row(0.5, z=0.4)]
        self.assertEqual(zero_crossings(rows, 'z'), [0.1])
        self.assertEqual(zero_crossings(rows, 'x'), [])

    def test_failed_positions_are_left_out_of_the_crossings(self):
        rows = [map_row(-0.5, ok=False), map_row(0.0, z=-0.1), map_row(0.25, z=0.1)]
        self.assertEqual(zero_crossings(rows, 'z'), [0.125])

    def test_table(self):
        rows = [map_row(0.25, z=0.15, peak=0.3), map_row(-0.25, z=-0.1, peak=0.1), map_row(0.5, ok=False)]
        text = '\n'.join(map_table(0, 'z', rows, 0.05))
        self.assertIn('ADXL map T0 Z', text)
        self.assertIn('50 ms high-pass', text)
        self.assertLess(text.index('-0.25'), text.index('+0.25'))        # sorted by offset
        self.assertIn('3/3', text)
        self.assertIn('0/3', text)
        self.assertIn('Quietest pickup: -0.25 mm (0.100 g)', text)
        self.assertIn('Component z changes sign at -0.05 mm', text)
        self.assertIn('Nothing is changed by a map', text)


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
            'BED_MESH_CLEAR', "SELECT_TOOL T=0 RESTORE_AXIS=Z", 'M104 S150', 'BRUSH', 'M109 S150',
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
        self.clock = 0.0

    def get_status(self, eventtime):
        if getattr(self.dock, 'homed', False):
            self.homed = 'xyz'
        return {'homed_axes': self.homed, 'position': (0.0, self.dock.y, 0.0, 0.0),
                'max_accel': self.dock.accel}

    def wait_moves(self):
        self.waits += 1
        self.clock += 0.5           # every wait takes half a second of print time

    def get_last_move_time(self):
        return self.clock

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


class MapChip:
    """ADXL of a tool: -1 g on z, at 0.25 s after the start of a cycle the pickup knocks with a strength that
    depends on how far the park values are from the true position, at 0.75 s a small knock of the dropoff"""

    def __init__(self, dock, toolhead, gain=(1.0, 1.0, 2.0)):
        self.dock, self.toolhead, self.gain = dock, toolhead, gain

    def start_internal_client(self):
        chip, start = self, self.toolhead.clock

        class Client:
            def finish_measurements(self):
                pass

            def get_samples(self):
                end = chip.toolhead.clock
                knock = [chip.gain[i] * (chip.dock.tool.params['params_park_' + a] - chip.dock.truth[a])
                         for i, a in enumerate('xyz')]
                out = []
                for i in range(int((end - start) * 1000) + 1):
                    t = i / 1000.0
                    x, y, z = 0.0, 0.0, -1.0
                    if abs(t - 0.25) < 0.0005:
                        x, y, z = x + knock[0], y + knock[1], z + knock[2]
                    if abs(t - 0.75) < 0.0005:
                        z += 0.1
                    out.append((start + t, x * G, y * G, z * G))
                return out
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

    def test_help_texts_have_no_html_tags(self):
        # the console of Mainsail shows text as HTML: <s> strikes everything through, <n> disappears
        import re
        self.assertIsNone(re.search(r'<[A-Za-z/]', dock_autotune.DockAutotune.cmd_DOCK_AUTOTUNE_help))
        with open(os.path.join(REPO, 'klipper', 'extras', 'dock_autotune.py'), encoding='utf-8') as f:
            strings = re.findall(r'"([^"' + chr(10) + r']*)"', f.read())
        offenders = [t for t in strings if re.search(r'<[A-Za-z/]', t) and not t.startswith('//')]
        self.assertEqual(offenders, [])

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

    def test_a_window_wider_than_the_range_says_how_wide_and_what_to_do(self):
        self.make(window={'x': (-3, 3), 'y': WINDOW['y'], 'z': WINDOW['z']})
        gcmd = self.scan(T=2, AXES='x', RANGE=0.4)
        self.assertIn('window not closed, value kept', gcmd.text)
        self.assertIn('raise RANGE', gcmd.text)
        self.assertIn('-0.40 mm (open, no failure up to here)', gcmd.text)
        self.assertIn('at least 0.80 mm wide', gcmd.text)
        self.assertEqual(self.tool.params['params_park_x'], 149.6)

    def test_coarse_step_and_resolution_find_the_edge_with_few_trials(self):
        self.make(window={'x': (-0.35, 0.25), 'y': WINDOW['y'], 'z': WINDOW['z']}, hidden=(0, 0, 0))
        gcmd = self.scan(T=2, AXES='x', RANGE=1.0, STEP=0.2, RESOLUTION=0.05, REPEATS=1)
        self.assertIn('window -0.350 .. +0.250', gcmd.text)
        self.assertAlmostEqual(self.tool.params['params_park_x'], 149.55, places=3)

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

    # -- ADXL map ----------------------------------------------------------------------------

    def make_map(self, **kw):
        self.make(**kw)
        self.printer.objects['adxl345 T2'] = MapChip(self.dock, self.printer.objects['toolhead'])

    def test_map_shows_the_knock_over_the_positions_and_changes_nothing(self):
        self.make_map(window={'x': WINDOW['x'], 'y': WINDOW['y'], 'z': (-1.0, 1.0)})
        gcmd = self.scan(T=2, MODE='map', AXES='z', RANGE=0.5, STEP=0.25, REPEATS=1)
        self.assertIn('ADXL map T2 Z', gcmd.text)
        self.assertIn('ADXL map finished after 5 pickups, the park values are unchanged', gcmd.text)
        # the true position of z is 0.1 above the start: the z component changes sign there
        self.assertIn('Component z changes sign at +0.10 mm', gcmd.text)
        self.assertIn('Quietest pickup: +0.00 mm', gcmd.text)
        self.assertEqual(self.tool.params['params_park_z'], 266.2)
        self.assertEqual(self.dock.pickups, 5)

    def test_map_order_and_repeats(self):
        self.make_map(window={'x': WINDOW['x'], 'y': WINDOW['y'], 'z': (-1.0, 1.0)})
        seen = []
        real = self.tool.set_parameter

        def spy(name, value):
            if name == 'params_park_z':
                seen.append(round(value - 266.2, 3))
            return real(name, value)
        self.tool.set_parameter = spy
        self.scan(T=2, MODE='map', AXES='z', RANGE=0.5, STEP=0.25, REPEATS=2)
        self.assertEqual(seen[:6], [0.0, 0.25, -0.25, 0.5, -0.5, 0.0])        # the last one is the restore
        self.assertEqual(self.dock.pickups, 10)

    def test_map_stops_a_side_after_failures_in_a_row(self):
        # the window ends 0.3 above and 0.4 below the start: +0.5, +0.75, +1.0 fail, and from the second one on the
        # side is skipped
        self.make_map(window={'x': WINDOW['x'], 'y': WINDOW['y'], 'z': (-0.5, 0.2)}, map_fail_stop='2')
        gcmd = self.scan(T=2, MODE='map', AXES='z', RANGE=1.5, STEP=0.25, REPEATS=1)
        offsets = [float(line.split()[0]) for line in gcmd.text.splitlines() if line[:1] in ' +-' and '/' in line]
        self.assertIn(0.25, offsets)
        self.assertIn(0.5, offsets)
        self.assertNotIn(1.0, offsets)          # skipped after 0.5 and 0.75 failed
        self.assertIn(-0.25, offsets)
        self.assertIn('0/1', gcmd.text)

    def test_map_needs_the_accelerometer(self):
        self.make()
        with self.assertRaises(CommandError) as ctx:
            self.scan(T=2, MODE='map', AXES='z')
        self.assertIn('MODE=map needs the accelerometer [adxl345 T2]', str(ctx.exception))
        self.assertEqual(self.dock.scripts, [])

    def test_unknown_mode(self):
        self.make_map()
        with self.assertRaises(CommandError):
            self.scan(T=2, MODE='nonsense')

    def test_map_with_a_failing_baseline_stops(self):
        self.make_map(hidden=(0.0, 0.0, 1.5))
        gcmd = self.scan(T=2, MODE='map', AXES='z', RANGE=0.5, STEP=0.25, REPEATS=1)
        self.assertIn('STOPPED', gcmd.text)
        self.assertIn('baseline Z', gcmd.text)
        self.assertEqual(self.tool.params['params_park_z'], 266.2)

    def test_raw_snippets_are_written(self):
        self.make_map(window={'x': WINDOW['x'], 'y': WINDOW['y'], 'z': (-1.0, 1.0)})
        self.scan(T=2, MODE='map', AXES='z', RANGE=0.25, STEP=0.25, REPEATS=1, RAW=1)
        folder = os.path.join(os.path.dirname(self.module.log_file), 'dock_autotune_adxl')
        files = sorted(os.listdir(folder))
        self.assertEqual(files, ['T2_z_+0.000_r1_pickup.csv', 'T2_z_+0.250_r1_pickup.csv', 'T2_z_-0.250_r1_pickup.csv'])
        with open(os.path.join(folder, files[0])) as f:
            self.assertTrue(f.readline().startswith('time_s,x_g'))

    def test_map_rows_go_to_the_log(self):
        self.make_map(window={'x': WINDOW['x'], 'y': WINDOW['y'], 'z': (-1.0, 1.0)})
        self.scan(T=2, MODE='map', AXES='z', RANGE=0.25, STEP=0.25, REPEATS=1)
        with open(self.module.log_file) as f:
            text = f.read()
        self.assertEqual(text.count('"map"'), 3)
        self.assertIn('"peak_g"', text)

    def test_dry_run_of_a_map(self):
        self.make_map()
        gcmd = self.scan(T=2, MODE='map', AXES='z', DRY_RUN=1)
        self.assertIn('ADXL map', gcmd.text)
        self.assertIn('nothing moved', gcmd.text)
        self.assertEqual(self.dock.scripts, [])

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
            "SELECT_TOOL T=0 RESTORE_AXIS=Z", 'M104 S150', 'M109 S150', 'CARTOGRAPHER_TOUCH_HOME',
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

    def test_the_touch_tool_brings_z_back_like_the_t_command(self):
        self.make()
        self.carto()
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertIn("SELECT_TOOL T=0 RESTORE_AXIS=Z", self.dock.scripts)
        self.make(touch_restore_axis='')
        self.carto()
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertIn("SELECT_TOOL T=0 RESTORE_AXIS=''", self.dock.scripts)

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
        self.assertEqual(self.dock.scripts[5:11], ["SELECT_TOOL T=0 RESTORE_AXIS=Z", 'M104 S148', 'BRUSH',
                                                   'M109 S148', 'CARTOGRAPHER_TOUCH_HOME', 'M104 S0'])

    def test_beacon_contact_homing_is_the_touch(self):
        self.make()
        self.carto({'stepper_z': {'endstop_pin': 'probe:z_virtual_endstop'}, 'beacon': {'home_method': 'contact'},
                    'z_tilt': {}}, commands=())
        self.scan(T=2, AXES='z', RANGE=0.3, PREPARE=1)
        self.assertEqual(self.dock.scripts[:9], ['INITIALIZE_TOOLCHANGER', 'G28', 'Z_TILT_ADJUST',
                                                 "SELECT_TOOL T=0 RESTORE_AXIS=Z", 'M104 S150', 'M109 S150',
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
