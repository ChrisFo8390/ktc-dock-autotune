import configparser
import os
import unittest

try:
    import jinja2
except ImportError:         # pragma: no cover
    jinja2 = None

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(REPO, 'config', 'dock_autotune_macro.cfg')


class MacroError(Exception):
    pass


def read_macro():
    """The [gcode_macro DOCK_AUTOTUNE] section the way Klipper reads it: comment lines are dropped"""
    parser = configparser.RawConfigParser(strict=False, comment_prefixes=('#', ';'), inline_comment_prefixes=())
    with open(CFG, encoding='utf-8') as f:
        parser.read_string(f.read())
    return dict(parser.items('gcode_macro DOCK_AUTOTUNE'))


def render(params):
    """Render the macro with the Jinja environment of Klipper's gcode_macro ({% %} for blocks, { } for values)"""
    env = jinja2.Environment('{%', '%}', '{', '}')

    def raise_error(msg):
        raise MacroError(msg)
    rawparams = ' '.join('%s=%s' % kv for kv in params.items())
    template = env.from_string(read_macro()['gcode'])
    out = template.render(params=params, rawparams=rawparams, action_raise_error=raise_error)
    return [line.strip() for line in out.splitlines() if line.strip()]


@unittest.skipIf(jinja2 is None, 'jinja2 is not installed')
class MacroTests(unittest.TestCase):
    def test_takes_the_name_of_the_module_command_and_renames_it(self):
        macro = read_macro()
        self.assertEqual(macro['rename_existing'], 'DOCK_AUTOTUNE_BASE')
        self.assertTrue(macro['description'])

    def test_forwards_all_parameters(self):
        self.assertEqual(render({'T': '1', 'AXES': 'z', 'RANGE': '0.4', 'BED': '75', 'PREPARE': '1'}),
                         ['DOCK_AUTOTUNE_BASE T=1 AXES=z RANGE=0.4 BED=75 PREPARE=1'])

    def test_parameters_without_a_field_are_forwarded_too(self):
        self.assertEqual(render({'T': '2', 'RANGE_Z': '0.3'}), ['DOCK_AUTOTUNE_BASE T=2 RANGE_Z=0.3'])

    def test_tool_is_required(self):
        for params in ({}, {'AXES': 'z'}, {'T': ''}):
            with self.assertRaises(MacroError) as ctx:
                render(params)
            self.assertIn('T=<tool number>', str(ctx.exception))

    def test_every_field_of_the_ui_is_a_parameter_of_the_macro(self):
        # Mainsail makes an input field of every params.<NAME> in the macro
        gcode = read_macro()['gcode']
        for name in ('T', 'AXES', 'RANGE', 'STEP', 'REPEATS', 'SPEED', 'BED', 'SOAK', 'PREPARE', 'TOUCH', 'ADXL',
                     'DRY_RUN'):
            self.assertIn('params.%s' % name, gcode)


if __name__ == '__main__':
    unittest.main()
