# ktc-dock-autotune

Finds the best park position of a [klipper-toolchanger](https://github.com/viesturz/klipper-toolchanger)
(KTC) dock automatically.

**Status: prototype.** Tested with unit tests and in a simulation with the real `toolchanger.py`, and run once on
one printer (Voron with Cartographer, five tools). Read "Safety" before the first run.

## How it works

A tool locks into the shuttle only inside a window of positions. `DOCK_AUTOTUNE` looks for that window per axis
and puts `params_park_x/y/z` of the tool in the middle of it:

1. A *trial* is a pickup (`SELECT_TOOL`, the tool detection at the `verify` waypoint must see the tool) and a
   dropoff (`UNSELECT_TOOL`), `repeats` times. All of them have to lock.
2. Starting at the current value (it has to lock, otherwise the scan stops) the offset moves outwards in `step`
   increments on both sides until a trial fails. Then the interval at the edge is halved down to `resolution`.
3. The centre of the window is tried again and, if it locks, applied. Axes are done one after the other.

The scan runs in the coordinate frame of a real tool change, so the values are valid for printing. The signal is the
tool detection pin. An accelerometer on the tool can be recorded alongside (see "Accelerometer").

If no trial fails on a side within `range_*`, there is no edge: the window is *not closed* and that axis is left
alone. The message says how wide the window is at least. Run it again with a larger range and step, for example
`RANGE=1.2 STEP=0.2`.

## Requirements

* KTC with a `detection_pin` in every `[tool]`.
* A toolchanger `error_gcode` that **leaves the dock** after a failed pickup and does not pause while a scan runs
  (`printer.dock_autotune.active`). With the stock `error_gcode: PAUSE` a failing trial pauses the print and the scan
  stops. [`examples/toolchanger-error-handler.cfg`](examples/toolchanger-error-handler.cfg) is such a handler (it needs
  the Mainsail/Fluidd client macros `PAUSE` and `_CLIENT_VARIABLE`).
* No running or paused print.
* For the prompt: a frontend that understands `// action:prompt_*` (Mainsail, Fluidd, KlipperScreen). Without one use
  `PREPARE=1` or `PREPARE=0`.
* Recommended: `fade_end` in `[bed_mesh]` (for example `fade_start: 1`, `fade_end: 10`, `fade_target: 0`), otherwise
  the mesh shifts the height of the dock.

## Install

```
git clone https://github.com/ChrisFo8390/ktc-dock-autotune.git ~/ktc-dock-autotune
~/ktc-dock-autotune/install.sh
```

The installer links the module into Klipper, copies `dock_autotune.cfg` (your settings, never overwritten) and
`dock_autotune_macro.cfg` (updated on every run), adds both `[include ...]` lines to `printer.cfg` (above
`SAVE_CONFIG`), adds an `[update_manager]` entry to `moonraker.conf` and restarts Klipper (and Moonraker if it changed
`moonraker.conf`). Backups are made before a file is changed, and running it again changes nothing that is already
there. `sudo` has to work without a password, otherwise the script prints the command to run.

| Option | |
|---|---|
| `--no-include` | do not touch `printer.cfg` |
| `--no-macro` | do not install the macro for the macro list |
| `--no-update-manager` | do not touch `moonraker.conf` |
| `--no-restart` | do not restart anything |
| `--uninstall` | remove the link and the include lines (the `.cfg` files and the update manager entry stay) |
| `KLIPPER_PATH=…`, `CONFIG_PATH=…` | other locations than `~/klipper` and `~/printer_data/config` |

Update with `git pull && ./install.sh` in the clone: module and macro file belong together.

## Use

```
DOCK_AUTOTUNE T=1 DRY_RUN=1             # detection and plan, nothing moves
DOCK_AUTOTUNE T=1 AXES=z RANGE=0.4      # first run: one axis, small range, watch it
DOCK_AUTOTUNE T=1                       # x, y and z with the configured ranges
DOCK_AUTOTUNE_INTERRUPT                 # stops after the current trial (M112 stops at once)
```

`DOCK_AUTOTUNE` also is in the macro list of Mainsail, with an input field per parameter (see "Macro list").

**Preparation.** Docks have to be tuned in the state they are used in: bed temperature, levelled gantry, and with a
touch homing the Z zero taken by the touch tool. Without `PREPARE` the command shows a prompt with the detected probe
and the steps; **Prepare and start** runs them (bed temperature 70 °C by default, change it with the buttons or
`BED=`), **The machine is already prepared** skips them. The steps follow a `PRINT_START`:

1. `INITIALIZE_TOOLCHANGER`, a tool on the shuttle goes back into its dock
2. `M190` (bed), `G28`, `QUAD_GANTRY_LEVEL` (or `Z_TILT_ADJUST`), `BED_MESH_CLEAR`
3. touch homing with `touch_tool` (T0 by default, picked up with `SELECT_TOOL`, so AFC is not involved): nozzle heats to
   `touch_temp` while `before_touch` runs (**put your brush macro there**), then the touch command runs
   `touch_repeats` times, the nozzle is turned off and the tool goes back into its dock. Without a touch homing the Z
   zero is taken with `G28 Z` after the levelling.

Detected are Cartographer (`CARTOGRAPHER_TOUCH_HOME`), Beacon (`home_method: contact`, touch is `G28 Z`), eddy current
probes, plain probes and a Z endstop. If Z cannot be homed with an empty shuttle (probe on a tool, `probe:` endstop
without a probe, legacy `[scanner]`), the prompt says so and only offers to continue with a machine you prepared by
hand (`PREPARE=0`). After a preparation the machine stays prepared, so the next run can use `PREPARE=0`.

**Result.** The new values apply until the next restart and are printed as `params_park_x: ...` lines. Copy them into
`toolchanger/tools/T<n>.cfg`. A failing trial prints a red `Expected tool ... but active is None`: that is expected,
it is how the edge is found. A scan takes at most `2 * range / step` trials per axis plus the edge refinement
(`DRY_RUN=1` prints the number), each with `repeats` pickups.

| Parameter | |
|---|---|
| `T=<n>` | tool, required |
| `AXES=xyz` | axes to tune, default all |
| `STEP`, `RESOLUTION`, `RANGE`, `RANGE_X/Y/Z` | override the config (mm) |
| `REPEATS`, `SPEED` | pickups per candidate, factor on `params_path_speed` while scanning |
| `PREPARE=1\|0` | prepare first and scan / the machine is prepared; no prompt |
| `BED`, `SOAK`, `TOUCH=0` | bed temperature (°C), extra wait (s), no touch homing |
| `ADXL=1` | record the accelerometer of the tool |
| `DRY_RUN=1` | only print the plan |

All options with their defaults are in [`config/dock_autotune.cfg`](config/dock_autotune.cfg).

Does the dock position depend on the bed temperature? Scan the same dock at `BED=60` and `BED=100` and compare the
`params_park_z` lines.

## Safety

* **Be at the machine** for the first runs and keep `M112` at hand. A failing pickup next to the window is harmless,
  a position far outside of it can hit the dock, so the default ranges are small.
* Speed and acceleration are reduced during the scan and restored afterwards, also after an error. The park values
  go back to the old ones if the scan stops or fails.
* After a failed pickup the toolchanger is initialized again and checked: shuttle empty, no tool detected, shuttle at
  `params_safe_y`. Anything else stops the scan.
* The scan uses `SELECT_TOOL`/`UNSELECT_TOOL`, not the `T<n>` commands. AFC is not asked to load anything but sees
  the extruder activations. Check that AFC and the toolchanger agree about the tool before the next print.

## Accelerometer (optional)

`ADXL=1` records the accelerometer of the tool during every pickup and dropoff and writes `peak_g` (largest deviation
from the median, in g) to the log. The chips have to be named per tool, `[adxl345 T0]` ... `[adxl345 T4]` (an unnamed
`[adxl345]` in several files is merged into one chip). `abort_g` stops the scan when a trial shows a shock above that
value. Whether the shock tells a good position from a worse one inside the window has to be seen on the machine.

## Macro list

Mainsail, Fluidd and touch UIs list only `[gcode_macro]` sections, not commands that a module registers.
`dock_autotune_macro.cfg` defines `[gcode_macro DOCK_AUTOTUNE]`, which calls the module command `DOCK_AUTOTUNE_RUN` with
all parameters. The module notices that macro and does not register the name itself. Do not add `rename_existing` to
it: Mainsail leaves such macros out of its list. Add the macro to a group under *Interface Settings → Macros*.

## Troubleshooting

| Problem | Check |
|---|---|
| Klipper stops: `Existing command 'DOCK_AUTOTUNE' not found` | old macro file with `rename_existing`: `git pull && ./install.sh` |
| `DOCK_AUTOTUNE` missing in the macro list | `dock_autotune_macro.cfg` included? Reload the page after a restart |
| No `[update_manager]` entry in Mainsail | restart Moonraker; the clone has to be a Git repository of the printer user; see `moonraker.log` |
| `ADXL=1` but "no [adxl345 T<n>] section" | name the chips per tool, see "Accelerometer" |
| Scan stops: "does not know the scan mode" | the `error_gcode` pauses: use the handler from `examples/` |
| Touch homing starts at the height of the dock | `touch_restore_axis: Z` (default) brings Z back after the pickup |

Files written: `~/printer_data/logs/dock_autotune.jsonl` (one line per trial with `tool`, `axis`, `value`, `ok`,
`peak_g`, `error`, a summary per scan). `printer.dock_autotune` has `active`, `tool`, `axis`, `phase`, `trial`, `result`.

## Tests

```
python3 -m unittest discover -s tests
```

## License

GPLv3, see [LICENSE](LICENSE). Use at your own risk: this software moves a toolhead into a dock.
