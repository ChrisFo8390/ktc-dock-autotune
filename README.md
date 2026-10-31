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
tool detection pin. An accelerometer on the tool can show how hard the tool seats at each position (see "ADXL map").

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

The installer links the module into Klipper and puts its files into a folder of your config directory:

```
printer_data/config/dock_autotune/
    dock_autotune.cfg          your settings (never overwritten)
    dock_autotune_macro.cfg    the macro for the macro list (updated on every run)
    results/                   one result file per run
```

It adds `[include dock_autotune/dock_autotune.cfg]` and `[include dock_autotune/dock_autotune_macro.cfg]` to
`printer.cfg` (above `SAVE_CONFIG`), adds an `[update_manager]` entry to `moonraker.conf` and restarts Klipper (and
Moonraker if it changed `moonraker.conf`). Backups are made before a file is changed, and running it again changes
nothing that is already there. An older version had both files directly in the config directory: the installer
moves your settings into the folder and changes the include lines. `sudo` has to work without a password, otherwise
the script prints the command to run.

| Option | |
|---|---|
| `--no-include` | do not touch `printer.cfg` |
| `--no-macro` | do not install the macro for the macro list |
| `--no-update-manager` | do not touch `moonraker.conf` |
| `--no-restart` | do not restart anything |
| `--uninstall` | remove the link and the include lines (the folder `dock_autotune/` and the update manager entry stay) |
| `KLIPPER_PATH=…`, `CONFIG_PATH=…` | other locations than `~/klipper` and `~/printer_data/config` |

Update with `git pull && ./install.sh` in the clone: module and macro file belong together. (The update manager of
Mainsail updates the module only; the installer also refreshes the macro file.)

## Use

```
DOCK_AUTOTUNE T=1 DRY_RUN=1             # detection and plan, nothing moves
DOCK_AUTOTUNE T=1 AXES=z RANGE=0.4      # first run: one axis, small range, watch it
DOCK_AUTOTUNE T=1                       # x, y and z with the configured ranges
DOCK_AUTOTUNE_INTERRUPT                 # stops after the current trial (M112 stops at once)
```

What the command does without further parameters is set in `dock_autotune.cfg`: `mode` (scan or map), `axes`,
`prepare` (ask, yes, no), `adxl`, `adxl_raw`, the ranges and so on. With `mode: map`, `axes: z` and `prepare: no`
the ADXL map of the Z axis is `DOCK_AUTOTUNE T=0`. A parameter of the call always wins.

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
`toolchanger/tools/T<n>.cfg`. The same numbers are in the result file (see "Result file"). A failing trial prints a red `Expected tool ... but active is None`: that is expected,
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
| `ADXL=1` | record the accelerometer of the tool during a scan (`peak_g` in the log) |
| `MODE=map`, `RAW=1` | ADXL map instead of a scan, with CSV snippets (see "ADXL map") |
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

## ADXL map (optional)

The detection pin only says "locked" or "not locked". The accelerometer on the tool also sees *how* it seats: when the
shuttle lifts the tool out of the dock it settles into the coupling with a small knock. `MODE=map` shows that knock for
a grid of positions and changes nothing:

```
DOCK_AUTOTUNE T=0 MODE=map AXES=z PREPARE=0 RAW=1
```

The grid goes `map_range` (1.5 mm) to both sides of the current position in `map_step` (0.25 mm) steps, with
`map_repeats` (3) pickups each. Set these once in `dock_autotune.cfg`; `RANGE=`, `STEP=` and `REPEATS=` override them for
a single call.

For every position (0, +step, -step, +2 step, ...) it does `REPEATS` pickups and dropoffs with the accelerometer
recording and prints one line per position: how many pickups locked, the strongest knock of the pickup (`peak`, in g),
its signed components x, y, z at the peak, the time of the peak after the start of the pickup, and the strongest knock
of the dropoff. Gravity and the smooth acceleration of the moves are removed with a 50 ms high-pass
(`adxl_window`), so what is left is what happens fast. Values are medians over the repeats. A side of the grid ends
after `map_fail_stop` failed positions in a row. At the end it names the quietest position and where a component
changes sign. `RAW=1` (or `adxl_raw: True`) also writes the raw samples around the peak of every pickup as CSV to
`~/printer_data/logs/dock_autotune_adxl/`.

Whether the knock has a minimum, or a sign change that shows the direction of an error, inside the window has to be
seen on the machine: with a self-centring coupling it may be flat. The map is for looking at that, nothing is applied.

Requirements: the chips named per tool, `[adxl345 T0]` ... `[adxl345 T4]` (an unnamed `[adxl345]` in several files is
merged into one chip). The components are in the axes of the chip after `axes_map`; the sign of gravity on the
vertical axis tells how the chip sits (a chip that reads -1 g on z points down). Test a chip with
`ACCELEROMETER_QUERY CHIP=T0`. Klipper reports mm/s², the module converts to g (1 g = 9806.65 mm/s²).

With `ADXL=1` a normal scan records the accelerometer too and writes `peak_g` per trial to the log. `abort_g` stops the
scan when a trial shows a shock above that value.

## Result file

Every run that took measurements writes one text file to `printer_data/config/dock_autotune/results/`, named like
`T0_map_20261031_194500.txt` (Mainsail: *Machine*, folder `dock_autotune/results`). It holds

* the status of the run (finished, or stopped and why), the settings and the number of pickups,
* the **park positions** of the tool: before, after and what happened to each axis, and the lines to put into
  `toolchanger/tools/T<n>.cfg`,
* for a map the table of the **ADXL measurements** per position, for a scan the trials in the order they were made
  (with `peak_g` if the accelerometer was on),
* all numbers once more as CSV, to paste into a spreadsheet.

A run that is stopped writes the file too, with what was measured until then. `results_dir:` in `dock_autotune.cfg`
changes the folder, an empty value switches the file off. The folder is part of the config directory, so a backup of
that directory (Klipper backup, git) contains the results as well.

## Macro list

Mainsail, Fluidd and touch UIs list only `[gcode_macro]` sections, not commands that a module registers.
`dock_autotune_macro.cfg` defines `[gcode_macro DOCK_AUTOTUNE]`, which calls the module command `DOCK_AUTOTUNE_RUN` with
all parameters. The module notices that macro and does not register the name itself. Do not add `rename_existing` to
it: Mainsail leaves such macros out of its list. Add the macro to a group under *Interface Settings → Macros*.

## Troubleshooting

| Problem | Check |
|---|---|
| Klipper stops: `Existing command 'DOCK_AUTOTUNE' not found` | old macro file with `rename_existing`: `git pull && ./install.sh` |
| `DOCK_AUTOTUNE` missing in the macro list | `dock_autotune/dock_autotune_macro.cfg` included? Reload the page after a restart |
| No `[update_manager]` entry in Mainsail | restart Moonraker; the clone has to be a Git repository of the printer user; see `moonraker.log` |
| `ADXL=1` but "no [adxl345 T<n>] section" | name the chips per tool, see "ADXL map" |
| Scan stops: "does not know the scan mode" | the `error_gcode` pauses: use the handler from `examples/` |
| Touch homing starts at the height of the dock | `touch_restore_axis: Z` (default) brings Z back after the pickup |

Files written: the result files (see above) and `~/printer_data/logs/dock_autotune.jsonl` (one line per trial with `tool`, `axis`, `value`, `ok`,
`peak_g`, `error`, a summary per scan). `printer.dock_autotune` has `active`, `tool`, `axis`, `phase`, `trial`, `result`.

## Tests

```
python3 -m unittest discover -s tests
```

## License

GPLv3, see [LICENSE](LICENSE). Use at your own risk: this software moves a toolhead into a dock.
