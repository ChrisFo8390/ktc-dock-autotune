# ktc-dock-autotune

Measures the docks of a [klipper-toolchanger](https://github.com/viesturz/klipper-toolchanger) (KTC) toolchanger and
finds the best park position of every tool. It was written for a StealthChanger-style shuttle (the tool hangs in its
dock and the shuttle takes it up with three vertical pins), and works with any KTC dock whose pickup path has a
`verify` waypoint.

**Status: prototype.** Tested with unit tests, in a simulation with the real `toolchanger.py`, and on one printer (Voron
Tallboi with Cartographer, five tools). Read "Safety" before the first run.

## What it does

`DOCK_AUTOTUNE` has two modes (`MODE=` in the command, or `mode:` in `dock_autotune.cfg`):

* **seat** (default). The empty shuttle drives under the tool, lifts into it up to the `verify` waypoint, reads the tool
  detection and goes down again. The tool never leaves its dock. The accelerometer of the tool records the contact, so
  the result file shows **at which height** shuttle and tool touch and **how the tool is pushed sideways** while the
  pins find their holes, for the current park position and for positions along X and Y. Nothing is changed.
* **scan**. Looks for the window of positions in which the tool locks and puts `params_park_x/y/z` in the middle of it.
  It does full pickups and dropoffs and uses only the tool detection pin. Rounded pins find their way in over a wide
  range, so on such a shuttle the window can be wider than any range you dare to scan: the axis is then left alone and
  the message says how wide the window is at least. The scan suits docks with a narrow window.

Both modes can run for one tool, for several, or for all, with one preparation (bed temperature, homing, levelling,
touch homing) and one result file.

## Requirements

* KTC with a `detection_pin` in every `[tool]` and a `verify` waypoint in `params_pickup_path`, reached by a vertical
  move (seat test).
* An accelerometer on every tool, named per tool: `[adxl345 T0]` ... `[adxl345 T4]` (seat test). Test a chip with
  `ACCELEROMETER_QUERY CHIP=T0`; at rest it reads about 1 g on the vertical axis. Klipper reports mm/s², the module
  converts to g (1 g = 9806.65 mm/s²).
* No running or paused print.
* For the prompts: a frontend that understands `// action:prompt_*` (Mainsail, Fluidd, KlipperScreen). Without one use
  `T=`/`TOOLS=` and `PREPARE=1` or `PREPARE=0` in the command.
* Scan only: a toolchanger `error_gcode` that **leaves the dock** after a failed pickup and does not pause while a scan
  runs (`printer.dock_autotune.active`). With the stock `error_gcode: PAUSE` a failing trial pauses the print and the
  scan stops. [`examples/toolchanger-error-handler.cfg`](examples/toolchanger-error-handler.cfg) is such a handler (it
  needs the Mainsail/Fluidd client macros `PAUSE` and `_CLIENT_VARIABLE`).
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
nothing that is already there. Files of an older version that sit directly in the config directory are moved into the
folder and the include lines are changed. `sudo` has to work without a password, otherwise the script prints the
command to run.

| Option | |
|---|---|
| `--no-include` | do not touch `printer.cfg` |
| `--no-macro` | do not install the macro for the macro list |
| `--no-update-manager` | do not touch `moonraker.conf` |
| `--no-restart` | do not restart anything |
| `--uninstall` | remove the link and the include lines (the folder `dock_autotune/` and the update manager entry stay) |
| `KLIPPER_PATH=…`, `CONFIG_PATH=…` | other locations than `~/klipper` and `~/printer_data/config` |

**Update.** The update manager of Mainsail pulls the clone and restarts Klipper, which updates the module. It does not
run the installer: after an update that changes `dock_autotune_macro.cfg` run `git pull && ./install.sh` in the clone
once. New options in `dock_autotune.cfg` need no change, missing ones use their defaults (the template in
[`config/dock_autotune.cfg`](config/dock_autotune.cfg) lists all of them).

## Use

**From the UI.** A click on `DOCK_AUTOTUNE` in the macro list (Mainsail, Fluidd) opens a prompt with one button per tool
and "All tools"; the next prompt asks whether to prepare the machine. Both buttons close the prompt and the run starts.
Set `tools:` in `dock_autotune.cfg` to skip the first question.

**From the console.**

```
DOCK_AUTOTUNE T=1 DRY_RUN=1                      # detection and plan, shows the moves, nothing moves
DOCK_AUTOTUNE T=1                                # seat test of T1
DOCK_AUTOTUNE TOOLS=0,2 PREPARE=1                # several tools (0-4, all), prepare first
DOCK_AUTOTUNE T=1 MODE=scan AXES=z RANGE=0.4     # scan, first run: one axis, small range, watch it
DOCK_AUTOTUNE_INTERRUPT                          # stops after the current lift or trial (M112 stops at once)
```

What the command does without further parameters is set in `dock_autotune.cfg` (`mode`, `axes`, `tools`, `prepare`, the
ranges and so on). A parameter of the call always wins.

**Preparation.** Docks have to be measured in the state they are used in: bed temperature, levelled gantry, and with a
touch homing the Z zero taken by the touch tool. Without `PREPARE` a prompt shows the detected probe and the steps;
**Prepare and start** (blue) runs them (bed temperature 70 °C by default, change it with the buttons or `BED=`),
**The machine is already prepared** (yellow) skips them. The steps follow a `PRINT_START`:

1. `INITIALIZE_TOOLCHANGER`, a tool on the shuttle goes back into its dock
2. `M190` (bed), `G28`, `QUAD_GANTRY_LEVEL` (or `Z_TILT_ADJUST`), `BED_MESH_CLEAR`
3. touch homing with `touch_tool` (T0 by default, picked up with `SELECT_TOOL`, so AFC is not involved): the nozzle heats
   to `touch_temp` (`M109`), `before_touch` runs (**put your brush macro there**), the temperature is waited for again,
   the touch command runs `touch_repeats` times (once by default), the nozzle is turned off and the tool goes back into
   its dock. Without a touch homing the Z zero is taken with `G28 Z` after the levelling.

Detected are Cartographer (`CARTOGRAPHER_TOUCH_HOME`), Beacon (`home_method: contact`, touch is `G28 Z`), eddy current
probes, plain probes and a Z endstop. If Z cannot be homed with an empty shuttle (probe on a tool, `probe:` endstop
without a probe, legacy `[scanner]`), the prompt says so and only offers to continue with a machine you prepared by
hand (`PREPARE=0`). After a preparation the machine stays prepared, so the next run can use `PREPARE=0`.

**Several tools.** `TOOLS=0,1,2`, `TOOLS=0-4` or `TOOLS=all` (every tool of the toolchanger). The preparation runs
once, then the tools follow one after the other, each starting from safe Y. If a tool stops (error,
`DOCK_AUTOTUNE_INTERRUPT`) the run ends there, the remaining tools are listed as "not run" and the shuttle stays where
it is.

**End of a run.** When everything is done the shuttle goes without a tool to the centre of the bed and Z 15: first
sideways at the height of the docks, then down. The centre comes from `mesh_min` / `mesh_max` of `[bed_mesh]` (else
from the axes); `end_x`, `end_y` and `end_z` in `dock_autotune.cfg` set another position, `end_move: False` leaves the
shuttle at safe Y. After an error or an interrupt it does not move on its own.

| Parameter | |
|---|---|
| `T=<n>` / `TOOLS=0,1,2` | one tool / several tools (`0-4`, `all`); without both a prompt asks |
| `MODE=seat\|scan` | what to do, default `seat` |
| `AXES=xyz` | axes of the scan |
| `STEP`, `RESOLUTION`, `RANGE`, `RANGE_X/Y/Z` | override the config (mm) |
| `REPEATS`, `SPEED` | repeats per position, factor on `params_path_speed` |
| `PREPARE=1\|0` | prepare first / the machine is prepared; no prompt |
| `BED`, `SOAK`, `TOUCH=0` | bed temperature (°C), extra wait (s), no touch homing |
| `DRY_RUN=1` | only print the plan and the moves |

## Seat test

```
DOCK_AUTOTUNE T=0
```

The moves are the G-code lines of the stock `pickup_gcode` of klipper-toolchanger-easy up to the `verify` waypoint:
`close_y`, X and Z to the start of `params_pickup_path`, Y into the dock, the path to the lowest point of the lift. Then
the lift to the `verify` waypoint (never higher: above it the tool would leave the dock), a pause of 0.3 s, the read of
the tool detection, and down again. Nothing is picked up: no `SELECT_TOOL`, the toolchanger keeps "no tool", no
`error_gcode`, no AFC event. `DRY_RUN=1` prints all lines; compare them with your `pickup_gcode`. The shuttle has to be
empty and not inside a dock; from anywhere else the entry starts with the move to `close_y`.

**Measurement.** The first lift runs at path speed (`speed_factor` of `params_path_speed` times the `f` of the lift) and
shows where the knocks are. All later lifts go at that speed up to `seat_zone` (1.5 mm) below the first knock and then
slowly through the contact (`seat_speed`, 2 mm/s) up to the `verify` waypoint. The knocks are found in the accelerometer
signal after a high-pass (`adxl_window`) that removes gravity and the smooth acceleration of the moves, and are timed
against the known move of the shuttle, so their height comes out in mm (about 0.01 mm). Positions: the park position,
then `seat_step` (0.25 mm) steps along X and along Y up to `seat_range` (1 mm) to both sides, `seat_repeats` (3) lifts
each. A side ends after `seat_fail_stop` positions in a row where the tool detection was off at the top.

**What the accelerometer sees.** The tools hang in their docks. If the position does not fit, the rounded pin tips lift
the tool a little and it falls into the shuttle as soon as it has enough play: that is the knock (about 8 g on a
StealthChanger, as strong at 2 mm/s as at 9 mm/s, because a fall does not depend on the speed of the shuttle), the tool
wobbles in the dock before, and it falls back into the dock when the shuttle goes down (`release`). The height of the
fall rises with the misalignment: the tool has to be lifted further. If the position is right, the pins just dive in,
the tool stays where it is and nothing is measured. So **quiet is good**.

**Result, per position** (medians over the repeats): the pin at the top, the number of knocks, the height and strength
of the strongest knock, the **energy of the whole hub** (every knock, also the small ones, the lift, the hold and the way
down), the **wobble** (slow sideways movement of the tool in the dock, x and y after a longer high-pass,
`wobble_window`), and the **score**, energy plus wobble. The score is the one number that says how much the tool moves.
The result file has more columns (lateral share, net velocity change) for a look at the raw numbers.

* **Quiet window (the suggestion).** For X and for Y the module takes the positions whose score is in the lowest
  quarter between the lowest and the highest, looks for a run of them with a louder position on **both** sides, and
  suggests its centre as the new `params_park_x/y` together with a ready `SET_TOOL_PARAMETER` line. Nothing is applied:
  try the value, then run the seat test again around it and check that the position is quiet and the window is
  centred. If the quiet range reaches the end of the measured range the window is *not closed* and no value is
  suggested (raise `seat_range`); if all positions are alike the message says so.
* **Z:** the strongest knock is where shuttle and tool touch or snap. The path continues up to the `verify` waypoint, so
  the distance between the two is how far the path goes on; the result file shows both.

If the slow lifts show no events, the knocks are below the threshold (6 times the noise, at least 0.5 g): lower
`seat_threshold` or raise `seat_speed`. The energy and the quiet window do not depend on that threshold.

## Scan

```
DOCK_AUTOTUNE T=1 MODE=scan AXES=z RANGE=0.4
```

1. A *trial* is a pickup (`SELECT_TOOL`, the tool detection at the `verify` waypoint must see the tool) and a dropoff
   (`UNSELECT_TOOL`), `repeats` times. All of them have to lock.
2. Starting at the current value (it has to lock, otherwise the scan stops) the offset moves outwards in `step`
   increments on both sides until a trial fails. Then the interval at the edge is halved down to `resolution`.
3. The centre of the window is tried again and, if it locks, applied. Axes are done one after the other.

The scan runs in the coordinate frame of a real tool change, so the values are valid for printing. If no trial fails on
a side within `range_*`, there is no edge: the window is *not closed* and that axis is left alone. Run it again with a
larger range and step, for example `RANGE=1.2 STEP=0.2`. A failing trial prints a red `Expected tool ... but active is
None`: that is expected, it is how the edge is found. A scan takes at most `2 * range / step` trials per axis plus the
edge refinement (`DRY_RUN=1` prints the number), each with `repeats` pickups.

The new values apply until the next restart. Copy the `params_park_x: ...` lines from the console or the result file
into `toolchanger/tools/T<n>.cfg`.

## Result files

Every run that took measurements writes one text file to `printer_data/config/dock_autotune/results/` (in Mainsail:
*Machine*, folder `dock_autotune/results`):

* `T0_seat_20261031_194500.txt`, `T0_scan_….txt` for one tool,
* `tools_seat_….txt`, `tools_scan_….txt` for several: how each tool ended, the park positions of all tools (before,
  after) with the lines for the tool files, and below that the full report of every tool.

A report holds the status (finished, or stopped and why), the settings, the **park positions** (before, after and what
happened to each axis), the measurements (seat: height of the lift and a table per axis; scan: the trials in the order
they were made) and all numbers as CSV. A run that is stopped writes the file too, with what was measured until then.
`results_dir:` in `dock_autotune.cfg` changes the folder, an empty value switches the file off. The folder is part of
the config directory, so a backup of that directory contains the results as well.

`~/printer_data/logs/dock_autotune.jsonl` has one line per scan trial and per seat position. `printer.dock_autotune` has
`active`, `tool`, `axis`, `phase`, `trial` and `result`.

## Macro list

Mainsail, Fluidd and touch UIs list only `[gcode_macro]` sections, not commands that a module registers.
`dock_autotune_macro.cfg` defines `[gcode_macro DOCK_AUTOTUNE]`, which calls the module command `DOCK_AUTOTUNE_RUN` with
all parameters. The module notices that macro and does not register the name itself. Do not add `rename_existing` to
it: Mainsail leaves such macros out of its list. Add the macro to a group under *Interface Settings → Macros*.

## Safety

* **Be at the machine** for the first runs and keep `M112` at hand.
* The seat test moves the shuttle with plain `G0` lines, not with the toolchanger commands. Read the output of
  `DRY_RUN=1` once. It never lifts higher than the `verify` waypoint, and `DOCK_AUTOTUNE_INTERRUPT` takes the shuttle
  out of the dock before it stops.
* The scan: a failing pickup next to the window is harmless, a position far outside of it can hit the dock, so the
  default ranges are small. Speed and acceleration are reduced while it runs and restored afterwards, also after an
  error, and the park values go back to the old ones if it stops or fails. After a failed pickup the toolchanger is
  initialized again and checked: shuttle empty, no tool detected, shuttle at `params_safe_y`. Anything else stops the
  scan. The scan uses `SELECT_TOOL`/`UNSELECT_TOOL`, not the `T<n>` commands: AFC is not asked to load anything but sees
  the extruder activations. Check that AFC and the toolchanger agree about the tool before the next print.

## Troubleshooting

| Problem | Check |
|---|---|
| `DOCK_AUTOTUNE` missing in the macro list | `dock_autotune/dock_autotune_macro.cfg` included? Reload the page after a restart |
| `TOOLS=...` is refused with "T=N is required" | old macro file: `git pull && ./install.sh` |
| Klipper stops: `Existing command 'DOCK_AUTOTUNE' not found` | very old macro file with `rename_existing`: `git pull && ./install.sh` |
| No `[update_manager]` entry in Mainsail | restart Moonraker; the clone has to be a Git repository of the printer user; see `moonraker.log` |
| "MODE=seat needs the accelerometer [adxl345 T1]" | name the chips per tool, or set `accelerometer:` in `dock_autotune.cfg` |
| "no knock in the lift into the shuttle" | tool not in its dock, accelerometer not working, or `seat_threshold` too high |
| "the shuttle is at Y=…, inside of the dock" | move the shuttle out first, for example `G0 Y100` |
| Scan stops: "does not know the scan mode" | the `error_gcode` pauses: use the handler from `examples/` |
| Touch homing starts at the height of the dock | `touch_restore_axis: Z` (default) brings Z back after the pickup |

## Changes

What changed and why is in [CHANGELOG.md](CHANGELOG.md).

## Tests

```
python3 -m unittest discover -s tests
```

## License

GPLv3, see [LICENSE](LICENSE). Use at your own risk: this software moves a toolhead into a dock.
