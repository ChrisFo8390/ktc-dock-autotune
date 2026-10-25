# ktc-dock-autotune

Finds the best park position of a [klipper-toolchanger](https://github.com/viesturz/klipper-toolchanger)
(KTC) dock automatically.

**Status: prototype. Tested against the real `toolchanger.py` in a simulation with a modelled
dock, and with unit tests. Not yet run on a real printer.** Read "Safety" before the first run.

## What it does

A tool locks into the shuttle only inside a small window of positions. `DOCK_AUTOTUNE` finds that
window per axis and sets `params_park_x/y/z` of the tool to its centre:

1. One *trial* is a pickup (`SELECT_TOOL`, the verify at the lock position of the pickup path must see
   the tool) followed by the dropoff (`UNSELECT_TOOL`), `repeats` times. All of them have to lock.
2. Starting at the current value (it must lock, otherwise the scan stops) the offset is moved outwards
   in `step` increments on both sides until a trial fails (`range_*` is the limit). One extra trial
   halves the step at the failing edge.
3. The centre of the window is tried again and, if it locks, applied. The axes are done one after the
   other (x, y, z). If a side did not fail within the range the window is "not closed" and that axis
   is left alone.

It tunes in the same coordinate frame as a real tool change (the toolchanger drops the tool offsets
during a change), so the values are valid for production. The measuring signal is the tool detection
pin, the same check KTC does at the lock position. An accelerometer on the tool can be recorded
alongside and can stop the scan (`abort_g`), but it never decides.

## Requirements

* KTC with tool detection pins (`detection_pin` in every `[tool]`), `require_tool_present: False` is fine.
* A toolchanger `error_gcode` that **leaves the dock** after a failed pickup and that does not pause while
  a scan runs (`printer.dock_autotune.active`). With the stock `error_gcode: PAUSE` a failing trial
  pauses the print; the module notices that and stops with a message. If the handler does not get the
  shuttle out of the dock the scan stops as well (it checks that the shuttle is back at `params_safe_y`).
  [`examples/toolchanger-error-handler.cfg`](examples/toolchanger-error-handler.cfg) is a handler that does
  this (it needs the Mainsail/Fluidd client macros `PAUSE` and `_CLIENT_VARIABLE`).
* Not usable while a print is running or paused. All axes homed, or let the preparation do it (see "Use").
* The prompt needs a frontend that understands `// action:prompt_*` (Mainsail, Fluidd, KlipperScreen).
  Without one, call `DOCK_AUTOTUNE` with `PREPARE=1` or `PREPARE=0` and no prompt is needed.

## Install

```
git clone https://github.com/ChrisFo8390/ktc-dock-autotune.git ~/ktc-dock-autotune
~/ktc-dock-autotune/install.sh
```

The installer does everything in one go:

* links `klipper/extras/dock_autotune.py` into Klipper and copies `config/dock_autotune.cfg` to your config
  directory (an existing file is never overwritten),
* adds `[include dock_autotune.cfg]` to `printer.cfg` (above the `SAVE_CONFIG` block, line endings kept),
* adds the `[update_manager ktc-dock-autotune]` entry to `moonraker.conf`,
* restarts Klipper, and Moonraker if it changed `moonraker.conf` (Moonraker reads its config only at start,
  without that restart the entry does not show up in Mainsail). `sudo` must work without a password,
  otherwise the script prints the command to run.

Backups (`printer.cfg.bak.<date>`, `moonraker.conf.bak.<date>`) are made before a file is changed. Running it
again changes nothing that is already there.

| Option | |
|---|---|
| `--no-include` | do not touch `printer.cfg` (add `[include dock_autotune.cfg]` yourself) |
| `--no-update-manager` | do not touch `moonraker.conf` |
| `--no-restart` | do not restart anything |
| `--uninstall` | remove the link and the include line (`dock_autotune.cfg` and the update manager entry stay) |
| `KLIPPER_PATH=…`, `CONFIG_PATH=…` | other locations than `~/klipper` and `~/printer_data/config` |

Update with `git pull` in the clone (or the update manager) and restart Klipper.

The update manager needs the clone (`path:` in the entry) to be a Git repository of the printer user with the
`origin` from the entry. A module copied by hand has no entry. Moonraker's reasons for a missing or invalid
entry are in `~/printer_data/logs/moonraker.log` (search for `update_manager`).

Without Git: copy `klipper/extras/dock_autotune.py` to `~/klipper/klippy/extras/` and `config/dock_autotune.cfg`
to the config directory by hand.

## Use

The docks have to be tuned in the state they are used in: same bed temperature, gantry levelled, and
(with a touch homing) the Z zero taken by the touch tool, because that zero depends on the bed and on the tool.
`DOCK_AUTOTUNE` therefore starts with a **preparation** that does what your `PRINT_START` does before the tools
are prepared. It works from any state of the machine: homed or not, cold or hot, a tool on the shuttle or not.

```
DOCK_AUTOTUNE T=1 DRY_RUN=1                 # only prints the detection and the plan
DOCK_AUTOTUNE T=1 AXES=z RANGE=0.4          # first run: one axis, small range, watch it
DOCK_AUTOTUNE T=1                           # x, y, z with the configured ranges
DOCK_AUTOTUNE_INTERRUPT                     # stops after the current trial (M112 stops at once)
```

Without `PREPARE` the command shows a prompt and does nothing else:

* It names the detected probe (Cartographer, Beacon, eddy current, plain probe, or none: Z endstop) and
  whether a touch homing is used (`CARTOGRAPHER_TOUCH_HOME`; Beacon with `home_method: contact`, where
  `G28 Z` touches), and lists the steps.
* **Prepare and start** runs them with the bed temperature shown (default `bed_temp: 70`, changed with the
  -10 / -5 / +5 / +10 buttons or `BED=`), then the scan starts.
* **The machine is already prepared** skips the preparation (like `PREPARE=0`).
* Preparation steps: `INITIALIZE_TOOLCHANGER`; a tool on the shuttle is put back into its dock (with a `G28`
  first if nothing is homed); `M190` to the bed temperature (`SOAK=<s>` waits longer); `G28`; `QUAD_GANTRY_LEVEL`
  (or `Z_TILT_ADJUST`); `BED_MESH_CLEAR`; the touch homing with `touch_tool` (default T0, **only that tool**,
  it is picked up with `SELECT_TOOL`, not with `T0`, so AFC is not asked to load anything): nozzle to
  `touch_temp`, `before_touch` (for example `BRUSH`), `touch_repeats` times the touch command, nozzle off,
  tool back into its dock. Without a touch homing the Z zero is taken again with `G28 Z` after the levelling.
  A failure in any step stops everything with a report.
* If Z cannot be homed with an empty shuttle (the probe sits on a tool: `tool_probe`; or `probe:` is the Z
  endstop but no probe section exists; or the legacy `[scanner]` module) the prompt says so and offers only
  "The machine is prepared - continue" or "Close". `PREPARE=1` is refused with the same message. Prepare the
  machine by hand and use `PREPARE=0`.

`PREPARE=0` expects the state of a print start: homed, levelled, Z reference from the same homing as in
production, bed hot, shuttle empty or the tool of the scan on it. `TOUCH=0` leaves out the touch homing.

At the end the new values are applied until the next restart and printed as `params_park_x: ...`
lines. Copy them into `toolchanger/tools/T<n>.cfg`. (`SAVE_TOOL_PARAMETER` plus `SAVE_CONFIG` works too,
but the line has to be commented out in the cfg first, otherwise Klipper refuses the config.)

Every failing trial prints a red `Expected tool ... but active is None` in the console. That is expected,
it is how the edge of the window is found.

| Parameter | |
|---|---|
| `T=<n>` | tool, required |
| `AXES=xyz` | axes to tune, default all |
| `STEP=0.1`, `RANGE=<mm>`, `RANGE_X/Y/Z=<mm>` | override `step` and `range_*` |
| `REPEATS=2` | pickups per candidate |
| `SPEED=0.5` | factor on `params_path_speed` while scanning |
| `ADXL=1` | record the accelerometer of the tool (`[adxl345 T<n>]`) |
| `PREPARE=1` / `PREPARE=0` | prepare like a print start, then scan / the machine is prepared; no prompt |
| `BED=<°C>`, `SOAK=<s>` | bed temperature of the preparation (default `bed_temp`), extra wait |
| `TOUCH=0` | no touch homing in the preparation |
| `DRY_RUN=1` | print the plan, move nothing |

Does the dock position depend on the bed temperature? Scan the same dock twice, for example `BED=60` and
`BED=100`, and compare the `params_park_z` lines. If they differ by more than a fraction of the window, tune at the
temperature you print at most and keep in mind that the other prints sit closer to the edge.

A scan takes at most `2 * range / step + 4` trials per axis (`DRY_RUN=1` prints the number), each
`repeats` pickups. Estimate, not measured on a printer: 15 to 25 s per pickup at half speed, so
plan 15 to 30 minutes per dock.

## Safety

* **Be at the machine** for the first runs and keep `M112` at hand. A failing pickup next to the
  window is harmless, a position far outside of it can hit the dock. That is why the ranges are small
  and why the scan stops when the baseline (the current value) does not lock.
* Speed and acceleration are reduced during the scan and restored afterwards, also after an error.
  The park values go back to the old ones if the scan is stopped or fails.
* After a failed pickup the toolchanger is initialized again and checked: shuttle empty, no tool
  detected, shuttle at `params_safe_y`. Anything else stops the scan.
* The scan uses `SELECT_TOOL`/`UNSELECT_TOOL` directly, not the `T<n>` commands. AFC is not asked to load
  or unload anything, but it sees the extruder activations and may take the scanned tool as its current
  lane. Check the AFC state (and that the toolchanger and AFC agree about the tool) before the next print.
* Tune with the mesh out of the way: set `fade_end` in `[bed_mesh]` (for example `fade_start: 1`,
  `fade_end: 10`, `fade_target: 0`), otherwise the mesh changes the dock height by its edge value.

## Accelerometer (optional)

`ADXL=1` records the accelerometer of the tool during every pickup/dropoff and writes `peak_g`
(largest deviation from the median, in g) to the log. The sections must be named per tool,
`[adxl345 T0]` ... `[adxl345 T4]`; an unnamed `[adxl345]` in every tool cfg is merged by Klipper into one
chip, the last one wins. `abort_g` stops the scan when a trial shows a shock above that value. Use it as a
guard, not as a measure of quality: how well the shock separates a good from a bad position has to
be seen on the machine first.

## Files written

`~/printer_data/logs/dock_autotune.jsonl`: one line per trial (`tool`, `axis`, `value`, `ok`, `peak_g`,
`error`), a summary at the end of a scan, a line if it was stopped. `printer.dock_autotune` (Mainsail
console, macros): `active`, `tool`, `axis`, `phase`, `trial`, `result`.

## Tests

```
python3 -m unittest discover -s tests
```

## License

GPLv3, see [LICENSE](LICENSE). Use at your own risk: this software moves a toolhead into a dock.
