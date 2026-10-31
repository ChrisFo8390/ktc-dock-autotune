# Changelog

The project is a prototype; the entries explain what changed and why, newest first.

## 2026-10-31 (night): end position

* After a successful run (one tool or several) the shuttle goes without a tool to the centre of the bed and Z 15 (`end_z`)
  instead of staying at safe Y: sideways at the height of the docks first, then down. The centre comes from
  `[bed_mesh]` (`mesh_min` / `mesh_max`), else from the axes; `end_x`, `end_y` override it, `end_move: False` turns it
  off. After an error or an interrupt the shuttle stays where it is. This replaces the return to where the toolhead stood
  at the start, which only existed for several tools.

## 2026-10-31 (evening): the quiet window replaces sign changes and lateral share

**What the first seat test showed** (T0, bed 70 °C, X and Y ±1 mm in 0.25 mm steps, 3 lifts each, 52 lifts):

* The tool detection was on at the top of every lift, as in every earlier scan: the rounded pins find their way in
  from almost anywhere, so "locked" says nothing about how cleanly the tool is taken up.
* In Y the lift was calm for every offset from -0.25 to -1.0 mm (no knock above 0.5 g, release knock 0.7 to 1.3 g,
  which is the noise floor) and loud from 0 to +1.0 mm: a knock of 8.3, 7.6, 6.0, 3/11 and 5.2 g, the release knock
  rising from 2.4 to 6.5 g, and the height of the main knock rising from Z 260.99 to 263.79 with +Y.
* In X the lifts were loud at -0.25 and 0 (8.9 and 8.3 g) and at +1.0 (4.7 g), calm from +0.25 to +0.75 and from -1.0
  to -0.5.
* The main knock was as strong at 2 mm/s as at 8.75 mm/s and sat about 5.1 mm below the `verify` waypoint.
* The lateral share and the sign of the net velocity change, which the first version of the seat test looked at,
  jumped between positions and said nothing.

**Why** (from the way the dock works): the tools hang in their docks. If the position does not fit, the rounded pin tips
lift the tool and it falls into the shuttle as soon as it has enough play. That fall is the knock; it does not depend on
the speed of the shuttle, and the height of the fall grows with the misalignment because the tool has to be lifted
further. The tool also wobbles in the dock before, and falls back into it when the shuttle goes down (the release
knock). If the position is right the pins just dive in and the tool does not move. Quiet is good.

**What changed**

* Every hub now gets a **score**: the energy of the whole hub (lift, hold and way down, every knock, also the small ones
  below the threshold) plus the **wobble**, the slow sideways movement of the tool in the dock (x and y after a longer
  high-pass, `wobble_window`, 0.2 s).
* Per axis the module finds the **quiet window**: the positions whose score is in the lowest quarter between the lowest
  and the highest, as a run with a louder position on both sides. Its centre is suggested as the new park position,
  with a ready `SET_TOOL_PARAMETER` line. A window that reaches the end of the measured range is not closed: no value
  is suggested and the message asks for a larger `seat_range`. Nothing is applied.
* The tables show `z_main` (height of the strongest knock), `peak`, `energy`, `wobble`, `score` and `release`. Lateral
  share and net velocity change stay in the CSV block of the result file.
* The files of several tools list the suggestions of all tools.

**Not verified on the machine yet:** the score, the choice of the lowest quarter and the order of the positions
(0, +step, -step, ...). If the tool rests differently in the dock after every lift, repeated measurements at the same
position will differ; that shows only in a second run.

## 2026-10-31: seat test, several tools, prompts

* `MODE=seat` (now the default): the empty shuttle lifts into the tool in its dock up to the `verify` waypoint and
  down again. The tool never leaves its dock, `SELECT_TOOL` is not used. The accelerometer of the tool measures where
  shuttle and tool touch. Reason: a scan over full pickups found no edge of the window (35 of 35 trials locked at X
  ±0.6, Y ±0.5, Z ±0.5 mm), and a map of the knock of full pickups was flat (39 of 39 locked at Z ±1.5 mm, the knock was
  13 g and 20 ms at every offset), because the pins catch every misalignment.
* `TOOLS=0,1,2 | 0-4 | all` runs several tools with one preparation, returns the shuttle to where the toolhead stood,
  and writes one result file with the park positions of all tools.
* A click on the macro without `T=`/`TOOLS=` opens a prompt that asks for the tool; the buttons of the preparation prompt
  close it (blue main button, yellow second button).
* The touch preparation heats up and waits first (`M109`), runs `before_touch`, waits again and touches once.
* Files moved into `config/dock_autotune/` (settings, macro, `results/`); the installer migrates older installs.
* Result files per run (`T0_seat_<time>.txt`, `tools_<mode>_<time>.txt`) with the park positions before and after.
* **Removed:** the ADXL map mode (`MODE=map`, `RAW`, `map_*`) and the accelerometer option of the scan (`ADXL=1`,
  `abort_g`, `peak_g`). The last commit with them is 363ce40.

## Before

* `MODE=scan`: window search over full pickups and dropoffs with the tool detection pin, preparation like a
  `PRINT_START` (probe detection, bed, homing, levelling, touch homing), Mainsail prompt, installer, macro for the macro
  list, update manager entry.
