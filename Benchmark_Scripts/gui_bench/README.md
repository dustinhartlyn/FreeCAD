# GUI Sketcher responsiveness benchmark (A/B: your build vs FreeCAD 1.1)

Measures how responsive dense-sketch editing feels **in the real GUI**, comparing
your pixi build against a stock FreeCAD 1.1 install — through the full interactive
path the headless benches never touch.

Why a GUI bench: the headless benchmarks timed only the constraint *solve* /
`doc.recompute()`. In the GUI every edit also fires
`ViewProviderSketch::draw()` — a Coin scene-graph rebuild of every edge, vertex,
and constraint icon — plus a GL repaint. For dense sketches that redraw is a big
part of the felt latency, and it sits on top of the solve. This harness exercises
solve **+ draw() rebuild** end-to-end.

## Files
- `gui_bench.py` — runs *inside* a FreeCAD GUI process; builds parametrized dense
  sketches, enters Sketcher edit mode, times five scenarios, writes JSON.
- `run_gui_bench.ps1` — launches `gui_bench.py` in each build, collects the JSON,
  prints an A/B table, writes `results/ab_summary.csv`.
- `results/` — `dev.json`, `stock.json`, `ab_summary.csv`.

## Run it
```powershell
# from the repo root, Windows PowerShell (5.1 is fine)
.\Benchmark_Scripts\gui_bench\run_gui_bench.ps1
.\Benchmark_Scripts\gui_bench\run_gui_bench.ps1 -Sizes 50,100,200,300
.\Benchmark_Scripts\gui_bench\run_gui_bench.ps1 -Scenarios add_dimension,drag -Builds dev
.\Benchmark_Scripts\gui_bench\run_gui_bench.ps1 -DevConfig release      # fair absolute A/B (see caveats)
```
`-Sizes` is rectangles (rect scenarios) / chain segments (drag). `-Builds dev,stock`.

> **Before running:** close any FreeCAD GUI you have open. A running GUI locks the
> pyds (the dev-pyd copy fails silently → you'd bench stale code) and stray
> processes get force-killed between runs. **Don't touch the mouse during the drag
> scenario** — it warps the cursor for the grab-check.

## Scenarios
| scenario | sketch | what it exercises |
|---|---|---|
| `enter_edit` | rect array | one-time edit-mode scene-graph build (pure GUI, scales with N) |
| `add_dimension` | rect array | add+remove a `Distance` → full `diagnose()` + redraw each time (the "dimension-add stall") |
| `datum_edit` | rect array | change an existing datum value → solve + island rebuild + redraw |
| `recompute` | rect array | `touch()` + `doc.recompute()` + redraw |
| `drag` | floppy chain | sweep the free tip → per-frame `moveGeometries` solve + `draw()` rebuild |

## How each build is launched
- **dev**: `pixi run <.pixi env FreeCAD.exe> gui_bench.py`. `pixi run` activates the
  conda/pixi env (PATH to its DLLs) — launching the bare exe fails to start.
  The freshly built `Sketcher.pyd` + `SketcherGui.pyd` are copied into
  `.pixi/envs/default/Library/Mod/Sketcher/` first (install-debug does **not**
  refresh those). Provenance in the output prints the loaded `Sketcher.__file__`
  and version so you can confirm current code ran.
- **stock**: `"C:\Program Files\FreeCAD 1.1\bin\freecad.exe" gui_bench.py`
  (self-contained; launched directly).

`gui_bench.py` defers its work to a `QTimer` (so the Qt event loop is live), arms a
watchdog that force-exits if anything hangs, writes results JSON, and hard-exits so
the harness proceeds. The harness polls for the JSON file (the pixi FreeCAD.exe is a
thin launcher that can exit before the child GUI finishes).

## Drag methodology (important)
A real drag frame = `moveGeometries()` solve + `draw(true,false)` rebuild + GL repaint.
Two deliberate choices:

1. **The frame is driven by `sk.moveGeometry(...)`, not synthetic mouse-move events.**
   QTest/Qt mouse *press/click* events do drive Coin's pick pipeline (we use one to
   validate the vertex is grabbable — the `grabbable`/`grab_subs` fields), but
   synthetic mouse-*move* events are **not** delivered to the SoQt/Quarter drag
   handler from Python, so they can't drive a live drag. `moveGeometry()` runs the
   *identical* `moveGeometries` solve the interactive drag uses, and `Gui.updateGui()`
   fires the same `ViewProviderSketch::draw()` rebuild — so the measured work equals a
   real drag frame's CPU work. (`grab-check n/a` on a build just means the synthetic
   click didn't register there; the timing is still valid — check `measurement_ok`,
   which confirms the tip actually swept.)
2. **We don't force a synchronous GL repaint inside the timing.** That blocks on
   vsync (~16/33 ms buckets) and would swamp the solver+rebuild signal with GPU noise.
   The N-scaling scene-graph rebuild is captured; the near-constant GL blit is not.

Headline `ms` per scenario is the **median** frame (robust to warm-up/vsync outliers);
`min_ms`/`max_ms`/`ms_each` are in the JSON.

## Caveats when reading the numbers
- **dev is a *debug* build** by default (`-DevConfig debug`); stock 1.1 is release. A
  debug build is normally *slower*, so any speedup understates the real algorithmic
  win. For a fair absolute comparison run `pixi run install-release` then
  `-DevConfig release`.
- The two are different FreeCAD builds/compilers, so absolute ms aren't perfectly
  apples-to-apples; the **relative speedup and the scaling with size** are the signal.
- `enter_edit` is essentially unchanged Gui code → expect it near parity; the solver /
  `diagnose()` / rebuild scenarios are where your commits show.

## Example result (sizes 40, 100 — debug dev vs release stock)
```
scenario         size   stock-1.1         dev   speedup
enter_edit         40     1,522.6     1,365.9     1.11x
enter_edit        100     2,403.4     1,702.1     1.41x
add_dimension      40       683.8       155.6     4.40x
add_dimension     100     1,853.1       596.2     3.11x
datum_edit         40       814.5       195.5     4.17x
datum_edit        100     1,657.9       497.5     3.33x
recompute          40       594.0       120.1     4.94x
recompute         100     1,266.3       289.4     4.37x
drag               40        24.7         5.6     4.41x
drag              100        25.0         7.4     3.38x
```
Reads exactly as expected: `enter_edit` (pure GUI scene build, untouched code) barely
moves; the solve / `diagnose()` / island-rebuild scenarios are 3–5× faster **through
the full GUI redraw path** — and that's a debug build beating release.
```
```
