# Sketcher performance benchmark scripts

The scripts used to measure the Sketcher responsiveness work (see the PR
description for the results table). They are kept on the
`planegcs-solver-instrumented` branch, alongside the env-gated profilers and
differential check modes they pair with; the PR branch ships without any
instrumentation.

Everything here runs against a built FreeCAD from this branch. The headless
scripts need `FreeCADCmd`; the GUI harness drives the real `FreeCAD` GUI and can
A/B it against a stock install.

## Headless (App + solver) benchmarks — run with `FreeCADCmd`

```
FreeCADCmd Benchmark_Scripts/bench_dense.py
```

| script | what it measures |
|---|---|
| `bench_dense.py` | recompute of a dense per-segment `Distance` chain (100–500 nodes), banded Jacobian; the core solver workload |
| `bench_solver.py` | force-recompute timing, toggling a constraint value each iteration to trigger a real re-solve |
| `bench_solve_vs_recompute.py` | splits `sketch.solve()` (constraint solver) from `doc.recompute()` (solver + shape rebuild + dependency graph) — shows which layer users actually feel |
| `bench_rectangles.py` | reproduces the "adding a dimension stalls" case on a dense rectangle array (topology change → full `diagnose()`) |
| `bench_islands.py` | many disconnected sub-sketches, to exercise the per-component solve |
| `bench_conic.py` | four `errorgrad()` conic constraint types, each in its own subsystem |
| `drag_snap_repro.py` | headless reproduction of the drag orchestration end-to-end; checks a dragged vertex does not snap to the origin |
| `probe300.py`, `probe_recompute.py` | small ad-hoc timing probes (300-element sketch; 500-node recompute) |

### Environment-variable note

`bench_dense.py` and `bench_solver.py` mention a `GCS_SPARSE_LDLT=1` toggle. That
was a development-time A/B switch between the old dense `FullPivLU` Gauss-Newton
step and the new sparse least-norm step. **It no longer exists in the code** —
`SparseLDLT` is now the hardcoded default (`System::dogLegGaussStep`). To compare
against the pre-optimization solver, benchmark a stock FreeCAD 1.1 build (as the
"before" column of the results table does) rather than expecting that variable to
switch paths.

The per-stage profilers this branch carries (`GCS_DIAGPROF`, `SKETCH_EXECPROF`,
and the `GCS_DIAG_SELFCHECK` / `SKETCH_SHAPECACHE_CHECK` differential checks) are
read from the environment by the C++ code; set them before launching
`FreeCADCmd` to get stderr timing lines / mismatch warnings.

## GUI A/B benchmark — `gui_bench/`

Measures felt responsiveness through the full interactive path (solve +
`ViewProviderSketch::draw()` scene-graph rebuild + GL repaint), comparing this
build against a stock FreeCAD 1.1 install. See `gui_bench/README.md` for the
scenarios, flags, and caveats.

```powershell
# Windows PowerShell, from the repo root
.\Benchmark_Scripts\gui_bench\run_gui_bench.ps1 -Sizes 50,100,200,300
```

`gui_bench/results/` holds the captured `dev.json` / `stock.json` and the
`ab_summary.csv` the results table was built from.
