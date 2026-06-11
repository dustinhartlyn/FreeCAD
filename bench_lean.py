#!/usr/bin/env python3
"""
Lean stratified perturbation benchmark.
Sweeps: node counts × perturbation (5%, 10%, 15%).
Outputs [ROW] lines readable as TSV. Exit 0 on completion.

Usage:
  set GCS_SPARSE_LDLT=   & set GCS_MAX_ITERATIONS=20 & build\debug\bin\FreeCADCmd.exe --console bench_lean.py > results_fullpivlu.tsv
  set GCS_SPARSE_LDLT=1  & set GCS_MAX_ITERATIONS=20 & build\debug\bin\FreeCADCmd.exe --console bench_lean.py > results_sparseldlt.tsv
"""
import os, time, sys
import FreeCAD as App
import Part
import Sketcher

sys.stderr = open(os.devnull, 'w')

MODE = "SparseLDLT" if os.getenv("GCS_SPARSE_LDLT") else "FullPivLU"
os.environ["GCS_MAX_ITERATIONS"] = "20"
NODE_COUNTS = [50, 100, 150, 200, 250]
PERTURBATION_PCTS = [5, 10, 15]
N_WARMUP = 3
N_MEASURED = 8
BASE_LEN = 10.0

# Header
print(f"mode\tn_vars\tn_nodes\tpct_pct\tavg_ms\ttotal_s\twarmup_n\tmeasured_n")

for num_nodes in NODE_COUNTS:
    n_vars = num_nodes * 2
    doc = None
    try:
        doc = App.newDocument(f"B{num_nodes}")
        sketch = doc.addObject("Sketcher::SketchObject", "Sketch")

        # Build: N line segments end-to-end
        sketch.addGeometry(
            Part.LineSegment(App.Vector(0, 0, 0), App.Vector(BASE_LEN, BASE_LEN, 0)), False
        )
        sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 1, -1, 1))
        for i in range(1, num_nodes):
            sketch.addGeometry(
                Part.LineSegment(
                    App.Vector(i * BASE_LEN, 0, 0),
                    App.Vector((i + 1) * BASE_LEN, BASE_LEN, 0),
                ),
                False,
            )
            sketch.addConstraint(Sketcher.Constraint("Coincident", i - 1, 2, i, 1))
            sketch.addConstraint(Sketcher.Constraint("Equal", i - 1, i))

        nominal = num_nodes * BASE_LEN
        drv = sketch.addConstraint(
            Sketcher.Constraint("DistanceX", 0, 1, num_nodes - 1, 2, nominal)
        )

        doc.recompute()  # initial solve

        for pct in PERTURBATION_PCTS:
            try:
                delta = nominal * (pct / 100.0)
                # Warmup
                for w in range(N_WARMUP):
                    sign = 1.0 if (w % 2 == 0) else -1.0
                    sketch.setDatum(drv, App.Units.Quantity(f"{nominal + sign * delta} mm"))
                    doc.recompute()
                # Measured
                t0 = time.perf_counter()
                for m in range(N_MEASURED):
                    sign = 1.0 if (m % 2 == 0) else -1.0
                    sketch.setDatum(drv, App.Units.Quantity(f"{nominal + sign * delta} mm"))
                    doc.recompute()
                t1 = time.perf_counter()
                avg_ms = (t1 - t0) / N_MEASURED * 1000.0
                print(f"{MODE}\t{n_vars}\t{num_nodes}\t{pct}\t{avg_ms:.3f}\t{t1 - t0:.4f}\t{N_WARMUP}\t{N_MEASURED}")
                sys.stdout.flush()
            except Exception:
                print(f"{MODE}\t{n_vars}\t{num_nodes}\t{pct}\tERR\t-1\t{N_WARMUP}\t{N_MEASURED}")
                sys.stdout.flush()
    except Exception:
        for pct in PERTURBATION_PCTS:
            print(f"{MODE}\t{n_vars}\t{num_nodes}\t{pct}\tERR\t-1\t{N_WARMUP}\t{N_MEASURED}")
            sys.stdout.flush()
    finally:
        if doc is not None:
            try:
                App.closeDocument(doc.Name)
            except Exception:
                pass
