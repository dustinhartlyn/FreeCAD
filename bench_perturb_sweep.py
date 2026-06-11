#!/usr/bin/env python3
"""
Stratified Perturbation Benchmark: FullPivLU vs SparseLDLT
Sweeps: node counts (50, 100, 150, 200, 250) x perturbation (5%, 10%, 15%)
Each node = 2 DOF => variable counts: 100, 200, 300, 400, 500

Set GCS_SPARSE_LDLT=1 for SparseLDLT; unset for FullPivLU.
Perturbation is applied as a ±pct shift on the driving DistanceX datum,
forcing the solver to reconverge from a known violation magnitude.
"""
import os, time, sys
import FreeCAD as App
import Part
import Sketcher

# Silence C++ stderr diagnostics per context-diet policy
sys.stderr = open(os.devnull, 'w')

MODE = "SparseLDLT" if os.getenv("GCS_SPARSE_LDLT") else "FullPivLU"
NODE_COUNTS = [50, 100, 150, 200, 250]
PERTURBATION_PCTS = [5, 10, 15]
N_WARMUP = 5
N_MEASURED = 30
BASE_LEN = 10.0  # mm per segment

print(f"[BENCH_START] mode={MODE} node_counts={NODE_COUNTS} perturbations={PERTURBATION_PCTS}")
print(f"[BENCH_START] warmup={N_WARMUP} measured={N_MEASURED}")

for num_nodes in NODE_COUNTS:
    doc = App.newDocument(f"B_{num_nodes}")
    sketch = doc.addObject("Sketcher::SketchObject", "Sketch")

    # Build chain: N line segments end-to-end with coincident + equal constraints
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

    # Driving DistanceX between first start (0,1) and last end (N-1,2)
    nominal = num_nodes * BASE_LEN
    drv = sketch.addConstraint(
        Sketcher.Constraint("DistanceX", 0, 1, num_nodes - 1, 2, nominal)
    )

    # Initial solve to establish equilibrium
    doc.recompute()

    for pct in PERTURBATION_PCTS:
        delta = nominal * (pct / 100.0)

        # Warmup: alternate +pct / -pct
        for w in range(N_WARMUP):
            sign = 1.0 if (w % 2 == 0) else -1.0
            sketch.setDatum(drv, App.Units.Quantity(f"{nominal + sign * delta} mm"))
            doc.recompute()

        # Measured: alternate +pct / -pct to defeat caching
        t0 = time.perf_counter()
        for m in range(N_MEASURED):
            sign = 1.0 if (m % 2 == 0) else -1.0
            sketch.setDatum(drv, App.Units.Quantity(f"{nominal + sign * delta} mm"))
            doc.recompute()
        t1 = time.perf_counter()

        avg_ms = (t1 - t0) / N_MEASURED * 1000.0
        total_s = t1 - t0
        n_vars = num_nodes * 2

        print(
            f"[ROW] {MODE}\t{n_vars}\t{pct}\t{avg_ms:.3f}\t{total_s:.4f}"
        )

    App.closeDocument(doc.Name)

print(f"[BENCH_DONE] mode={MODE}")
