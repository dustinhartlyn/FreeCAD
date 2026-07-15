#!/usr/bin/env python3
"""Dense geometry benchmark: local per-segment Distance constraints for 100-500 node testing.
Uses banded Jacobian topology (no long-range coupling) to avoid divergence at high node counts.
Set GCS_SPARSE_LDLT=1 for SparseLDLT; unset for FullPivLU.
Triggers re-solve by toggling first segment's distance value each iteration."""
import os, time
import FreeCAD as App
import Part
import Sketcher

mode_name = "SparseLDLT" if os.getenv("GCS_SPARSE_LDLT") else "FullPivLU"
NODE_COUNTS = [100, 200, 300, 500]
N_WARMUP = 5
N_MEASURED = 40

import sys
# DO NOT redirect stderr — we need [SOLVER_PROFILE] output and [ITERATION_COUNT]

results = []

for NUM_NODES in NODE_COUNTS:
    doc = App.newDocument(f"Bench{NUM_NODES}")
    sketch = doc.addObject("Sketcher::SketchObject", "Sketch")

    # Build chain of line segments with coincident constraints + local per-segment Distance
    sketch.addGeometry(Part.LineSegment(App.Vector(0, 0, 0), App.Vector(10, 10, 0)), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 1, -1, 1))
    drv = sketch.addConstraint(Sketcher.Constraint("Distance", 0, 10.0))
    for i in range(1, NUM_NODES):
        sketch.addGeometry(Part.LineSegment(App.Vector(i * 10, 0, 0), App.Vector((i + 1) * 10, 10, 0)), False)
        sketch.addConstraint(Sketcher.Constraint("Coincident", i - 1, 2, i, 1))
        sketch.addConstraint(Sketcher.Constraint("Distance", i, 10.0))

    try:
        doc.recompute()
    except Exception as e:
        App.Console.PrintMessage(f"[{NUM_NODES} nodes] Initial recompute FAILED: {e}\n")
        App.closeDocument(doc.Name)
        results.append((NUM_NODES, float('nan'), 0, "INIT_FAIL"))
        continue

    # Warmup
    for w in range(N_WARMUP):
        try:
            sketch.setDatum(drv, App.Units.Quantity(f"{10.0 + w * 0.1} mm"))
            doc.recompute()
        except Exception:
            pass

    # Timed loop — toggle first segment's datum
    failures = 0
    t0 = time.perf_counter()
    for i in range(N_MEASURED):
        try:
            val = 10.0 + (i % 20) * 0.5
            sketch.setDatum(drv, App.Units.Quantity(f"{val} mm"))
            doc.recompute()
        except Exception:
            failures += 1
            if failures >= 3:  # abort tier on 3 consecutive failures
                break
    t1 = time.perf_counter()

    n_ok = i + 1 - failures  # i+1 because loop may have broken early
    if n_ok > 0:
        avg_ms = (t1 - t0) / n_ok * 1000
    else:
        avg_ms = float('nan')

    status = "OK" if failures == 0 else f"DIVERGED({failures}/{n_ok + failures})"
    
    App.Console.PrintMessage(
        f"\n[{mode_name}] {NUM_NODES} nodes | {n_ok} solves | Avg: {avg_ms:.2f} ms | Status: {status}\n"
    )
    App.Console.PrintMessage(
        f"[PERF PROFILE DENSE] {mode_name} | Nodes: {NUM_NODES} | Avg: {avg_ms:.2f} ms | Status: {status}\n"
    )

    results.append((NUM_NODES, avg_ms, failures, status))
    App.closeDocument(doc.Name)

# Summary
App.Console.PrintMessage(f"\n=== DENSE BENCHMARK SUMMARY [{mode_name}] ===\n")
for nodes, avg_ms, failures, status in results:
    App.Console.PrintMessage(f"  {nodes} nodes: {avg_ms:.2f} ms  ({status})\n")
