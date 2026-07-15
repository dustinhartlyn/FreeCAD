#!/usr/bin/env python3
"""Force-recompute benchmark: FullPivLU vs SparseLDLT.
Set GCS_SPARSE_LDLT=1 for SparseLDLT; unset for FullPivLU.
Triggers real re-solve by toggling constraint value each iteration."""
import os, time
import FreeCAD as App
import Part
import Sketcher

mode_name = "SparseLDLT" if os.getenv("GCS_SPARSE_LDLT") else "FullPivLU"
NUM_NODES = 100
N_WARMUP = 5
N_MEASURED = 40

# Quiet down C++ stderr noise
import sys
sys.stderr = open(os.devnull, 'w')

doc = App.newDocument("Bench")
sketch = doc.addObject("Sketcher::SketchObject", "Sketch")

# Build chain of line segments with coincident+equal constraints
sketch.addGeometry(Part.LineSegment(App.Vector(0,0,0), App.Vector(10,10,0)), False)
sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 1, -1, 1))
for i in range(1, NUM_NODES):
    sketch.addGeometry(Part.LineSegment(App.Vector(i*10,0,0), App.Vector((i+1)*10,10,0)), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", i-1, 2, i, 1))
    sketch.addConstraint(Sketcher.Constraint("Equal", i-1, i))

# Add driving DistanceX constraint between first and last points
drv = sketch.addConstraint(Sketcher.Constraint("DistanceX", 0, 1, NUM_NODES-1, 2, 1000.0))
doc.recompute()

# Warmup
for _ in range(N_WARMUP):
    sketch.setDatum(drv, App.Units.Quantity(f"{1000.0 + _ * 0.1} mm"))
    doc.recompute()

# Timed loop — toggle datum to force actual re-solve
t0 = time.perf_counter()
for i in range(N_MEASURED):
    val = 1000.0 + (i % 20) * 0.5
    sketch.setDatum(drv, App.Units.Quantity(f"{val} mm"))
    doc.recompute()
t1 = time.perf_counter()

avg_ms = (t1 - t0) / N_MEASURED * 1000

App.Console.PrintMessage(f"\n[{mode_name}] {NUM_NODES} nodes | {N_MEASURED} solves | Avg: {avg_ms:.2f} ms\n")
App.Console.PrintMessage(f"[PERF PROFILE] {mode_name} | Nodes: {NUM_NODES} | Avg: {avg_ms:.2f} ms\n")

App.closeDocument(doc.Name)
