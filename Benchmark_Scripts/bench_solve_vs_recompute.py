#!/usr/bin/env python3
"""Partition dense-sketch update cost: sketch.solve() (the constraint solver)
vs doc.recompute() (solver + geometry/shape rebuild + dependency graph).
Tells us whether the solver is the bottleneck users actually feel."""
import time
import FreeCAD as App
import Part
import Sketcher

for NUM_NODES in [100, 300, 500]:
    doc = App.newDocument(f"Probe{NUM_NODES}")
    sk = doc.addObject("Sketcher::SketchObject", "Sketch")
    sk.addGeometry(Part.LineSegment(App.Vector(0, 0, 0), App.Vector(10, 10, 0)), False)
    sk.addConstraint(Sketcher.Constraint("Coincident", 0, 1, -1, 1))
    drv = sk.addConstraint(Sketcher.Constraint("Distance", 0, 10.0))
    for i in range(1, NUM_NODES):
        sk.addGeometry(Part.LineSegment(App.Vector(i*10, 0, 0), App.Vector((i+1)*10, 10, 0)), False)
        sk.addConstraint(Sketcher.Constraint("Coincident", i-1, 2, i, 1))
        sk.addConstraint(Sketcher.Constraint("Distance", i, 10.0))
    doc.recompute()

    N = 20
    # warmup
    for w in range(3):
        sk.setDatum(drv, App.Units.Quantity(f"{10.0 + w*0.1} mm"))
        sk.solve()

    # time solve() only (no recompute)
    t0 = time.perf_counter()
    for i in range(N):
        sk.setDatum(drv, App.Units.Quantity(f"{10.0 + (i%20)*0.5} mm"))
        sk.solve()
    t1 = time.perf_counter()
    solve_ms = (t1 - t0)/N*1000

    # time full recompute
    t0 = time.perf_counter()
    for i in range(N):
        sk.setDatum(drv, App.Units.Quantity(f"{10.0 + (i%20)*0.5} mm"))
        doc.recompute()
    t1 = time.perf_counter()
    recompute_ms = (t1 - t0)/N*1000

    App.Console.PrintMessage(
        f"[PARTITION] {NUM_NODES} nodes | solve()={solve_ms:.2f} ms | recompute()={recompute_ms:.2f} ms "
        f"| solver share={100*solve_ms/recompute_ms:.0f}%\n")
    App.closeDocument(doc.Name)
