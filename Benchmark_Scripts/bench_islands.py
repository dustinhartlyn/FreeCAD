#!/usr/bin/env python3
"""Independent-component (island) benchmark: many disconnected sub-sketches that
share no constraints, to exercise the parallel connected-component solve.
Each island is a moderate chain so per-component work is non-trivial."""
import time, FreeCAD as App, Part, Sketcher

ISLANDS = 12      # independent components
SEG = 20          # segments per island
N = 15

doc = App.newDocument("Islands")
sk = doc.addObject("Sketcher::SketchObject", "Sketch")
gid = 0
first_datum = None
for isl in range(ISLANDS):
    ox = isl * 1000.0  # keep islands far apart / disconnected
    sk.addGeometry(Part.LineSegment(App.Vector(ox, 0, 0), App.Vector(ox+10, 10, 0)), False)
    base = gid
    sk.addConstraint(Sketcher.Constraint("DistanceX", base, 1, ox))  # anchor this island only
    sk.addConstraint(Sketcher.Constraint("DistanceY", base, 1, 0.0))
    d = sk.addConstraint(Sketcher.Constraint("Distance", base, 10.0))
    if first_datum is None:
        first_datum = d
    gid += 1
    for s in range(1, SEG):
        sk.addGeometry(Part.LineSegment(App.Vector(ox+s*10, 0, 0), App.Vector(ox+(s+1)*10, 10, 0)), False)
        sk.addConstraint(Sketcher.Constraint("Coincident", gid-1, 2, gid, 1))
        sk.addConstraint(Sketcher.Constraint("Distance", gid, 10.0))
        gid += 1
doc.recompute()

for w in range(3):
    sk.setDatum(first_datum, App.Units.Quantity(f"{10.0+w*0.1} mm")); sk.solve()

t0 = time.perf_counter()
for i in range(N):
    sk.setDatum(first_datum, App.Units.Quantity(f"{10.0+(i%10)*0.3} mm")); sk.solve()
t1 = time.perf_counter()
App.Console.PrintMessage(
    f"[ISLANDS] {ISLANDS} islands x {SEG} segs | solve()={(t1-t0)/N*1000:.2f} ms\n")
App.closeDocument(doc.Name)
