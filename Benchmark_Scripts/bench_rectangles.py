#!/usr/bin/env python3
"""Reproduce 'adding a dimension stalls' on a dense array of constrained
rectangles. Batch-adds geometry/constraints (one diagnose), then times adding a
single dimension (topology change -> full diagnose())."""
import time, FreeCAD as App, Part, Sketcher

RECTS = 150
doc = App.newDocument("Rects")
sk = doc.addObject("Sketcher::SketchObject", "Sketch")

geos = []
for r in range(RECTS):
    ox, oy = (r % 15) * 50.0, (r // 15) * 30.0
    geos += [
        Part.LineSegment(App.Vector(ox,    oy,    0), App.Vector(ox+20, oy,    0)),
        Part.LineSegment(App.Vector(ox+20, oy,    0), App.Vector(ox+20, oy+10, 0)),
        Part.LineSegment(App.Vector(ox+20, oy+10, 0), App.Vector(ox,    oy+10, 0)),
        Part.LineSegment(App.Vector(ox,    oy+10, 0), App.Vector(ox,    oy,    0)),
    ]
sk.addGeometry(geos, False)

cons = []
for r in range(RECTS):
    a = r * 4; b = a + 1; c = a + 2; d = a + 3
    ox, oy = (r % 15) * 50.0, (r // 15) * 30.0
    cons += [
        Sketcher.Constraint("Coincident", a, 2, b, 1),
        Sketcher.Constraint("Coincident", b, 2, c, 1),
        Sketcher.Constraint("Coincident", c, 2, d, 1),
        Sketcher.Constraint("Coincident", d, 2, a, 1),
        Sketcher.Constraint("Horizontal", a),
        Sketcher.Constraint("Horizontal", c),
        Sketcher.Constraint("Vertical", b),
        Sketcher.Constraint("Vertical", d),
        Sketcher.Constraint("DistanceX", a, 1, ox),
        Sketcher.Constraint("DistanceY", a, 1, oy),
        Sketcher.Constraint("DistanceX", a, 1, a, 2, 20.0),
        Sketcher.Constraint("DistanceY", b, 1, b, 2, 10.0),
    ]
sk.addConstraint(cons)

t0 = time.perf_counter(); doc.recompute(); t1 = time.perf_counter()
App.Console.PrintMessage(f"[RECTS] {RECTS} rects ({RECTS*4} lines) | initial recompute={(t1-t0)*1000:.0f} ms\n")

N = 3
times = []
for i in range(N):
    cidx = sk.addConstraint(Sketcher.Constraint("Distance", 0, 20.0 + i))
    t0 = time.perf_counter(); doc.recompute(); t1 = time.perf_counter()
    times.append((t1-t0)*1000)
    sk.delConstraint(cidx); doc.recompute()
App.Console.PrintMessage(f"[RECTS] add-dimension recompute avg = {sum(times)/len(times):.0f} ms  (each: {['%.0f'%x for x in times]})\n")
App.closeDocument(doc.Name)
