#!/usr/bin/env python3
"""Drag snap-to-origin regression repro (dev-only, headless via FreeCADCmd).

Reproduces the drag orchestration end-to-end (moveGeometry -> initTemporaryMove ->
moveGeometries -> two-subsystem SQP) and checks that dragging one vertex of an
underconstrained sketch does NOT cause other vertices to collapse onto the
horizontal origin line (Y = 0). Historically this "snap to origin" appeared with
cluster decomposition enabled (useClusters=true); the fix defaults useClusters to
false so the drag is seeded from a rank-healthy monolithic solve.

Exit code 0 = no spurious snap (PASS), 1 = snap detected (FAIL).
"""
import sys
import FreeCAD as App
import Part
import Sketcher

SNAP_TOL = 1e-6      # |Y| below this counts as "on the origin line"
AWAY_TOL = 0.5       # a vertex this far from Y=0 before the drag must stay off it

def main():
    doc = App.newDocument("DragSnapRepro")
    sk = doc.addObject("Sketcher::SketchObject", "Sketch")

    # Underconstrained fan/chain: several segments whose free ends sit well above
    # the X axis. Only coincident chaining is applied, leaving DOF for dragging.
    NUM = 8
    sk.addGeometry(Part.LineSegment(App.Vector(0, 5, 0), App.Vector(10, 8, 0)), False)
    for i in range(1, NUM):
        y0 = 5 + i
        y1 = 8 + i
        sk.addGeometry(Part.LineSegment(App.Vector(i * 10, y0, 0),
                                        App.Vector((i + 1) * 10, y1, 0)), False)
        # chain: end of prev segment coincident with start of this one
        sk.addConstraint(Sketcher.Constraint("Coincident", i - 1, 2, i, 1))
    doc.recompute()

    # Record every vertex Y (start=1, end=2 of each segment)
    def all_ys():
        ys = []
        for g in range(NUM):
            for pos in (1, 2):
                ys.append((g, pos, sk.getPoint(g, pos).y))
        return ys

    before = all_ys()

    # Drag the free end of the LAST segment upward-and-over by a small vector.
    # relative=False -> absolute target near its current location.
    tgt = sk.getPoint(NUM - 1, 2)
    sk.moveGeometry(NUM - 1, 2, App.Vector(tgt.x + 3.0, tgt.y + 3.0, 0), False)
    doc.recompute()

    after = {(g, pos): y for (g, pos, y) in all_ys()}

    # Any vertex that was clearly OFF the origin line before must not have
    # collapsed onto it (that is the snap-to-origin bug).
    snapped = []
    for (g, pos, y_before) in before:
        y_after = after[(g, pos)]
        if abs(y_before) > AWAY_TOL and abs(y_after) < SNAP_TOL:
            snapped.append((g, pos, y_before, y_after))

    App.Console.PrintMessage("\n[DRAG_SNAP_REPRO] vertices checked: %d\n" % len(before))
    if snapped:
        App.Console.PrintMessage("[DRAG_SNAP_REPRO] RESULT: FAIL - snapped to origin:\n")
        for (g, pos, yb, ya) in snapped:
            App.Console.PrintMessage("    geo %d pos %d: Y %.4f -> %.8f\n" % (g, pos, yb, ya))
    else:
        App.Console.PrintMessage("[DRAG_SNAP_REPRO] RESULT: PASS - no spurious snap to origin\n")

    App.closeDocument(doc.Name)
    return 1 if snapped else 0

if __name__ == "__main__":
    sys.exit(main())
