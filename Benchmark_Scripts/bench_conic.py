#!/usr/bin/env python3
"""Conic benchmark: exercises 4 errorgrad() constraint types in isolated subsystems.

Each errorgrad constraint type is tested in its own independently-solvable sketch
to avoid cross-system cascading failures under perturbation.

Errorgrad types exercised:
  - C2CDistance (30)  [toggled]
  - P2CDistance (32)  [toggled]
  - C2LDistance (31)  [toggled]
  - ArcLength (36)    [toggled]
  - EqualLineLength (25) [static, non-toggled]

N_MEASURED=100 solves per trial per type, N_TRIALS=3, reports median.
"""

import os, sys, time, statistics, math
import FreeCAD as App
import Part
import Sketcher

N_WARMUP = 5
N_MEASURED = 100
N_TRIALS = 3

mode_name = "SparseLDLT" if os.getenv("GCS_SPARSE_LDLT") else "FullPivLU"

# Silence C++ stderr noise
sys.stderr = open(os.devnull, 'w')


def build_c2cdistance(doc):
    """C2CDistance (type 30): Circle C1(fixed) to Circle C2(free center X)."""
    sketch = doc.addObject("Sketcher::SketchObject", "C2CDistance")
    sketch.addGeometry(Part.Circle(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 10), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 3, -1, 1))
    sketch.addConstraint(Sketcher.Constraint("Radius", 0, 10.0))
    sketch.addGeometry(Part.Circle(App.Vector(30, 0, 0), App.Vector(0, 0, 1), 5), False)
    sketch.addConstraint(Sketcher.Constraint("Radius", 1, 5.0))
    sketch.addConstraint(Sketcher.Constraint("DistanceY", 1, 3, -1, 1, 0.0))
    drv = sketch.addConstraint(Sketcher.Constraint("Distance", 0, 1, 15.0))
    doc.recompute()
    return sketch, drv


def build_p2cdistance(doc):
    """P2CDistance (type 32): Circle C0 center (fixed) to Circle C1 (free center X).
    
    Uses C0 center as the "point" for P2CDistance. C0 center at (0,0),
    C1 at (30,0) radius 5 → center-to-circumference = 30-5 = 25.
    """
    sketch = doc.addObject("Sketcher::SketchObject", "P2CDistance")
    # Circle 0 at origin - center acts as the fixed "point"
    sketch.addGeometry(Part.Circle(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 10), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 3, -1, 1))  # fix center
    sketch.addConstraint(Sketcher.Constraint("Radius", 0, 10.0))
    # Circle 1 at (30,0) radius 5, center Y fixed
    sketch.addGeometry(Part.Circle(App.Vector(30, 0, 0), App.Vector(0, 0, 1), 5), False)
    sketch.addConstraint(Sketcher.Constraint("Radius", 1, 5.0))
    sketch.addConstraint(Sketcher.Constraint("DistanceY", 1, 3, -1, 1, 0.0))
    # P2CDistance: C0 center to C1 circumference = 30-5 = 25
    drv = sketch.addConstraint(Sketcher.Constraint("Distance", 0, 3, 1, 25.0))
    doc.recompute()
    return sketch, drv


def build_c2ldistance(doc):
    """C2LDistance (type 31): Circle C1(free center Y) to Line L1(fixed horizontal)."""
    sketch = doc.addObject("Sketcher::SketchObject", "C2LDistance")
    sketch.addGeometry(Part.Circle(App.Vector(30, 20, 0), App.Vector(0, 0, 1), 8), False)
    sketch.addConstraint(Sketcher.Constraint("Radius", 0, 8.0))
    sketch.addConstraint(Sketcher.Constraint("DistanceX", 0, 3, -1, 1, 30.0))
    sketch.addGeometry(Part.LineSegment(App.Vector(0, 0, 0), App.Vector(60, 0, 0)), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 1, 1, -1, 1))
    sketch.addConstraint(Sketcher.Constraint("Horizontal", 1))
    drv = sketch.addConstraint(Sketcher.Constraint("Distance", 0, 1, 12.0))
    doc.recompute()
    return sketch, drv


def build_arclength(doc):
    """ArcLength (type 36): Arc A1(center+radius fixed, angles free) + EqualLineLength for L2/L3."""
    sketch = doc.addObject("Sketcher::SketchObject", "ArcLength")
    sketch.addGeometry(
        Part.ArcOfCircle(Part.Circle(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 10),
                         0.0, 2.0), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 0, 3, -1, 1))
    sketch.addConstraint(Sketcher.Constraint("Radius", 0, 10.0))
    drv = sketch.addConstraint(Sketcher.Constraint("Distance", 0, 5.0))
    # Static EqualLineLength (type 25): Line L2 to Line L3
    sketch.addGeometry(Part.LineSegment(App.Vector(0, 30, 0), App.Vector(30, 30, 0)), False)
    sketch.addGeometry(Part.LineSegment(App.Vector(0, 40, 0), App.Vector(30, 40, 0)), False)
    sketch.addConstraint(Sketcher.Constraint("Coincident", 1, 1, -1, 1))
    sketch.addConstraint(Sketcher.Constraint("Horizontal", 1))
    sketch.addConstraint(Sketcher.Constraint("Coincident", 2, 1, -1, 1))
    sketch.addConstraint(Sketcher.Constraint("Horizontal", 2))
    sketch.addConstraint(Sketcher.Constraint("Equal", 1, 2))
    doc.recompute()
    return sketch, drv


# ---------------------------------------------------------------------------
# Build all sketches
# ---------------------------------------------------------------------------
doc = App.newDocument("BenchConic")
sketches = [
    ("C2CDistance", *build_c2cdistance(doc)),
    ("P2CDistance", *build_p2cdistance(doc)),
    ("C2LDistance", *build_c2ldistance(doc)),
    ("ArcLength", *build_arclength(doc)),
]

# ---------------------------------------------------------------------------
# Warmup — one pass through all types
# ---------------------------------------------------------------------------
for name, sketch, drv in sketches:
    for i in range(N_WARMUP):
        sketch.setDatum(drv, App.Units.Quantity(f"{15.0 + i * 0.2} mm"))
        doc.recompute()

# ---------------------------------------------------------------------------
# Benchmark: per-type, each type perturbed sinusoidally
# ---------------------------------------------------------------------------
all_type_avgs = []
total_failures = 0

for name, sketch, drv in sketches:
    type_avgs = []
    App.Console.PrintMessage(f"\n--- {name} ---\n")
    for trial in range(1, N_TRIALS + 1):
        failures = 0
        t0 = time.perf_counter()
        for i in range(N_MEASURED):
            try:
                # Sinusoidal perturbation around a base value
                base = 15.0 if name != "ArcLength" else 5.0
                amp = 2.0 if name != "ArcLength" else 1.0
                val = base + amp * math.sin(i * 0.15 + trial * 2.0)
                sketch.setDatum(drv, App.Units.Quantity(f"{val} mm"))
                doc.recompute()
            except Exception:
                failures += 1
        t1 = time.perf_counter()
        n_ok = N_MEASURED - failures
        avg_ms = (t1 - t0) / n_ok * 1000 if n_ok > 0 else float('nan')
        type_avgs.append(avg_ms)
        total_failures += failures
        App.Console.PrintMessage(
            f"  Trial {trial}: {avg_ms:.2f} ms  Failures: {failures}/{N_MEASURED}\n"
        )

    median_ms = statistics.median([a for a in type_avgs if a == a])
    all_type_avgs.append(median_ms)
    App.Console.PrintMessage(f"  Median {name}: {median_ms:.2f} ms\n")

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
App.Console.PrintMessage(f"\n=== CONIC BENCHMARK [{mode_name}] ===\n")
App.Console.PrintMessage(f"Trials per type: {N_TRIALS} x {N_MEASURED} solves\n")
for name, median_ms in zip([s[0] for s in sketches], all_type_avgs):
    App.Console.PrintMessage(f"  {name}: {median_ms:.2f} ms\n")
App.Console.PrintMessage(f"Total failures: {total_failures}\n")

overall_median = statistics.median([a for a in all_type_avgs if a == a])
App.Console.PrintMessage(f"Overall median: {overall_median:.2f} ms\n")
App.Console.PrintMessage(
    f"[PERF_PROFILE_CONIC] {mode_name} | Overall Median: {overall_median:.2f} ms | "
    f"Failures: {total_failures}\n"
)

App.closeDocument(doc.Name)
