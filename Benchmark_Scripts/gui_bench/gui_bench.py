#!/usr/bin/env python3
"""In-GUI Sketcher responsiveness benchmark.

Runs INSIDE a FreeCAD GUI process (launched as `freecad.exe gui_bench.py`).
Unlike the headless benches, this exercises the full interactive path that the
user actually feels:

    drag frame = moveGeometries()        (constraint solve)
               + ViewProviderSketch::draw(true,false)   (Coin scene-graph rebuild)
               + View3DInventorViewer::redraw()          (GL repaint)

The headless benchmarks measured only the first term. This script enters real
Sketcher edit mode and drives the four constraint-editing scenarios through the
full GUI redraw path (setEdit / add-dimension / datum-edit / recompute each fire
ViewProviderSketch::draw()). For the DRAG scenario it (a) validates the vertex is
interactively grabbable with a synthetic QTest click — which DOES drive Coin's
pick pipeline — then (b) times each drag frame via moveGeometry(), the identical
`moveGeometries` solve the interactive drag runs, plus a forced draw()+repaint.
(Synthetic mouse-MOVE events are not delivered to the SoQt/Quarter drag handler
from Python, so they cannot drive a live drag; moveGeometry measures the exact
same per-frame work.)

Configuration comes from environment variables (set by run_gui_bench.ps1):

    GUI_BENCH_OUT      path to write JSON results (required)
    GUI_BENCH_LABEL    build label, e.g. "dev" or "stock-1.1"  (default "unknown")
    GUI_BENCH_SIZES    comma list of sketch sizes, e.g. "50,100,200" (default "50,100,200")
    GUI_BENCH_SCENARIOS  comma list from
                         {enter_edit,add_dimension,datum_edit,recompute,drag}
                         (default: all)
    GUI_BENCH_DEV_SKETCHER  optional dir to sys.path.insert BEFORE importing
                            Sketcher (dev-pyd fallback; usually the harness
                            copies the pyd into the env instead)

Results are written as JSON and the process is hard-exited so the launching
harness can proceed to the next build.
"""

import os
import sys
import json
import time
import traceback

# ---------------------------------------------------------------------------
# Optional dev-pyd injection (fallback; the harness normally copies the pyd in)
# ---------------------------------------------------------------------------
_dev_sketcher = os.environ.get("GUI_BENCH_DEV_SKETCHER")
if _dev_sketcher and os.path.isdir(_dev_sketcher):
    sys.path.insert(0, _dev_sketcher)

import FreeCAD as App          # noqa: E402
import Part                    # noqa: E402
import Sketcher                # noqa: E402
import FreeCADGui as Gui       # noqa: E402

LABEL = os.environ.get("GUI_BENCH_LABEL", "unknown")
OUT = os.environ.get("GUI_BENCH_OUT")
SIZES = [int(x) for x in os.environ.get("GUI_BENCH_SIZES", "50,100,200").split(",") if x.strip()]
SCENARIOS = [s.strip() for s in os.environ.get(
    "GUI_BENCH_SCENARIOS",
    "enter_edit,add_dimension,datum_edit,recompute,drag").split(",") if s.strip()]
PROGRESS = (OUT + ".progress") if OUT else None
WATCHDOG_SEC = int(os.environ.get("GUI_BENCH_WATCHDOG", "300"))


def stage(msg):
    """Append a stage marker so a hang is diagnosable from the .progress file."""
    line = f"{time.strftime('%H:%M:%S')} {msg}\n"
    try:
        App.Console.PrintMessage(f"[gui_bench:{LABEL}] {msg}\n")
    except Exception:
        pass
    if PROGRESS:
        try:
            with open(PROGRESS, "a") as f:
                f.write(line); f.flush()
        except Exception:
            pass

# ---------------------------------------------------------------------------
# Qt binding shim (FreeCAD 1.1 may ship PySide2 or PySide6)
# ---------------------------------------------------------------------------
QtCore = QtGui = QtWidgets = None
QtOpenGLWidgetClass = None
try:
    from PySide6 import QtCore, QtGui, QtWidgets          # noqa: E402
    from PySide6.QtTest import QTest                       # noqa: E402
    try:
        from PySide6.QtOpenGLWidgets import QOpenGLWidget as QtOpenGLWidgetClass
    except Exception:
        QtOpenGLWidgetClass = None
    _QT = "PySide6"
except Exception:
    from PySide2 import QtCore, QtGui, QtWidgets          # noqa: E402
    from PySide2.QtTest import QTest                       # noqa: E402
    QtOpenGLWidgetClass = getattr(QtWidgets, "QOpenGLWidget", None)
    _QT = "PySide2"


def pump(n=1):
    """Flush the Qt event queue so scheduled redraws actually happen."""
    app = QtWidgets.QApplication.instance()
    for _ in range(n):
        app.processEvents(QtCore.QEventLoop.AllEvents, 50)


def now():
    return time.perf_counter()


# ---------------------------------------------------------------------------
# Sketch builders
# ---------------------------------------------------------------------------
def build_rect_array(sk, rects):
    """Dense array of fully-constrained rectangles -> the add-dimension /
    datum-edit / recompute (diagnose + rebuild) stress case."""
    geos = []
    for r in range(rects):
        ox, oy = (r % 15) * 50.0, (r // 15) * 30.0
        geos += [
            Part.LineSegment(App.Vector(ox, oy, 0), App.Vector(ox + 20, oy, 0)),
            Part.LineSegment(App.Vector(ox + 20, oy, 0), App.Vector(ox + 20, oy + 10, 0)),
            Part.LineSegment(App.Vector(ox + 20, oy + 10, 0), App.Vector(ox, oy + 10, 0)),
            Part.LineSegment(App.Vector(ox, oy + 10, 0), App.Vector(ox, oy, 0)),
        ]
    sk.addGeometry(geos, False)
    cons = []
    for r in range(rects):
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


def build_chain(sk, nodes):
    """Floppy chain: coincident-linked segments, only the first point pinned.
    No per-segment Distance constraints (their dimension icons would otherwise
    clutter every vertex and block a clean grab). The far tip is a free,
    icon-clear, draggable vertex; dragging it re-solves the whole chain -> the
    dense DRAG stress case."""
    sk.addGeometry(Part.LineSegment(App.Vector(0, 0, 0), App.Vector(10, 0, 0)), False)
    sk.addConstraint(Sketcher.Constraint("Coincident", 0, 1, -1, 1))  # pin start to origin
    for i in range(1, nodes):
        sk.addGeometry(
            Part.LineSegment(App.Vector(i * 10, 0, 0), App.Vector((i + 1) * 10, 0, 0)), False)
        sk.addConstraint(Sketcher.Constraint("Coincident", i - 1, 2, i, 1))
    return nodes - 1  # geoId of the last segment (its point 2 is the free tip)


# ---------------------------------------------------------------------------
# GUI helpers
# ---------------------------------------------------------------------------
def get_view():
    """Return the active View3DInventorPy, ensuring one exists."""
    gd = Gui.activeDocument()
    if gd is None:
        return None
    v = gd.activeView()
    if v is None:
        try:
            views = gd.mdiViewsOfType("Gui::View3DInventor")
            v = views[0] if views else None
        except Exception:
            v = None
    return v


def get_gl_widget():
    """Find the QOpenGLWidget backing the active 3D view."""
    mw = Gui.getMainWindow()
    mdi = mw.findChild(QtWidgets.QMdiArea)
    sub = mdi.currentSubWindow() or mdi.activeSubWindow()
    root = sub.widget() if sub else mw
    if QtOpenGLWidgetClass is not None:
        gl = root.findChild(QtOpenGLWidgetClass)
        if gl is not None:
            return gl
    # Fallback: the largest child widget under the MDI subwindow.
    best, area = None, -1
    for w in root.findChildren(QtWidgets.QWidget):
        a = w.width() * w.height()
        if a > area:
            best, area = w, a
    return best


def model_to_widget(view, gl, vec):
    """Sketch-3D point -> QPointF in gl widget-local (logical, top-left) coords."""
    px, py = view.getPointOnScreen(vec)         # Coin viewport pixels, origin bottom-left, device px
    dpr = gl.devicePixelRatioF() if hasattr(gl, "devicePixelRatioF") else 1.0
    lx = px / dpr
    ly = gl.height() - (py / dpr)               # flip Y to Qt top-left origin
    return QtCore.QPointF(lx, ly)


def grab_check(view, gl, sk, geo_id, pt_idx):
    """Prove the target vertex is interactively grabbable in the real GUI: warp
    the cursor to its projected position with QTest and click. QTest press/click
    DO drive Coin's pick pipeline (unlike raw QMouseEvents). Returns True if the
    click selected a vertex of our sketch — i.e. a real hand-drag would grab it.

    (We validate the grab with a synthetic click, but *time* the drag frames via
    moveGeometry below: synthetic mouse-MOVE events are not delivered to the
    SoQt/Quarter drag handler from Python, so they can't drive an actual drag —
    while moveGeometry runs the identical `moveGeometries` solve the interactive
    drag uses, and Gui.updateGui() forces the same ViewProviderSketch::draw()
    scene rebuild + repaint. So the measured frame = real drag frame work.)"""
    try:
        center = model_to_widget(view, gl, sk.getPoint(geo_id, pt_idx)).toPoint()
        gl.setFocus(); pump(1)
        # try the exact projection first, then a small neighborhood (HiDPI / marker
        # tolerance can put the pickable point a couple logical px off).
        offsets = [(0, 0)]
        for r in (2, 4, 6):                                  # widening rings
            offsets += [(r, 0), (-r, 0), (0, r), (0, -r), (r, -r), (-r, r), (r, r), (-r, -r)]
        for dx, dy in offsets:
            p = QtCore.QPoint(center.x() + dx, center.y() + dy)
            Gui.Selection.clearSelection(); pump(1)
            QTest.mouseMove(gl, p); QTest.qWait(15)          # hover to preselect
            QTest.mouseClick(gl, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, p)
            QTest.qWait(15); pump(2)
            subs = []
            for so in Gui.Selection.getSelectionEx():
                subs += list(so.SubElementNames)
            Gui.Selection.clearSelection(); pump(1)
            if any("Vertex" in s for s in subs):
                return True, subs
        return False, subs
    except Exception as e:
        return False, [f"grab_check error: {e}"]


def simulate_drag(view, gl, sk, geo_id, pt_idx, steps=24):
    """Time a dense interactive drag: sweep the free tip through an arc, one
    frame per step. Each frame's CPU work =
        moveGeometry()  (the identical moveGeometries solve the live drag runs)
      + Gui.updateGui() (fires ViewProviderSketch::draw() — the Coin scene-graph
                         rebuild that scales with sketch size).
    We deliberately do NOT force a synchronous GL repaint inside the timing: that
    blocks on vsync (~16/33 ms buckets) and would swamp the solver+rebuild signal
    with GPU-paint noise. The scene-graph rebuild (the N-scaling GUI cost) is
    captured; the near-constant GL blit is not. Returns (per-frame ms, moved)."""
    import math
    start = sk.getPoint(geo_id, pt_idx)
    radius = max((start - App.Vector(0, 0, 0)).Length, 1.0)

    frames = []
    for k in range(1, steps + 1):
        ang = k * (1.2 / steps)                       # sweep up to ~1.2 rad
        target = App.Vector(radius * math.cos(ang), radius * math.sin(ang), start.z)
        t0 = now()
        sk.moveGeometry(geo_id, pt_idx, target, 0)    # solve
        Gui.updateGui()                               # -> updateData -> draw() rebuild
        frames.append((now() - t0) * 1000.0)

    gl.repaint(); pump(1)                             # flush one real frame (untimed)
    moved = (sk.getPoint(geo_id, pt_idx) - start).Length
    return frames, moved


# ---------------------------------------------------------------------------
# Scenario runners (each returns a dict of measurements)
# ---------------------------------------------------------------------------
def enter_edit(sk):
    """setEdit + fit; returns (view, elapsed_ms)."""
    t0 = now()
    Gui.activeDocument().setEdit(sk, 0)
    pump(3)
    view = get_view()
    try:
        view.fitAll()
    except Exception:
        pass
    pump(2)
    return view, (now() - t0) * 1000.0


def summarize(frames):
    fs = sorted(frames)
    n = len(fs)
    return {
        "ms_each": frames,
        "ms": fs[n // 2],                    # median headline (robust to warm-up/vsync outliers)
        "median_ms": fs[n // 2],
        "min_ms": fs[0],
        "max_ms": fs[-1],
    }


def run_size(size):
    res = {"size": size, "scenarios": {}}

    def record(name, fn):
        """Run one scenario in isolation so a failure never loses the others."""
        if name not in SCENARIOS:
            return
        try:
            stage(f"size={size} scenario={name}")
            res["scenarios"][name] = fn()
        except Exception as e:
            res["scenarios"][name] = {"error": f"{e}\n{traceback.format_exc()}"}
            stage(f"size={size} scenario={name} ERROR {e}")

    # ---- rectangle-array scenarios: enter_edit / add_dimension / datum_edit / recompute
    rect_scenarios = ("enter_edit", "add_dimension", "datum_edit", "recompute")
    if any(s in SCENARIOS for s in rect_scenarios):
        doc = App.newDocument("rects")
        sk = doc.addObject("Sketcher::SketchObject", "Sketch")
        build_rect_array(sk, size)
        doc.recompute()
        stage(f"size={size} rect array built ({size*4} lines)")

        view, edit_ms = enter_edit(sk)   # must enter edit before the GUI scenarios
        if "enter_edit" in SCENARIOS:
            res["scenarios"]["enter_edit"] = {"ms": edit_ms}
        stage(f"size={size} in edit mode ({edit_ms:.0f} ms)")

        def s_add_dimension():
            # topology change -> full diagnose() each time (the dimension-add stall)
            times = []
            for i in range(3):
                t0 = now()
                cidx = sk.addConstraint(Sketcher.Constraint("Distance", 0, 20.0 + i))
                doc.recompute(); pump(2)
                times.append((now() - t0) * 1000.0)
                sk.delConstraint(cidx); doc.recompute(); pump(1)
            return summarize(times)

        def s_datum_edit():
            # edit an EXISTING driving datum (rect 0 width = global constraint 10):
            # value-only change -> solve + rebuild + redraw, no topology change
            drv = 10
            times = []
            for i in range(6):
                t0 = now()
                sk.setDatum(drv, App.Units.Quantity(f"{20.0 + (i % 4) * 0.5} mm"))
                doc.recompute(); pump(2)
                times.append((now() - t0) * 1000.0)
            sk.setDatum(drv, App.Units.Quantity("20.0 mm")); doc.recompute(); pump(1)
            return summarize(times)

        def s_recompute():
            times = []
            for _ in range(4):
                sk.touch()
                t0 = now()
                doc.recompute(); pump(2)
                times.append((now() - t0) * 1000.0)
            return summarize(times)

        record("add_dimension", s_add_dimension)
        record("datum_edit", s_datum_edit)
        record("recompute", s_recompute)

        try:
            Gui.activeDocument().resetEdit(); pump(1)
        except Exception:
            pass
        App.closeDocument(doc.Name); pump(1)

    # ---- drag : floppy chain, synthetic Qt mouse events --------------------
    if "drag" in SCENARIOS:
        doc = App.newDocument("chain")
        sk = doc.addObject("Sketcher::SketchObject", "Sketch")
        last_geo = build_chain(sk, size)
        doc.recompute()
        stage(f"size={size} chain built, entering edit for drag")
        view, _ = enter_edit(sk)
        gl = get_gl_widget()
        stage(f"size={size} drag gl_widget={type(gl).__name__ if gl else None}")

        def s_drag():
            grabbable, subs = grab_check(view, gl, sk, last_geo, 2)
            frames, moved = simulate_drag(view, gl, sk, last_geo, 2)
            out = summarize(frames)
            out.update({
                "gl_widget": type(gl).__name__ if gl else None,
                "moved_dist": moved,
                "measurement_ok": moved > 1.0,   # the tip actually swept -> frames are real
                "grabbable": grabbable,          # synthetic click grabbed the vertex (interactive-reachability)
                "grab_subs": subs,
            })
            return out

        record("drag", s_drag)

        try:
            Gui.activeDocument().resetEdit(); pump(1)
        except Exception:
            pass
        App.closeDocument(doc.Name); pump(1)

    return res


def main():
    # provenance: prove which binaries actually ran
    try:
        sketcher_file = Sketcher.__file__
    except Exception:
        sketcher_file = "?"
    out = {
        "label": LABEL,
        "freecad_version": ".".join(App.Version()[0:3]) + " (" + App.Version()[3] + ")"
        if hasattr(App, "Version") else "?",
        "sketcher_pyd": sketcher_file,
        "qt": _QT,
        "sizes": SIZES,
        "scenarios": SCENARIOS,
        "results": [],
    }

    for size in SIZES:
        try:
            out["results"].append(run_size(size))
        except Exception as e:
            out["results"].append({"size": size, "error": f"{e}\n{traceback.format_exc()}"})
            stage(f"size={size} ERROR {e}")

    return out


def write_and_exit(out):
    if OUT:
        with open(OUT, "w") as f:
            json.dump(out, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        stage(f"wrote {OUT}")
    else:
        App.Console.PrintMessage(json.dumps(out, indent=2) + "\n")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def run_deferred():
    """Executed by a 0-delay timer *after* the Qt event loop is live, so all
    GUI operations (setEdit, redraw, synthetic events) have a running loop."""
    stage("run_deferred start")
    out = {"label": LABEL, "note": "no-results-yet"}
    try:
        out = main()
    except Exception as e:
        out = {"label": LABEL, "fatal": f"{e}\n{traceback.format_exc()}"}
        stage(f"FATAL {e}")
    write_and_exit(out)


def watchdog():
    stage("WATCHDOG fired — forcing exit")
    partial = {"label": LABEL, "watchdog_timeout": WATCHDOG_SEC}
    write_and_exit(partial)


# Reset progress log for this run.
if PROGRESS:
    try:
        open(PROGRESS, "w").close()
    except Exception:
        pass
stage(f"module loaded (Qt={_QT}); scheduling deferred run")

# Defer the benchmark until the event loop is spinning; arm a watchdog so a
# hang can never wedge the harness forever.
QtCore.QTimer.singleShot(800, run_deferred)
QtCore.QTimer.singleShot(WATCHDOG_SEC * 1000, watchdog)
