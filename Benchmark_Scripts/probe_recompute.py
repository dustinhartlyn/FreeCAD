import time, FreeCAD as App, Part, Sketcher
for NUM in [500]:
    doc=App.newDocument("R")
    sk=doc.addObject("Sketcher::SketchObject","Sketch")
    sk.addGeometry(Part.LineSegment(App.Vector(0,0,0),App.Vector(10,10,0)),False)
    sk.addConstraint(Sketcher.Constraint("Coincident",0,1,-1,1))
    drv=sk.addConstraint(Sketcher.Constraint("Distance",0,10.0))
    for i in range(1,NUM):
        sk.addGeometry(Part.LineSegment(App.Vector(i*10,0,0),App.Vector((i+1)*10,10,0)),False)
        sk.addConstraint(Sketcher.Constraint("Coincident",i-1,2,i,1))
        sk.addConstraint(Sketcher.Constraint("Distance",i,10.0))
    doc.recompute()
    for w in range(2):
        sk.setDatum(drv,App.Units.Quantity(f"{10.0+w*0.1} mm")); doc.recompute()
    t0=time.perf_counter()
    for i in range(8):
        sk.setDatum(drv,App.Units.Quantity(f"{10.0+i*0.5} mm")); doc.recompute()
    t1=time.perf_counter()
    App.Console.PrintMessage(f"[RECOMPUTE] {NUM} nodes | recompute()={(t1-t0)/8*1000:.1f} ms\n")
    App.closeDocument(doc.Name)
