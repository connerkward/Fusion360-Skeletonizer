#Author-conner ward
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
from math import radians, cos, sin, sqrt
from .Modules.pytess import tesselator
from .Modules.pytess import pytess as tess

def run(context):
    def distance(x1, x2, y1, y2):
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    ui = None
    try:
        # boilerplate=============
        app = adsk.core.Application.get()
        ui  = app.userInterface

        root = app.activeProduct.rootComponent  # type: adsk.fusion.Component
        extrudes = root.features.extrudeFeatures
        cut = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        button_enum = adsk.core.MessageBoxButtonTypes

        # select face entity in fusion ui -> extract BRepFace from entity
        selectedEntity = ui.selectEntity("Select the rectangle", "Faces")
        selected_face = adsk.fusion.BRepFace.cast(selectedEntity.entity)
        # create sketch
        sketch = root.sketches.add(selected_face)
        sketch.name = "the face"
        # bind points
        points = sketch.sketchPoints
        # collect all lines in sketch
        lines = sketch.sketchCurves.sketchLines
        # project edges of selected face onto sketch
        projection = sketch.project(selectedEntity.entity)
        outer_curves = adsk.core.ObjectCollection.create()
        for curve in projection:
            if curve.classType() == adsk.fusion.SketchCircle.classType():
                pass
            else:
                outer_curves.add(curve)

        # INITIAL OFFSET===========================================
        # - get all connected sketch lines of projected edge, and offsets them 
        # direction point for edge offset -> get user input on edge offset -> create initial offset
        dirPoint = adsk.core.Point3D.create(-1, -1, -1)
        points.add(dirPoint)
        outer_edge_offset = float(ui.inputBox("Exterior Edge Offset", "", "5")[0]) # in mm
        offsetCurves = sketch.offset(outer_curves, dirPoint, outer_edge_offset/10) # divide by 10 for mm from cm
        
        # prompt user if offset is the right direction
        while True:
            response = ui.messageBox("Reverse offset direction?", " ", button_enum.YesNoButtonType)
            if response == 2: # 2 yes, 3 no
                outer_edge_offset = float(ui.inputBox("Exterior Edge Offset", "", f"{outer_edge_offset}")[0]) # in mm
                for curve in offsetCurves:
                    curve.deleteMe()
                outer_edge_offset = -outer_edge_offset
                offsetCurves = sketch.offset(outer_curves, dirPoint, outer_edge_offset/10) # divide by 10 for mm from cm
            else:
                break
        exit()
        # INTERNAL HOLES OFFSET ===========================================
        # - get all interior holes in part, and offsets them for interior circles, create offsets
        # TODO: change intersect lines to be only exterior lines
        # TODO: differntiate exterior touching circles from interior circles. for now assume all holes are exterior holes        
        circles = sketch.sketchCurves.sketchCircles
        circle_centers = list()
        print("number of circles: ", len(circles))
        for circle in circles:
            center_point = circle.centerSketchPoint.geometry # worldGeometry
            # search for distance to outer_curves
            # shoot rays in 16 directions 15 degree angles
            ray_lines = list()
            degree = 15 # 45, 30, 15
            for rdirection in [radians(i) for i in range(0,361,degree)]:
                # create Line2D and in rdirection of legth circle.radius * 5 (TODO: arbitrary)
                far_x = circle.centerSketchPoint.geometry.x + circle.radius * 5 * cos(rdirection)
                far_y = circle.centerSketchPoint.geometry.y + circle.radius* 5 * sin (rdirection)
                
                farvec = adsk.core.Vector3D.create(far_x, far_y, 0)
                ifline = adsk.core.InfiniteLine3D.create(center_point, farvec)
                
                farpoint = adsk.core.Point3D.create(far_x, far_y)
                line = adsk.core.Line3D.create(center_point, farpoint)
                ray_lines.append(line)
                # for visualization purposes
                # lines.addByTwoPoints(center_point, farpoint).isConstruction = True
            
            # # find intersection with outer_curves distances, get min
            intersection_points = list()
            intersection_distances = dict()

            for ray in ray_lines:
                # intersectWithCurve
                # ProfileCurve->Curve3D by ProfileCurve::geometry
                for curve in outer_curves:
                    for point in ray.intersectWithCurve(curve.geometry):
                        if point:
                            intersection_points.append(point) 
                            # print(f"computing distance: {center_point.x} {center_point.y} {point.x} {point.y}")
                            intersection_distances[f"{point.x}{point.y}"] = (point, distance(center_point.x, point.x, center_point.y, point.y))
            # find shortest distance between center and outer edge
            # print(sorted(intersection_distances.values(), key=lambda x: x[1]))
            minpoint = min([(point, distance) for point, distance in intersection_distances.values()], key=lambda x: x[1])
            # print("---->final min point: ", minpoint)
            points.add(minpoint[0])
            print(minpoint)
            lines.addByTwoPoints(minpoint[0], center_point).isConstruction = True
            interior_offset = minpoint[1] - circle.radius
            # create offset
            try:
                dirPoint = adsk.core.Point3D.create(0, 0, 0)

                offset = sketch.offset(sketch.findConnectedCurves(circle), dirPoint, interior_offset) # offset is in cm
                circle_centers.append((circle.centerSketchPoint.geometry.x, circle.centerSketchPoint.geometry.y))
            except:
                pass
        
        # INTERIOR HOLE REINFORCEMENT =========================================
        # TODO: add manual interior circle / exterior circle user selection
        # # triangulate from centers
        # print("circle centers:\n",circle_centers)
        # tris = tess.triangulate(circle_centers)
        # print(tris)
        # lines = sketch.sketchCurves.sketchLines

        # for tri in tris:
        #     sketchpoint1 = adsk.core.Point3D.create(tri[0][0], tri[0][1], 0)
        #     sketchpoint2 = adsk.core.Point3D.create(tri[1][0], tri[1][1], 0)
        #     sketchpoint3 = adsk.core.Point3D.create(tri[2][0], tri[2][1], 0)
        #     lines.addByTwoPoints(sketchpoint1, sketchpoint2)
        #     lines.addByTwoPoints(sketchpoint1, sketchpoint3)

        # SELECT AND FILLET INTERIOR PROFILES ================================
        selectedEntity = ui.selectEntity("Select the Profile", "Profiles")
        # print(selectedEntity.entity.profileLoops)
        for loop in selectedEntity.entity.profileLoops:
            for curve in loop.profileCurves:
                print(curve.sketchEntity)
            # TODO: fillet interior profiles
            # fillet = sketch.sketchCurves.sketchArcs.addFillet()
        
        
        # EXTRUDE INTERIORS =================================================
        # TODO: get depth to extrude to be 1/3 of part thickness
        # TODO: allow user to select multiple interiors after generating some by default
        outer_edge_offset = float(ui.inputBox("Depth to cut", "", "4")[0]) # in mm
        distance = adsk.core.ValueInput.createByReal(-outer_edge_offset/10)

        # REFERENCE: sketch profiles 0-2 are holes, 3 is outer line, 4 is interior offset
        extrude1 = extrudes.addSimple(selectedEntity.entity, distance, adsk.fusion.FeatureOperations.CutFeatureOperation)   


        # ============ NOTES ===============
        # sketch = root.sketches.add(root.xYConstructionPlane)
        # sketch.name = "simple"
        # circles = sketch.sketchCurves.sketchCircles
        # circle1 = circles.addByCenterRadius(adsk.core.Point3D.create(0,0,0),2)
        # prof = sketch.profiles.item(0)
        # distance = adsk.core.ValueInput.createByReal(5)
        # extrude1 = extrudes.addSimple(prof, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # body1 = extrude1.bodies.item(0)
        # body1.name = "simple"
        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
