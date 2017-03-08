#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
custom interactor style for terrain
'''

import vtk
import math
import numpy as np
 
class TerrainInteractorStyle(vtk.vtkInteractorStyle):
    def __init__(self):
        self.MotionFactor = 10.0
        self.MouseWheelMotionFactor = 1.0
        self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.leftButtonReleaseEvent)
        self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
        self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        self.AddObserver("MiddleButtonPressEvent", self.middleButtonPressEvent)
        self.AddObserver("MiddleButtonReleaseEvent", self.middleButtonReleaseEvent)
        self.AddObserver("MouseWheelForwardEvent", self.mouseWheelForwardEvent)
        self.AddObserver("MouseWheelBackwardEvent", self.mouseWheelBackwardEvent)

    def mouseMoveEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()

        state = self.GetState()

        if state == 1:      # VTKIS_ROTATE
            self.FindPokedRenderer(x, y)
            self.Rotate()
            self.InvokeEvent(vtk.vtkCommand.InteractionEvent)
        elif state == 2:    # VTKIS_PAN
            self.FindPokedRenderer(x, y)
            self.Pan()
            self.InvokeEvent(vtk.vtkCommand.InteractionEvent)
        elif state == 4:    # VTKIS_DOLLY
            self.FindPokedRenderer(x, y)
            self.Dolly()
            self.InvokeEvent(vtk.vtkCommand.InteractionEvent)

    def leftButtonPressEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())
        if self.GetInteractor().GetShiftKey():
            self.StartPan()
        else:
            self.StartRotate()

    def leftButtonReleaseEvent(self, obj, event):
        state = self.GetState()

        if state == 1:
            self.EndRotate()
            if self.GetInteractor():
                self.ReleaseFocus()
        elif state == 2:
            self.EndPan()
            if self.GetInteractor():
                self.ReleaseFocus()
 
    def middleButtonPressEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())
        self.StartPan()
 
    def middleButtonReleaseEvent(self, obj, event):
        state = self.GetState()

        if state == 2:
            self.EndPan()
            if self.GetInteractor():
                self.ReleaseFocus()

    def rightButtonPressEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())
        self.StartDolly()
 
    def rightButtonReleaseEvent(self, obj, event):
        state = self.GetState()

        if state == 4:
            self.EndDolly()
            if self.GetInteractor():
                self.ReleaseFocus()

    def mouseWheelForwardEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())

        self.StartDolly()
        factor = 10*0.2*self.MouseWheelMotionFactor
        self.DollyWheel(math.pow(1.1, factor))
        self.EndDolly()
        self.ReleaseFocus()

    def mouseWheelBackwardEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())

        self.StartDolly()
        factor = 10*-0.2*self.MouseWheelMotionFactor
        self.DollyWheel(math.pow(1.1, factor))
        self.EndDolly()
        self.ReleaseFocus()

    def Rotate(self):
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        x, y = rwi.GetEventPosition()
        x_last, y_last = rwi.GetLastEventPosition()

        dx = -(x-x_last)
        dy = -(y-y_last)

        size = self.GetCurrentRenderer().GetRenderWindow().GetSize()

        a = dx/float(size[0]) * 180.0
        e = dy/float(size[1]) * 180.0

        if rwi.GetControlKey():
            if abs(dx) >= abs(dy):
                e = 0.0
            else:
                a = 0.0

        camera = self.GetCurrentRenderer().GetActiveCamera()
        camera.Azimuth(a)

        dop = np.asarray(camera.GetDirectionOfProjection())
        dop /= np.linalg.norm(dop)
        vup = np.asarray(camera.GetViewUp())
        vup /= np.linalg.norm(vup)

        angle = np.degrees(math.acos(np.dot(dop, vup)))

        if (angle + e) > 177.4 or (angle + e) < 2.6:
            e = 0.0

        camera.Elevation(e)

        if self.GetAutoAdjustCameraClippingRange():
            self.GetCurrentRenderer().ResetCameraClippingRange()

        rwi.Render()

    def Pan(self):
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        camera = self.GetCurrentRenderer().GetActiveCamera()
        pos = list(camera.GetPosition())
        fp = list(camera.GetFocalPoint())

        focalPoint = [0,0,0]
        self.ComputeWorldToDisplay(self.GetCurrentRenderer(), fp[0], fp[1], fp[2], focalPoint)

        x, y = rwi.GetEventPosition()
        x_last, y_last = rwi.GetLastEventPosition()

        if rwi.GetControlKey():
            mouseDeltaX = x - x_last
            mouseDeltaY = y - y_last
            if abs(mouseDeltaX) >= abs(mouseDeltaY):
                y = y_last
            else:
                x = x_last

        p1 = [0,0,0,0]
        p2 = [0,0,0,0]
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x, y, focalPoint[2], p1)
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x_last, y_last, focalPoint[2], p2)

        v = [0,0,0]
        for i in xrange(3):
            v[i] = p2[i] - p1[i]
            pos[i] += v[i]
            fp[i] += v[i]

        camera.SetPosition(pos)
        camera.SetFocalPoint(fp)

        if rwi.GetLightFollowCamera():
            self.GetCurrentRenderer().UpdateLightsGeometryToFollowCamera()

        rwi.Render()

    def Dolly(self):
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        x, y = rwi.GetEventPosition()
        x_last, y_last = rwi.GetLastEventPosition()

        camera = self.GetCurrentRenderer().GetActiveCamera()
        center = self.GetCurrentRenderer().GetCenter()

        dy = y - y_last
        dyf = self.MotionFactor*dy/center[1]
        zoomFactor = math.pow(1.1, dyf)

        if camera.GetParallelProjection():
            camera.SetParallelScale(camera.GetParallelScale()/zoomFactor)
        else:
            camera.Dolly(zoomFactor)
            if self.GetAutoAdjustCameraClippingRange():
                self.GetCurrentRenderer().ResetCameraClippingRange()

        if rwi.GetLightFollowCamera():
            self.GetCurrentRenderer().UpdateLightsGeometryToFollowCamera()

        rwi.Render()

    def DollyWheel(self, factor):
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        camera = self.GetCurrentRenderer().GetActiveCamera()
        center = self.GetCurrentRenderer().GetCenter()

        if camera.GetParallelProjection():
            camera.SetParallelScale(camera.GetParallelScale()/factor)
        else:
            camera.Dolly(factor)
            if self.GetAutoAdjustCameraClippingRange():
                self.GetCurrentRenderer().ResetCameraClippingRange()

        if rwi.GetLightFollowCamera():
            self.GetCurrentRenderer().UpdateLightsGeometryToFollowCamera()

        rwi.Render()
