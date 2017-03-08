#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
custom interactor style for top-down
'''

import vtk
import math
import numpy as np
 
class TopDownInteractorStyle(vtk.vtkInteractorStyle):
    def __init__(self):
        self.MotionFactor = 10.0
        self.MouseWheelMotionFactor = 1.0
        self.MinHeight = 5.0
        self.MaxHeight = 1e6
        self.MinParallelScale = 0.5
        self.MaxParallelScale = 2500
        self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.leftButtonReleaseEvent)
        self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
        self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        self.AddObserver("MiddleButtonPressEvent", self.middleButtonPressEvent)
        self.AddObserver("MiddleButtonReleaseEvent", self.middleButtonReleaseEvent)
        self.AddObserver("MouseWheelForwardEvent", self.mouseWheelForwardEvent)
        self.AddObserver("MouseWheelBackwardEvent", self.mouseWheelBackwardEvent)
        self.AddObserver("KeyPressEvent", self.keyPressEvent)
        self.AddObserver("KeyReleaseEvent", self.keyReleaseEvent)
        self.coordText = vtk.vtkTextActor()
        self.coordText.GetTextProperty().SetColor(255/255.0,25/255.0,15/255.0)
        self.coordText.GetTextProperty().BoldOn()
        self.ctrlKey = False

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

        if self.ctrlKey:
            self.ShowCoords()

    def leftButtonPressEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return
        self.GrabFocus(self.OnMouseMove())
        if self.GetInteractor().GetShiftKey():
            self.StartRotate()
        else:
            self.StartPan()

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

    def keyPressEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        if self.GetInteractor().GetControlKey():
            if not self.ctrlKey:
                self.ctrlKey = True
                self.ShowCoords()
                self.GetCurrentRenderer().AddActor(self.coordText)
                rwi.Render()

    def keyReleaseEvent(self, obj, event):
        x, y = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(x, y)
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()
        if self.ctrlKey:
            if not self.GetInteractor().GetControlKey():
                self.ctrlKey = False
                self.GetCurrentRenderer().RemoveActor(self.coordText)
                rwi.Render()

    def Rotate(self):
        if self.GetCurrentRenderer() == None:
            return

        rwi = self.GetInteractor()

        x, y = rwi.GetEventPosition()
        x_last, y_last = rwi.GetLastEventPosition()

        dx = -(x-x_last)
        dy = -(y-y_last)

        size = self.GetCurrentRenderer().GetRenderWindow().GetSize()

        if x/float(size[0]) < 0.5 and y/float(size[1]) > 0.5:
            quad = 0
        elif x/float(size[0]) >= 0.5 and y/float(size[1]) > 0.5:
            quad = 1
        elif x/float(size[0]) >= 0.5 and y/float(size[1]) <= 0.5:
            quad = 2
        else:
            quad = 3

        a = dx/float(size[0])*180
        b = dy/float(size[1])*180
        mag = math.hypot(a,b)

        if quad == 0:
            if abs(dx) > abs(dy):
                if dx > 0:
                    mag *= -1
            elif abs(dy) > abs(dx):
                if dy > 0:
                    mag *= -1
            else:
                mag = 0
        elif quad == 1:
            if abs(dx) > abs(dy):
                if dx > 0:
                    mag *= -1.0
            elif abs(dy) > abs(dx):
                if dy < 0:
                    mag *= -1.0
            else:
                mag = 0.0
        elif quad == 2:
            if abs(dx) > abs(dy):
                if dx < 0:
                    mag *= -1.0
            elif abs(dy) > abs(dx):
                if dy < 0:
                    mag *= -1.0
            else:
                mag = 0
        elif quad == 3:
            if abs(dx) > abs(dy):
                if dx < 0:
                    mag *= -1.0
            elif abs(dy) > abs(dx):
                if dy > 0:
                    mag *= -1.0
            else:
                mag = 0.0

        camera = self.GetCurrentRenderer().GetActiveCamera()
        camera.Roll(-mag)

        if self.GetAutoAdjustCameraClippingRange():
            self.GetCurrentRenderer().ResetCameraClippingRange()

        rwi.Render()

    def ShowCoords(self):
        ren = self.GetCurrentRenderer()
        if ren == None:
            return

        rwi = self.GetInteractor()

        x, y = rwi.GetEventPosition()

        camera = self.GetCurrentRenderer().GetActiveCamera()

        size = self.GetCurrentRenderer().GetRenderWindow().GetSize()

        p = [0,0,0,0]
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x, y, 0, p)

        coord_str = '(' + '{0:.2f}'.format(p[0]) + ', ' + '{0:.2f}'.format(p[1]) + ')'
        self.coordText.SetInput(coord_str)
        self.coordText.SetPosition(x,y)

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

        p1 = [0,0,0,0]
        p2 = [0,0,0,0]
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x, y, focalPoint[2], p1)
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x_last, y_last, focalPoint[2], p2)
        p1[2] = p2[2] = 0.0
        p1[3] = p2[3] = 1.0

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
            if zoomFactor > 1:
                if camera.GetParallelScale() >= self.MinParallelScale:
                    camera.SetParallelScale(camera.GetParallelScale()/zoomFactor)
            else:
                if camera.GetParallelScale() <= self.MaxParallelScale:
                    camera.SetParallelScale(camera.GetParallelScale()/zoomFactor)
        else:
            if zoomFactor > 1:
                if camera.GetPosition()[2] >= self.MinHeight:
                    camera.Dolly(zoomFactor)
            else:
                if camera.GetPosition()[2] <= self.MaxHeight:
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

        x, y = rwi.GetEventPosition()

        camera = self.GetCurrentRenderer().GetActiveCamera()
        center = self.GetCurrentRenderer().GetCenter()

        xy_world = [0,0,0,0]
        self.ComputeDisplayToWorld(self.GetCurrentRenderer(), x, y, 0, xy_world)
        xy_camera = camera.GetPosition()

        x_delta = xy_world[0] - xy_camera[0]
        y_delta = xy_world[1] - xy_camera[1]
        
        if camera.GetParallelProjection():
            if factor > 1:
                if camera.GetParallelScale() >= self.MinParallelScale:
                    camera.SetParallelScale(camera.GetParallelScale()/factor)
            else:
                if camera.GetParallelScale() <= self.MaxParallelScale:
                    camera.SetParallelScale(camera.GetParallelScale()/factor)
        else:
            if factor > 1:
                if camera.GetPosition()[2] >= self.MinHeight:
                    camera.Dolly(factor)
            else:
                if camera.GetPosition()[2] <= self.MaxHeight:
                    camera.Dolly(factor)

            # new_center = [0,0,0]
            # new_center[0] = xy_camera[0] + x_delta - x_delta/factor
            # new_center[1] = xy_camera[1] + y_delta - y_delta/factor
            # new_center[2] = camera.GetPosition()[2]

            # camera.SetPosition(new_center)
            # new_center[2] = 0
            # camera.SetFocalPoint(new_center)
            # print camera.GetPosition(), camera.GetFocalPoint()

            if self.GetAutoAdjustCameraClippingRange():
                self.GetCurrentRenderer().ResetCameraClippingRange()

        if rwi.GetLightFollowCamera():
            self.GetCurrentRenderer().UpdateLightsGeometryToFollowCamera()

        rwi.Render()
