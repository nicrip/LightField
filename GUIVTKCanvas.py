#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
main UI VTK canvas
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt4 import QtCore, QtGui

''' custom libs '''
from TerrainInteractorStyle import TerrainInteractorStyle
from TopDownInteractorStyle import TopDownInteractorStyle
import Primitives
import Billboards
import TransformUtils
import GUIPrimitiveDialog
import GUITransformDialog
import GUIOffsetOrientationDialog
import GUIActorPropertiesDock

''' thirdparty libs '''
from thirdparty import transformations

DIR_TYPE = 'directory'
ACTOR_TYPE = 'actor'
BILLBOARD_TYPE = 'billboard'
AXES_TYPE = 'axes'

class VTKCanvas(QtGui.QFrame):
    ''' the Qt4 frame that holds the main VTK canvas '''

    def __init__(self, parent, Qt4GUI):
        super(VTKCanvas,self).__init__(parent)
        # dictionary to keep track of tree widget items that correspond to actors/icons - used to handle vtk->Qt4 scene changes
        self.actors_to_tree_widget_items = {}

        # Qt4 GUI that this canvas is associated with
        self.Qt4GUI = Qt4GUI

        # camera perspective type check
        self.camera_perspective = False

        # setup the vtk renderers - one for objects, one for overlay icons/text (which always renders on top)
        self.vtk_renderer = vtk.vtkRenderer()

        # setup the vtk interactor
        self.vtk_interactor = QVTKRenderWindowInteractor(self)

        # setup the vtk render window
        self.vtk_render_window = self.vtk_interactor.GetRenderWindow()
        self.vtk_render_window.AddRenderer(self.vtk_renderer)
        self.vtk_render_window.SetInteractor(self.vtk_interactor)

        #setup the layout of the vtk interactor in Qt
        self.layout = QtGui.QHBoxLayout()
        self.layout.addWidget(self.vtk_interactor)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)

    def setupTimerCallback(self, targetFPS=30):
        # calling this function will setup a timer and callback to continuously render the scene at the target FPS
        self.vtk_interactor.AddObserver('TimerEvent', self.requestUpdate)
        self.vtk_interactor.CreateRepeatingTimer(long(targetFPS))

    def requestUpdate(self, obj, event):
        # request a render update of the scene
        self.vtk_render_window.Render()

    def start(self):
        # setup the vtk background - as default, set to light
        self.vtk_renderer.GradientBackgroundOn()
        self.setBackgroundLight()

        # setup the default camera
        self.camera_perspective = False
        self.defaultPerspectiveCamera()

        # setup a default light kit (simple sun illumination + camera spotlight)
        self.light_kit = vtk.vtkLightKit()
        self.light_kit.MaintainLuminanceOn()
        self.light_kit.AddLightsToRenderer(self.vtk_renderer)

        # add the orientation axes to the bottom left of the canvas
        self.Qt4GUI.addOrientationAxes()
        
        # setup the default base grid - 1x1km, 10m squares
        self.Qt4GUI.addGrid(['grids', '1 km x 1 km, 10 m'], 1000, 10)

        # startup vtk
        self.vtk_interactor.Initialize()
        self.vtk_interactor.Start()

    def resetCamera(self):
        if self.camera_perspective:
            self.camera_perspective = False
            self.defaultPerspectiveCamera()
        else:
            self.camera_perspective = True
            self.defaultTopDownCamera()

    def defaultPerspectiveCamera(self):
        self.Qt4GUI.camera_perspective_signal.emit(True)
        if not self.camera_perspective:
            self.vtk_renderer.SetActiveCamera(vtk.vtkCamera())
            self.vtk_interactor.SetInteractorStyle(TerrainInteractorStyle())
            self.vtk_renderer.GetActiveCamera().SetViewUp(0,0,1)
            self.vtk_renderer.GetActiveCamera().SetPosition(0,1,10)
            self.vtk_renderer.GetActiveCamera().SetFocalPoint(0,0,0)
            self.vtk_renderer.GetActiveCamera().Azimuth(135)
            self.vtk_renderer.GetActiveCamera().Elevation(86)
            self.vtk_renderer.GetActiveCamera().Dolly(0.4)
            self.vtk_renderer.GetActiveCamera().Zoom(1)
            self.vtk_renderer.ResetCameraClippingRange()
            self.camera_perspective = True
            self.vtk_interactor.Render()

    def perspectiveCamera(self):
        self.Qt4GUI.camera_perspective_signal.emit(True)
        if not self.camera_perspective:
            curr_fp = self.vtk_renderer.GetActiveCamera().GetFocalPoint()
            curr_pos = [curr_fp[0], curr_fp[1]+1, 10]
            curr_fp = [curr_fp[0], curr_fp[1], 0]
            self.vtk_renderer.SetActiveCamera(vtk.vtkCamera())
            self.vtk_interactor.SetInteractorStyle(TerrainInteractorStyle())
            self.vtk_renderer.GetActiveCamera().SetViewUp(0,0,1)
            self.vtk_renderer.GetActiveCamera().SetPosition(curr_pos)
            self.vtk_renderer.GetActiveCamera().SetFocalPoint(curr_fp)
            self.vtk_renderer.GetActiveCamera().Azimuth(135)
            self.vtk_renderer.GetActiveCamera().Elevation(86)
            self.vtk_renderer.GetActiveCamera().Dolly(0.4)
            self.vtk_renderer.GetActiveCamera().Zoom(1)
            self.vtk_renderer.ResetCameraClippingRange()
            self.camera_perspective = True
            self.vtk_interactor.Render()

    def defaultTopDownCamera(self):
        self.Qt4GUI.camera_perspective_signal.emit(False)
        if self.camera_perspective:
            self.vtk_renderer.SetActiveCamera(vtk.vtkCamera())
            self.vtk_interactor.SetInteractorStyle(TopDownInteractorStyle())
            self.vtk_renderer.GetActiveCamera().SetViewUp(0,1,0)
            self.vtk_renderer.GetActiveCamera().ParallelProjectionOn()
            self.vtk_renderer.GetActiveCamera().SetParallelScale(1000)
            self.vtk_renderer.GetActiveCamera().SetPosition(0,0,1e4)
            self.vtk_renderer.GetActiveCamera().SetFocalPoint(0,0,0)
            self.vtk_renderer.GetActiveCamera().Elevation(0)
            self.vtk_renderer.GetActiveCamera().Azimuth(0)
            self.vtk_renderer.GetActiveCamera().Dolly(0.4)
            self.vtk_renderer.GetActiveCamera().Zoom(100)
            self.vtk_renderer.ResetCameraClippingRange()
            self.camera_perspective = False
            self.vtk_interactor.Render()

    def topDownCamera(self):
        self.Qt4GUI.camera_perspective_signal.emit(False)
        if self.camera_perspective:
            curr_fp = self.vtk_renderer.GetActiveCamera().GetFocalPoint()
            curr_pos = [curr_fp[0], curr_fp[1], 1e4]
            curr_fp = [curr_fp[0], curr_fp[1], 0]
            self.vtk_renderer.SetActiveCamera(vtk.vtkCamera())
            self.vtk_interactor.SetInteractorStyle(TopDownInteractorStyle())
            self.vtk_renderer.GetActiveCamera().SetViewUp(0,1,0)
            self.vtk_renderer.GetActiveCamera().ParallelProjectionOn()
            self.vtk_renderer.GetActiveCamera().SetParallelScale(1000)
            self.vtk_renderer.GetActiveCamera().SetPosition(curr_pos)
            self.vtk_renderer.GetActiveCamera().SetFocalPoint(curr_fp)
            self.vtk_renderer.GetActiveCamera().Elevation(0)
            self.vtk_renderer.GetActiveCamera().Azimuth(0)
            self.vtk_renderer.GetActiveCamera().Dolly(0.4)
            self.vtk_renderer.GetActiveCamera().Zoom(100)
            self.vtk_renderer.ResetCameraClippingRange()
            self.camera_perspective = False
            self.vtk_interactor.Render()
        
    def setBackgroundLight(self):
        self.Qt4GUI.background_light_signal.emit(True)
        self.vtk_renderer.SetBackground2(220.0/255,225.0/255,235.0/255)
        self.vtk_renderer.SetBackground(105.0/255,135.0/255,155.0/255)
        self.vtk_interactor.Render()

    def setBackgroundDark(self):
        self.Qt4GUI.background_light_signal.emit(False)
        self.vtk_renderer.SetBackground(0.0/255,10.0/255,15.0/255)
        self.vtk_renderer.SetBackground2(60.0/255,80.0/255,110.0/255)
        self.vtk_interactor.Render()

    def addActorFrameAxes(self, tree_widget):
        frame_axes = Primitives.Axes()
        frame_axes.AxisLabelsOff()
        frame_axes.VisibilityOff()
        self.vtk_renderer.AddActor(frame_axes)
        self.vtk_interactor.Render()
        return frame_axes

    def removeActorFrameAxes(self, actor):
        self.vtk_renderer.RemoveActor(actor)
        self.vtk_interactor.Render()

    def addActor(self, tree_widget, actor):
        self.actors_to_tree_widget_items[actor] = tree_widget
        self.vtk_renderer.AddActor(actor)
        self.vtk_interactor.Render()

    def removeActor(self, actor):
        del self.actors_to_tree_widget_items[actor]
        self.vtk_renderer.RemoveActor(actor)
        self.vtk_interactor.Render()

    def replaceActor(self, remove_actor, tree_widget, actor):
        self.removeActor(remove_actor)
        self.addActor(tree_widget, actor)

    def setActorVisibility(self, actor, visible):
        if visible:
            actor.VisibilityOn()
        else:
            actor.VisibilityOff()
        self.vtk_interactor.Render()

    def setActorScale(self, actor, scale):
        if actor.GetClassName() == "vtkAxesActor":
            actor.SetTotalLength(scale, scale, scale)
        else:
            if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
                # billboards have no scaling
                return
            actor.SetScale(scale)
        self.vtk_interactor.Render()

    def getActorScale(self, actor):
        if actor.GetClassName() == "vtkAxesActor":
            return actor.GetTotalLength()
        else:
            if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
                # billboards have no scaling
                return [1,1,1]
            return actor.GetScale()

    def setActorOpacity(self, actor, opacity):
        actor.GetProperty().SetOpacity(opacity)
        self.vtk_interactor.Render()

    def getActorOpacity(self, actor):
        return actor.GetProperty().GetOpacity()

    def setActorPointSize(self, actor, size):
        actor.GetProperty().SetPointSize(size)
        self.vtk_interactor.Render()

    def getActorPointSize(self, actor):
        return actor.GetProperty().GetPointSize()

    def setActorLineWidth(self, actor, width):
        actor.GetProperty().SetLineWidth(width)
        self.vtk_interactor.Render()

    def getActorLineWidth(self, actor):
        return actor.GetProperty().GetLineWidth()

    def setActorToSurface(self, actor):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards have no render mode
            return
        actor.GetProperty().EdgeVisibilityOff()
        actor.GetProperty().SetRepresentationToSurface()
        self.vtk_interactor.Render()

    def setActorToWireframe(self, actor):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards have no render mode
            return
        actor.GetProperty().EdgeVisibilityOff()
        actor.GetProperty().SetRepresentationToWireframe()
        self.vtk_interactor.Render()

    def setActorToSurfaceEdges(self, actor):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards have no render mode
            return
        actor.GetProperty().EdgeVisibilityOn()
        actor.GetProperty().SetRepresentationToSurface()
        self.vtk_interactor.Render()

    def setActorToPoints(self, actor):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards have no render mode
            return
        actor.GetProperty().EdgeVisibilityOff()
        actor.GetProperty().SetRepresentationToPoints()
        self.vtk_interactor.Render()

    def getActorRenderMode(self, actor):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards have no render mode
            return None, None
        edge = actor.GetProperty().GetEdgeVisibility()
        mode = actor.GetProperty().GetRepresentation()
        return edge, mode

    def getActorColor(self, actor):
        return actor.GetProperty().GetColor()

    def setActorColor(self, actor, r, g, b):
        actor.GetProperty().SetColor(r,g,b)
        self.vtk_interactor.Render()

    def setActorTexture(self, actor, image_path):
        if actor.__class__ == Billboards.TextBillboard:
            # text billboards have no texture
            return
        if image_path[-4:].lower() == '.jpg' or image_path[-4:].lower() == '.jpeg':
            reader = vtk.vtkJPEGReader()
            reader.SetFileName(image_path)
            reader.Update()
        elif image_path[-4:].lower() == '.png':
            reader = vtk.vtkPNGReader()
            reader.SetFileName(image_path)
            reader.Update()
        texture = vtk.vtkTexture()
        texture.RepeatOn()
        texture.SetInputConnection(reader.GetOutputPort())

        actor.SetTexture(texture)

        # polydata = actor.GetMapper().GetInput()
        # pointdata = polydata.GetPointData()
        # celldata = polydata.GetCellData()
        # numtuples = celldata.GetNumberOfTuples()
        # numpolygons = polydata.GetNumberOfPolys()

        self.vtk_interactor.Render()

    def removeActorTexture(self, actor):
        actor.SetTexture(None)
        self.vtk_interactor.Render()

    def setActorOffset(self, actor, x_offset, y_offset, z_offset):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards are unaffected by offset changes
            pass
        else:
            actor.SetPosition(x_offset,y_offset,z_offset)
            self.vtk_interactor.Render()

    def setActorOrientation(self, actor, roll, pitch, yaw):
        actor.SetOrientation(roll, pitch, yaw)
        self.vtk_interactor.Render()