#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
LightField API object
'''

''' standard libs '''
import math
import warnings
import numpy as np
from contextlib import contextmanager

''' VTK and PyQt4 '''
import vtk
from PyQt4 import QtCore, QtGui, uic

''' custom libs '''
from GUIMain import MainApp
import Primitives

@contextmanager
def wait_signal(signal_origin, signal_name, return_status_list, timeout=10000):
    """Block loop until signal emitted, or timeout (ms) elapses."""
    return_status_list[0] = 'timeout'
    def return_status(status):
        return_status_list[0] = status

    loop = QtCore.QEventLoop()
    QtCore.QObject.connect(signal_origin, QtCore.SIGNAL(signal_name), loop.quit)
    QtCore.QObject.connect(signal_origin, QtCore.SIGNAL(signal_name), return_status)

    yield

    if timeout is not None:
        QtCore.QTimer.singleShot(timeout, loop.quit)
    loop.exec_()

class LightFieldAPI(object):
    ''' object to setup and interface with the LightField Visualizer '''

    def __init__(self):
        # Recompile ui
        with open("LightFieldVisualizer.ui") as ui_file:
            with open("LightFieldVisualizer_ui.py","w") as py_ui_file:
                uic.compileUi(ui_file,py_ui_file)
        self.app = QtGui.QApplication([])
        self.main_window = MainApp()
        self.main_window.show()

    def addActor(self, levellist, actor, actortype):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'addActorStatus', return_status_list):
            self.main_window.add_actor_signal.emit(levellist, actor, actortype)
        return_status = return_status_list[0]
        return return_status

    def setActor(self, levellist, actor, actortype):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorStatus', return_status_list):
            self.main_window.set_actor_signal.emit(levellist, actor, actortype)
        return_status = return_status_list[0]
        return return_status

    def addPolyData(self, levellist, vtkpolydata, actortype):
        mapper = vtk.vtkDataSetMapper()
        mapper.SetInput(vtkpolydata)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return self.addActor(levellist, actor, actortype)

    def addGrid(self, levellist, fulllength, celllength):
        actor = Primitives.Grid(fulllength, celllength)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_GRID)

    def addAxes(self, levellist):
        actor = Primitives.Axes()
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_AXES)

    def addArrow(self, levellist, resolution):
        actor = Primitives.Arrow(resolution)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_ARROW)

    def addBox(self, levellist, xlength, ylength, zlength):
        actor = Primitives.Box(xlength, ylength, zlength)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_BOX)

    def addSphere(self, levellist, radius, thetaresolution, phiresolution):
        actor = Primitives.Sphere(radius, thetaresolution, phiresolution)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_SPHERE)

    def addCylinder(self, levellist, radius, height, resolution):
        actor = Primitives.Cylinder(radius, height, resolution)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_CYLINDER)

    def addEllipsoid(self, levellist, xradius, yradius, zradius):
        actor = Primitives.Ellipsoid(xradius, yradius, zradius)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_ELLIPSOID)

    def addCone(self, levellist, radius, height, resolution):
        actor = Primitives.Cone(radius, height, resolution)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_CONE)

    def addTorus(self, levellist, ringradius, crosssectionradius):
        actor = Primitives.Torus(ringradius, crosssectionradius)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_TORUS)

    def addMeshFile(self, levellist, filepath, texturepath=None):
        actor = Primitives.Model(filepath, texturepath)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_MODEL)

    def addTriangleStrip(self, levellist, vertices, colors=None):
        actor = Primitives.TriangleStrip(vertices, colors)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_TRIANGLE_STRIP)

    def addLineStrip(self, levellist, vertices, colors=None):
        actor = Primitives.LineStrip(vertices, colors)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_LINE_STRIP)

    def addPointCloud(self, levellist, points, colors=None):
        actor = Primitives.PointCloud(points, colors)
        return self.addActor(levellist, actor, Primitives.PRIMITIVE_POINT_CLOUD)

    def setPointCloud(self, levellist, points, colors=None):
        actor = Primitives.PointCloud(points, colors)
        return self.setActor(levellist, actor, Primitives.PRIMITIVE_POINT_CLOUD)

    def addDirectory(self, levellist):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'addDirectoryStatus', return_status_list):
            self.main_window.add_dir_signal.emit(levellist)
        return_status = return_status_list[0]
        return return_status

    def removeActor(self, levellist):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'removeActorStatus', return_status_list):
            self.main_window.remove_actor_signal.emit(levellist)
        return_status = return_status_list[0]
        return return_status

    def removeDirectory(self, levellist):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'removeDirectoryStatus', return_status_list):
            self.main_window.remove_dir_signal.emit(levellist)
        return_status = return_status_list[0]
        return return_status

    def setActorOffsetOrientation(self, levellist, offset, orientationeuler):
        if len(offset) != 3 and len(orientationeuler) !=3:
            warn_str = "setActorOffsetOrientation failed: offset length != 3: " + str(offset) + " or Euler orientation length != 3: " + str(orientationeuler)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorOffsetOrientationStatus', return_status_list):
            self.main_window.set_actor_offset_orientation_signal.emit(levellist, offset, orientationeuler)
        return_status = return_status_list[0]
        return return_status

    def setActorColor(self, levellist, rgb):
        if len(rgb) != 3:
            warn_str = "setActorColor failed: rgb color length != 3: " + str(rgb)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorColorStatus', return_status_list):
            self.main_window.set_actor_color_signal.emit(levellist, rgb)
        return_status = return_status_list[0]
        return return_status

    def setActorOpacity(self, levellist, opacity):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorOpacityStatus', return_status_list):
            self.main_window.set_actor_alpha_signal.emit(levellist, opacity)
        return_status = return_status_list[0]
        return return_status

    def setActorScale(self, levellist, scale):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorScaleStatus', return_status_list):
            self.main_window.set_actor_scale_signal.emit(levellist, scale)
        return_status = return_status_list[0]
        return return_status

    def setActorPointSize(self, levellist, pointsize):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorPointSizeStatus', return_status_list):
            self.main_window.set_actor_pointsize_signal.emit(levellist, pointsize)
        return_status = return_status_list[0]
        return return_status

    def setActorVisibility(self, levellist, visibility):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorVisibilityStatus', return_status_list):
            self.main_window.set_actor_visibility_signal.emit(levellist, visibility)
        return_status = return_status_list[0]
        return return_status

    def setActorLineWidth(self, levellist, linewidth):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorLineWidthStatus', return_status_list):
            self.main_window.set_actor_linewidth_signal.emit(levellist, linewidth)
        return_status = return_status_list[0]
        return return_status

    def setActorMode(self, levellist, mode):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorModeStatus', return_status_list):
            self.main_window.set_actor_mode_signal.emit(levellist, mode)
        return_status = return_status_list[0]
        return return_status

    def setActorTransform(self, levellist, translation, rotationeuler, order=None):
        if len(translation) != 3 and len(rotationeuler) !=3:
            warn_str = "setActorTransform failed: translation length != 3: " + str(translation) + " or Euler rotation length != 3: " + str(rotationeuler)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setActorTransformStatus', return_status_list):
            self.main_window.set_actor_transform_signal.emit(levellist, translation, rotationeuler, order)
        return_status = return_status_list[0]
        return return_status

    def applyActorTransform(self, levellist, translation, rotationeuler, order=None):
        if len(translation) != 3 and len(rotationeuler) !=3:
            warn_str = "applyActorTransform failed: translation length != 3: " + str(translation) + " or Euler rotation length != 3: " + str(rotationeuler)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'applyActorTransformStatus', return_status_list):
            self.main_window.apply_actor_transform_signal.emit(levellist, translation, rotationeuler, order)
        return_status = return_status_list[0]
        return return_status

    def resetActorTransform(self, levellist):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'resetActorTransformStatus', return_status_list):
            self.main_window.reset_actor_transform_signal.emit(levellist)
        return_status = return_status_list[0]
        return return_status

    def setDirectoryTransform(self, levellist, translation, rotationeuler, order=None):
        if len(translation) != 3 and len(rotationeuler) !=3:
            warn_str = "setDirectoryTransform failed: translation length != 3: " + str(translation) + " or Euler rotation length != 3: " + str(rotationeuler)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'setDirectoryTransformStatus', return_status_list):
            self.main_window.set_directory_transform_signal.emit(levellist, translation, rotationeuler, order)
        return_status = return_status_list[0]
        return return_status

    def applyDirectoryTransform(self, levellist, translation, rotationeuler, order=None):
        if len(translation) != 3 and len(rotationeuler) !=3:
            warn_str = "setDirectoryTransform failed: translation length != 3: " + str(translation) + " or Euler rotation length != 3: " + str(rotationeuler)
            warnings.warn(warn_str, RuntimeWarning)
            return
        return_status_list = ['request']
        with wait_signal(self.main_window, 'applyDirectoryTransformStatus', return_status_list):
            self.main_window.apply_directory_transform_signal.emit(levellist, translation, rotationeuler, order)
        return_status = return_status_list[0]
        return return_status

    def resetDirectoryTransform(self, levellist):
        return_status_list = ['request']
        with wait_signal(self.main_window, 'resetDirectoryTransformStatus', return_status_list):
            self.main_window.reset_directory_transform_signal.emit(levellist)
        return_status = return_status_list[0]
        return return_status

    def start(self, timer_update=False, timer_fps=30):
        self.main_window.start(timer_update, timer_fps)
        self.app.exec_()

if __name__ == "__main__":
    LFAPI = LightFieldAPI()
    LFAPI.start()