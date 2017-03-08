#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
additional gui elements - the actor properties dock widget
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
import vtk
from PyQt4 import QtCore, QtGui, uic

class ActorPropertiesDock(QtGui.QDockWidget):
    ''' a dialog box for adding primitive actors '''
    def __init__(self, parent):
        QtGui.QDockWidget.__init__(self)
        self.parent = parent
        self.setWindowTitle('Actor Properties')
        self.setFeatures(QtGui.QDockWidget.DockWidgetFloatable | QtGui.QDockWidget.DockWidgetMovable)
        self.main_frame = QtGui.QFrame()
        #self.main_frame.setStyleSheet("QFrame {background-color: rgb(255,255,255); margin:9px; border:1px solid rgb(196, 193, 189);}")
        self.setWidget(self.main_frame)
        self.widgetStack = QtGui.QStackedWidget(self.main_frame)
        self.uiEmptySetup()
        self.uiActorSetup()
        self.tree_object = None

    def uiEmptySetup(self):
        self.emptyStack = QtGui.QWidget()
        self.widgetStack.addWidget(self.emptyStack)

    def uiActorSetup(self):
        self.actorStack = QtGui.QWidget()
        self.widgetStack.addWidget(self.actorStack)
        self.actorLayout = QtGui.QGridLayout()
        self.actorLayout.setAlignment(QtCore.Qt.AlignTop)

        horLine = QtGui.QFrame()
        horLine.setStyleSheet("color: rgb(196, 193, 189);")
        horLine.setFrameStyle(QtGui.QFrame.HLine)
        horLine.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.actorLayout.addWidget(horLine,1,1,1,2)

        # frame axes
        self.texture = QtGui.QGroupBox('Frame Axes')
        self.actorLayout.addWidget(self.texture,2,1,1,2)

        # frame axes visibility
        self.actorLayout.addWidget(QtGui.QLabel('Visibility:'),3,1)
        self.frameAxesVisibilityCheckBox = QtGui.QCheckBox('')
        self.frameAxesVisibilityCheckBox.stateChanged.connect(self.actorFrameAxesVisibility)
        self.actorLayout.addWidget(self.frameAxesVisibilityCheckBox,3,2)

        # frame axes scale
        self.actorLayout.addWidget(QtGui.QLabel('Scale:'),4,1)
        self.axesSpinBox = QtGui.QDoubleSpinBox()
        self.axesSpinBox.setRange(0,1e4)
        self.axesSpinBox.setSingleStep(0.1)
        self.axesSpinBox.setValue(1.0)
        self.axesSpinBox.valueChanged.connect(self.actorFrameAxesScale)
        self.actorLayout.addWidget(self.axesSpinBox,4,2)

        horLine = QtGui.QFrame()
        horLine.setStyleSheet("color: rgb(196, 193, 189);")
        horLine.setFrameStyle(QtGui.QFrame.HLine)
        horLine.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.actorLayout.addWidget(horLine,5,1,1,2)

        # geometry
        self.texture = QtGui.QGroupBox('Geometry')
        self.actorLayout.addWidget(self.texture,6,1,1,2)

        # geometry alpha
        self.actorLayout.addWidget(QtGui.QLabel('Alpha:'),7,1)
        self.alphaSpinBox = QtGui.QDoubleSpinBox()
        self.alphaSpinBox.setRange(0,1)
        self.alphaSpinBox.setSingleStep(0.1)
        self.alphaSpinBox.setValue(1.0)
        self.alphaSpinBox.valueChanged.connect(self.alphaChanged)
        self.actorLayout.addWidget(self.alphaSpinBox,7,2)

        # geometry point size
        self.actorLayout.addWidget(QtGui.QLabel('Point Size:'),8,1)
        self.pointSizeSpinBox = QtGui.QDoubleSpinBox()
        self.pointSizeSpinBox.setRange(1,1e4)
        self.pointSizeSpinBox.setSingleStep(1)
        self.pointSizeSpinBox.setValue(1.0)
        self.pointSizeSpinBox.valueChanged.connect(self.pointSizeChanged)
        self.actorLayout.addWidget(self.pointSizeSpinBox,8,2)

        # geometry line width
        self.actorLayout.addWidget(QtGui.QLabel('Line Width:'),9,1)
        self.lineWidthSpinBox = QtGui.QDoubleSpinBox()
        self.lineWidthSpinBox.setRange(1,1e4)
        self.lineWidthSpinBox.setSingleStep(1)
        self.lineWidthSpinBox.setValue(1.0)
        self.lineWidthSpinBox.valueChanged.connect(self.lineWidthChanged)
        self.actorLayout.addWidget(self.lineWidthSpinBox,9,2)

        # geometry scale
        self.actorLayout.addWidget(QtGui.QLabel('Scale:'),10,1)
        self.scaleSpinBox = QtGui.QDoubleSpinBox()
        self.scaleSpinBox.setRange(0,1e4)
        self.scaleSpinBox.setSingleStep(0.1)
        self.scaleSpinBox.setValue(1.0)
        self.scaleSpinBox.valueChanged.connect(self.actorScale)
        self.actorLayout.addWidget(self.scaleSpinBox,10,2)

        # geometry mode
        self.actorLayout.addWidget(QtGui.QLabel('Mode:'),11,1)
        self.modeComboBox = QtGui.QComboBox()
        self.modeComboBox.addItem('Surface')
        self.modeComboBox.addItem('Wireframe')
        self.modeComboBox.addItem('Surface & Edges')
        self.modeComboBox.addItem('Points')
        self.modeComboBox.currentIndexChanged.connect(self.modeChange)
        self.actorLayout.addWidget(self.modeComboBox,11,2)

        horLine = QtGui.QFrame()
        horLine.setStyleSheet("color: rgb(196, 193, 189);")
        horLine.setFrameStyle(QtGui.QFrame.HLine)
        horLine.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.actorLayout.addWidget(horLine,12,1,1,2)

        self.colortexture = QtGui.QGroupBox('Color and Texture')
        self.actorLayout.addWidget(self.colortexture,13,1,1,2)

        self.actorColorLabel = QtGui.QLabel('Color (R,G,B):')
        self.actorLayout.addWidget(self.actorColorLabel,14,1)
        self.actorColorValue = QtGui.QLabel('')
        self.actorLayout.addWidget(self.actorColorValue,14,2)

        self.colorSetButton = QtGui.QPushButton('Set Color')
        self.colorSetButton.clicked.connect(self.colorSet)
        self.actorLayout.addWidget(self.colorSetButton,15,1,1,2)

        self.textureSetButton = QtGui.QPushButton('Set Texture')
        self.textureSetButton.clicked.connect(self.textureSet)
        self.textureSetButton.setFixedWidth(120)
        self.actorLayout.addWidget(self.textureSetButton,16,1)
        self.textureRemoveButton = QtGui.QPushButton('Remove Texture')
        self.textureRemoveButton.clicked.connect(self.textureRemove)
        self.textureRemoveButton.setFixedWidth(120)
        self.actorLayout.addWidget(self.textureRemoveButton,16,2)

        horLine = QtGui.QFrame()
        horLine.setStyleSheet("color: rgb(196, 193, 189);")
        horLine.setFrameStyle(QtGui.QFrame.HLine)
        horLine.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.actorLayout.addWidget(horLine,17,1,1,2)

        self.actorStack.setLayout(self.actorLayout)

    def actorFrameAxesVisibility(self, state):
        if state == 2:
            self.tree_object.axes_visible = True
        else:
            self.tree_object.axes_visible = False
        self.parent.vtk_main_canvas.setActorVisibility(self.tree_object.axes, self.tree_object.axes_visible)

    def actorFrameAxesScale(self):
        scale = self.axesSpinBox.value()
        self.parent.vtk_main_canvas.setActorScale(self.tree_object.axes, scale)

    def alphaChanged(self):
        alpha = self.alphaSpinBox.value()
        self.parent.vtk_main_canvas.setActorOpacity(self.tree_object.actor, alpha)

    def pointSizeChanged(self):
        size = self.pointSizeSpinBox.value()
        self.parent.vtk_main_canvas.setActorPointSize(self.tree_object.actor, size)

    def lineWidthChanged(self):
        width = self.lineWidthSpinBox.value()
        self.parent.vtk_main_canvas.setActorLineWidth(self.tree_object.actor, width)

    def actorScale(self):
        scale = self.scaleSpinBox.value()
        self.parent.vtk_main_canvas.setActorScale(self.tree_object.actor, scale)

    def modeChange(self):
        mode = self.modeComboBox.currentText()
        if mode == 'Surface':
            self.parent.vtk_main_canvas.setActorToSurface(self.tree_object.actor)
        elif mode == 'Wireframe':
            self.parent.vtk_main_canvas.setActorToWireframe(self.tree_object.actor)
        elif mode == 'Surface & Edges':
            self.parent.vtk_main_canvas.setActorToSurfaceEdges(self.tree_object.actor)
        elif mode == 'Points':
            self.parent.vtk_main_canvas.setActorToPoints(self.tree_object.actor)

    def colorSet(self):
        color = QtGui.QColorDialog.getColor()
        if color.isValid():
            r = color.red()/255.0
            g = color.green()/255.0
            b = color.blue()/255.0
            self.parent.vtk_main_canvas.setActorColor(self.tree_object.actor, r, g, b)
        r, g, b = self.parent.vtk_main_canvas.getActorColor(self.tree_object.actor)
        color_text = '(' + str(int(r*255)) + ', ' + str(int(g*255)) + ', ' + str(int(b*255)) + ')'
        self.actorColorValue.setText(color_text)

    def textureSet(self):
        dlg = QtGui.QFileDialog()
        dlg.setNameFilters(["Images (*.png *.jpg)"])
        dlg.selectNameFilter("Images (*.png *.jpg)")
        if dlg.exec_():
            filename = dlg.selectedFiles()[0]
            self.parent.vtk_main_canvas.setActorTexture(self.tree_object.actor, str(filename))

    def textureRemove(self):
        self.parent.vtk_main_canvas.removeActorTexture(self.tree_object.actor)

    def display(self, name, tree_object=None):
        if name == 'empty':
            self.widgetStack.setCurrentIndex(0)
            self.tree_object = None
        elif name == 'actor':
            self.widgetStack.setCurrentIndex(1)
            self.tree_object = tree_object
            if self.tree_object.axes_visible:
                self.frameAxesVisibilityCheckBox.setCheckState(2)
            else:
                self.frameAxesVisibilityCheckBox.setCheckState(0)
            self.axesSpinBox.setValue(self.parent.vtk_main_canvas.getActorScale(self.tree_object.axes)[0])
            self.alphaSpinBox.setValue(self.parent.vtk_main_canvas.getActorOpacity(self.tree_object.actor))
            self.pointSizeSpinBox.setValue(self.parent.vtk_main_canvas.getActorPointSize(self.tree_object.actor))
            self.lineWidthSpinBox.setValue(self.parent.vtk_main_canvas.getActorLineWidth(self.tree_object.actor))
            self.scaleSpinBox.setValue(self.parent.vtk_main_canvas.getActorScale(self.tree_object.actor)[0])
            edge, mode = self.parent.vtk_main_canvas.getActorRenderMode(self.tree_object.actor)
            if edge == 1:
                self.modeComboBox.setCurrentIndex(2)
            elif mode == 0: # points
                self.modeComboBox.setCurrentIndex(3)
            elif mode == 1: # wireframe
                self.modeComboBox.setCurrentIndex(1)
            elif mode == 2: # surface
                self.modeComboBox.setCurrentIndex(0)
            else:
                self.modeComboBox.setCurrentIndex(0)
            r, g, b = self.parent.vtk_main_canvas.getActorColor(self.tree_object.actor)
            color_text = '(' + str(int(r*255)) + ', ' + str(int(g*255)) + ', ' + str(int(b*255)) + ')'
            self.actorColorValue.setText(color_text)
