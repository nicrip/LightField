#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
additional gui elements - the set offset/orientation dialog
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
from PyQt4 import QtCore, QtGui, uic

''' custom libs '''
import TransformUtils

class OffsetOrientationDialog(QtGui.QDialog):
    ''' a dialog box for adding primitive actors '''
    def __init__(self):
        QtGui.QDialog.__init__(self)
        self.setWindowTitle('Set Offset/Orientation')
        self.uiSetup()
        self.return_dict = {}

    def uiSetup(self):
        self.setFixedWidth(350)
        self.setFixedHeight(250)      
        self.layout = QtGui.QVBoxLayout()
        buttonLayout = QtGui.QHBoxLayout()
        self.translateWidget()
        self.rotateWidget()
        self.layout.addLayout(buttonLayout)
        cancelButton = QtGui.QPushButton("Cancel")
        cancelButton.setFixedWidth(100)
        addButton = QtGui.QPushButton("Set")
        addButton.setFixedWidth(100)
        buttonLayout.setAlignment(QtCore.Qt.AlignBottom)
        buttonLayout.addStretch()
        buttonLayout.addWidget(cancelButton)
        buttonLayout.addStretch()
        buttonLayout.addWidget(addButton)
        buttonLayout.addStretch()

        # add triggers
        cancelButton.clicked.connect(self.cancel)
        addButton.clicked.connect(self.set)
        addButton.setDefault(True)

        self.setLayout(self.layout)

    def set(self):
        self.return_dict['xoffset'] = self.translate_widget.xSpinBox.value()
        self.return_dict['yoffset'] = self.translate_widget.ySpinBox.value()
        self.return_dict['zoffset'] = self.translate_widget.zSpinBox.value()
        self.return_dict['rollrotate'] = self.rotate_widget.rollSpinBox.value()
        self.return_dict['pitchrotate'] = self.rotate_widget.pitchSpinBox.value()
        self.return_dict['yawrotate'] = self.rotate_widget.yawSpinBox.value()
        self.close()

    def cancel(self):
        self.close()

    def translateWidget(self):
        self.translate_widget = QtGui.QGroupBox('Offset')
        layout = QtGui.QGridLayout()
        layout.addWidget(QtGui.QLabel('x:'),1,1)
        layout.addWidget(QtGui.QLabel('y:'),1,2)
        layout.addWidget(QtGui.QLabel('z:'),1,3)
        self.translate_widget.xSpinBox = QtGui.QDoubleSpinBox()
        self.translate_widget.xSpinBox.setRange(-1e6, 1e6)
        self.translate_widget.xSpinBox.setSingleStep(1.0)
        self.translate_widget.xSpinBox.setValue(0)
        self.translate_widget.ySpinBox = QtGui.QDoubleSpinBox()
        self.translate_widget.ySpinBox.setRange(-1e6, 1e6)
        self.translate_widget.ySpinBox.setSingleStep(1.0)
        self.translate_widget.ySpinBox.setValue(0)
        self.translate_widget.zSpinBox = QtGui.QDoubleSpinBox()
        self.translate_widget.zSpinBox.setRange(-1e6, 1e6)
        self.translate_widget.zSpinBox.setSingleStep(1.0)
        self.translate_widget.zSpinBox.setValue(0)
        layout.addWidget(self.translate_widget.xSpinBox,2,1)
        layout.addWidget(self.translate_widget.ySpinBox,2,2)
        layout.addWidget(self.translate_widget.zSpinBox,2,3)
        self.translate_widget.setLayout(layout)
        self.layout.addWidget(self.translate_widget)

    def rotateWidget(self):
        self.rotate_widget = QtGui.QGroupBox('Orientation')
        layout = QtGui.QGridLayout()
        layout.addWidget(QtGui.QLabel('roll:'),1,1)
        layout.addWidget(QtGui.QLabel('pitch:'),1,2)
        layout.addWidget(QtGui.QLabel('yaw:'),1,3)
        self.rotate_widget.rollSpinBox = QtGui.QDoubleSpinBox()
        self.rotate_widget.rollSpinBox.setRange(-180, 180)
        self.rotate_widget.rollSpinBox.setSingleStep(1)
        self.rotate_widget.rollSpinBox.setValue(0)
        self.rotate_widget.pitchSpinBox = QtGui.QDoubleSpinBox()
        self.rotate_widget.pitchSpinBox.setRange(-90, 90)
        self.rotate_widget.pitchSpinBox.setSingleStep(1)
        self.rotate_widget.pitchSpinBox.setValue(0)
        self.rotate_widget.yawSpinBox = QtGui.QDoubleSpinBox()
        self.rotate_widget.yawSpinBox.setRange(-180, 180)
        self.rotate_widget.yawSpinBox.setSingleStep(1)
        self.rotate_widget.yawSpinBox.setValue(0)
        layout.addWidget(self.rotate_widget.rollSpinBox,2,1)
        layout.addWidget(self.rotate_widget.pitchSpinBox,2,2)
        layout.addWidget(self.rotate_widget.yawSpinBox,2,3)
        self.rotate_widget.setLayout(layout)
        self.layout.addWidget(self.rotate_widget)
