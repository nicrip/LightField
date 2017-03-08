#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
additional gui elements - the apply transformation dialog
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
from PyQt4 import QtCore, QtGui, uic

''' custom libs '''
import TransformUtils

class TransformDialog(QtGui.QDialog):
    ''' a dialog box for adding primitive actors '''
    def __init__(self):
        QtGui.QDialog.__init__(self)
        self.setWindowTitle('Apply Transform')
        self.uiSetup()
        self.return_dict = {}

    def uiSetup(self):
        self.setFixedWidth(350)
        self.setFixedHeight(350)      
        self.layout = QtGui.QVBoxLayout()
        buttonLayout = QtGui.QHBoxLayout()
        self.translateWidget()
        self.rotateWidget()
        self.orderWidget()
        self.stackWidget()
        self.layout.addLayout(buttonLayout)
        cancelButton = QtGui.QPushButton("Cancel")
        cancelButton.setFixedWidth(100)
        addButton = QtGui.QPushButton("Apply")
        addButton.setFixedWidth(100)
        buttonLayout.setAlignment(QtCore.Qt.AlignBottom)
        buttonLayout.addStretch()
        buttonLayout.addWidget(cancelButton)
        buttonLayout.addStretch()
        buttonLayout.addWidget(addButton)
        buttonLayout.addStretch()

        # add triggers
        cancelButton.clicked.connect(self.cancel)
        addButton.clicked.connect(self.apply)
        addButton.setDefault(True)

        self.setLayout(self.layout)

    def apply(self):
        if self.order_widget.order0RadioButton.isChecked():
            self.return_dict['order'] = 0
        else:
            self.return_dict['order'] = 1
        self.return_dict['xtranslate'] = self.translate_widget.xSpinBox.value()
        self.return_dict['ytranslate'] = self.translate_widget.ySpinBox.value()
        self.return_dict['ztranslate'] = self.translate_widget.zSpinBox.value()
        self.return_dict['rollrotate'] = self.rotate_widget.rollSpinBox.value()
        self.return_dict['pitchrotate'] = self.rotate_widget.pitchSpinBox.value()
        self.return_dict['yawrotate'] = self.rotate_widget.yawSpinBox.value()
        if self.stack_widget.stackRadioButton.isChecked():
            self.return_dict['stack'] = 1
        else:
            self.return_dict['stack'] = 0
        self.close()

    def cancel(self):
        self.close()

    def translateWidget(self):
        self.translate_widget = QtGui.QGroupBox('Translation')
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
        self.rotate_widget = QtGui.QGroupBox('Rotation')
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

    def orderWidget(self):
        self.order_widget = QtGui.QGroupBox('Order')
        layout = QtGui.QHBoxLayout()
        self.order_widget.order0RadioButton = QtGui.QRadioButton("Rotate->Translate")
        self.order_widget.order0RadioButton.setChecked(False)
        self.order_widget.order1RadioButton = QtGui.QRadioButton("Translate->Rotate")
        self.order_widget.order1RadioButton.setChecked(True)
        layout.addWidget(self.order_widget.order0RadioButton)
        layout.addWidget(self.order_widget.order1RadioButton)
        self.order_widget.setLayout(layout)
        self.layout.addWidget(self.order_widget)

    def stackWidget(self):
        self.stack_widget = QtGui.QGroupBox('Stack/Set Transform')
        layout = QtGui.QHBoxLayout()
        self.stack_widget.stackRadioButton = QtGui.QRadioButton("Stack Transform")
        self.stack_widget.stackRadioButton.setChecked(True)
        self.stack_widget.setRadioButton = QtGui.QRadioButton("Set Transform")
        self.stack_widget.setRadioButton.setChecked(False)
        layout.addWidget(self.stack_widget.stackRadioButton)
        layout.addWidget(self.stack_widget.setRadioButton)
        self.stack_widget.setLayout(layout)
        self.layout.addWidget(self.stack_widget) 
