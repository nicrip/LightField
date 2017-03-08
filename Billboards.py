#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
billboard classes
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK '''
import vtk

class TextBillboard(vtk.vtkTextActor):
    ''' a fixed-size text label (i.e. a 2D label in the 3D world) '''
    def __init__(self, text, text_size=None):
        self.vtkTransform = None
        self.pos = vtk.vtkCoordinate()
        self.SetInput(text)
        self.GetTextProperty().BoldOn()
        self.GetTextProperty().SetColor(0.0,0.0,0.0)
        self.GetTextProperty().SetFontFamilyToArial()
        if text_size is not None:
            self.GetTextProperty().SetFontSize(text_size)

        self.pos.SetCoordinateSystemToWorld()
        coordView = vtk.vtkCoordinate()
        coordView.SetReferenceCoordinate(self.pos)
        coordView.SetCoordinateSystemToViewport()

        mapper = vtk.vtkPolyDataMapper2D()
        mapper.SetTransformCoordinate(coordView)

        self.SetMapper(mapper)

    def SetPosition(self, x, y, z):
        self.pos.SetValue(x, y, z)

    def SetOrientation(self, roll, pitch, yaw):
        prop = self.GetTextProperty()
        prop.SetOrientation(roll)

    def ApplyTransformation(self):
        mat = self.vtkTransform.GetMatrix()
        a = np.zeros((4,4))

        for r in xrange(4):
            for c in xrange(4):
                a[r][c] = mat.GetElement(r, c)

        self.SetPosition(a[0,3],a[1,3],a[2,3])

    def SetUserTransform(self, transform):
        self.vtkTransform = transform
        self.ApplyTransformation()

    def GetUserTransform(self):
        return self.vtkTransform

class ImageBillboard(vtk.vtkTexturedActor2D):
    ''' a fixed-size billboard for images (i.e. a 2D sprite in the 3D world) '''
    def __init__(self, image_path, width=None, height=None):
        self.vtkTransform = None
        self.pos = vtk.vtkCoordinate()
        if image_path[-4:].lower() == '.jpg' or image_path[-4:].lower() == '.jpeg':
            reader = vtk.vtkJPEGReader()
            reader.SetFileName(image_path)
            reader.Update()
        elif image_path[-4:].lower() == '.png':
            reader = vtk.vtkPNGReader()
            reader.SetFileName(image_path)
            reader.Update()
        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())
        image_dims = reader.GetOutput().GetDimensions()

        self.SetTexture(texture)

        if width is not None and height is not None:
            self.construct(width, height)
        else:
            self.construct(image_dims[0], image_dims[1])

    def construct(self, width, height):
        points = vtk.vtkPoints()
        points.InsertNextPoint([-float(width)/2, -float(height)/2, 0.0])
        points.InsertNextPoint([float(width)/2, -float(height)/2, 0.0])
        points.InsertNextPoint([float(width)/2, float(height)/2, 0.0])
        points.InsertNextPoint([-float(width)/2, float(height)/2, 0.0])

        polys = vtk.vtkCellArray()
        poly = vtk.vtkPolygon()
        poly.GetPointIds().SetNumberOfIds(4)
        poly.GetPointIds().SetId(0,0)
        poly.GetPointIds().SetId(1,1)
        poly.GetPointIds().SetId(2,2)
        poly.GetPointIds().SetId(3,3)
        polys.InsertNextCell(poly)

        self.data = vtk.vtkPolyData()
        self.data.SetPoints(points)
        self.data.SetPolys(polys)

        texCoords = vtk.vtkFloatArray()
        texCoords.SetNumberOfComponents(2)
        texCoords.SetName("TextureCoordinates")
        texCoords.InsertNextTuple((0.0, 0.0))
        texCoords.InsertNextTuple((1.0, 0.0))
        texCoords.InsertNextTuple((1.0, 1.0))
        texCoords.InsertNextTuple((0.0, 1.0))

        self.data.GetPointData().SetTCoords(texCoords)

        self.pos.SetCoordinateSystemToWorld()
        coordView = vtk.vtkCoordinate()
        coordView.SetReferenceCoordinate(self.pos)
        coordView.SetCoordinateSystemToViewport()

        mapper = vtk.vtkPolyDataMapper2D()
        mapper.SetInput(self.data)
        mapper.SetTransformCoordinate(coordView)

        self.SetMapper(mapper)

    def SetPosition(self, x, y, z):
        self.pos.SetValue(x, y, z)

    def SetOrientation(self, roll, pitch, yaw):
        t = vtk.vtkTransform()
        t.RotateZ(roll)
        tf = vtk.vtkTransformPolyDataFilter()
        tf.SetInput(self.data)
        tf.SetTransform(t)
        tf.Update()

        coordView = vtk.vtkCoordinate()
        coordView.SetReferenceCoordinate(self.pos)
        coordView.SetCoordinateSystemToViewport()

        mapper = vtk.vtkPolyDataMapper2D()
        mapper.SetInputConnection(tf.GetOutputPort())
        mapper.SetTransformCoordinate(coordView)

        self.SetMapper(mapper)

    def ApplyTransformation(self):
        mat = self.vtkTransform.GetMatrix()
        a = np.zeros((4,4))

        for r in xrange(4):
            for c in xrange(4):
                a[r][c] = mat.GetElement(r, c)

        self.SetPosition(a[0,3],a[1,3],a[2,3])

    def SetUserTransform(self, transform):
        self.vtkTransform = transform
        self.ApplyTransformation()

    def GetUserTransform(self):
        return self.vtkTransform