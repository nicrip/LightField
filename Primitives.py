#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
primitive actor constructors
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK '''
import vtk
from vtk.util import numpy_support as nps

''' custom libs '''
import Billboards

''' primitives add-able via gui '''
PRIMITIVE_GRID = 'grid'
PRIMITIVE_AXES = 'axes'
PRIMITIVE_ARROW = 'arrow'
PRIMITIVE_BOX = 'box'
PRIMITIVE_SPHERE = 'sphere'
PRIMITIVE_CYLINDER = 'cylinder'
PRIMITIVE_ELLIPSOID = 'ellipsoid'
PRIMITIVE_CONE = 'cone'
PRIMITIVE_TORUS = 'torus'

''' fundamental primitives '''
PRIMITIVE_LINE_STRIP = 'linestrip'
PRIMTIIVE_LINE_LIST = 'linelist'
PRIMITIVE_LINE_LOOP = 'lineloop'
PRIMITIVE_TRIANGLE_STRIP = 'trianglestrip'
PRIMITIVE_TRIANGLE_LIST = 'trianglelist'
PRIMITIVE_QUAD = 'quad'
PRIMITIVE_TEXTURED_QUAD = 'texturedquad'
PRIMITIVE_POINT_CLOUD = 'pointcloud'
PRIMITIVE_MODEL = 'model'                       # only .obj files supported right now
PRIMITIVE_TEXT_BILLBOARD = 'textbillboard'      # special custom primitive for text icons - handled specially by our interactors and render loop
PRIMITIVE_IMAGE_BILLBOARD = 'imagebillboard'    # special custom primitive for image icons - handled specially by our interactors and render loop

def _convertLineStripToLineList(numpy_array):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    new_numpy_array = np.zeros((len_array+(len_array-2), 3))
    new_numpy_array[0,:] = numpy_array[0,:]
    idx = 1
    for i in xrange(1,len_array-1):
        new_numpy_array[idx,:] = numpy_array[i,:]
        new_numpy_array[idx+1,:] = numpy_array[i,:]
        idx += 2
    new_numpy_array[-1,:] = numpy_array[-1,:]

    return new_numpy_array

def _convertLineLoopToLineList(numpy_array):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    new_numpy_array = np.zeros((2*len_array, 3))
    new_numpy_array[0,:] = numpy_array[0,:]
    idx = 1
    for i in xrange(1,len_array):
        new_numpy_array[idx,:] = numpy_array[i,:]
        new_numpy_array[idx+1,:] = numpy_array[i,:]
        idx += 2
    new_numpy_array[-1,:] = numpy_array[0,:]

    return new_numpy_array

def LineStrip(numpy_array, color_array=None):
    if color_array is not None:
        numpy_array = _convertLineStripToLineList(numpy_array)
        return LineList(numpy_array, color_array)

    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    lines = vtk.vtkCellArray()
    lines.InsertNextCell(len_array)
    for i in xrange(len_array):
        lines.InsertCellPoint(i)

    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)

    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputConnection(polygon.GetProducerPort())

    actor = vtk.vtkActor()
    actor.SetMapper(polygonMapper)

    return actor

def LineList(numpy_array, color_array=None):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    lines = vtk.vtkCellArray()
    num_lines = len_array/2
    line_count = 0
    for i in xrange(0, len_array, 2):
        lines.InsertNextCell(2)
        lines.InsertCellPoint(i)
        lines.InsertCellPoint(i+1)
        line_count += 1
        if line_count == num_lines:
            break

    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)

    # color
    if color_array is not None:
        if np.shape(color_array)[0] == 3:
            color_array = np.transpose(color_array)
        colors = nps.numpy_to_vtk(color_array, deep=1)
        colors.SetName("Colors")
        polygon.GetCellData().SetScalars(colors)

    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputConnection(polygon.GetProducerPort())

    actor = vtk.vtkActor()
    actor.SetMapper(polygonMapper)

    return actor

def LineLoop(numpy_array, color_array=None):
    if color_array is not None:
        numpy_array = _convertLineLoopToLineList(numpy_array)
        return LineList(numpy_array, color_array)

    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    lines = vtk.vtkCellArray()
    lines.InsertNextCell(len_array+1)
    for i in xrange(len_array):
        lines.InsertCellPoint(i)
    lines.InsertCellPoint(0)

    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)

    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputConnection(polygon.GetProducerPort())

    actor = vtk.vtkActor()
    actor.SetMapper(polygonMapper)

    return actor

def _convertTriangleStripToTriangleList(numpy_array):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    new_numpy_array = np.zeros(((3*(len_array-1))-3, 3))
    idx = 0
    for i in xrange(2,len_array):
        new_numpy_array[idx,:] = numpy_array[i-2,:]
        new_numpy_array[idx+1,:] = numpy_array[i-1,:]
        new_numpy_array[idx+2,:] = numpy_array[i,:]
        idx += 3

    return new_numpy_array

def TriangleStrip(numpy_array, color_array=None):
    if color_array is not None:
        numpy_array = _convertTriangleStripToTriangleList(numpy_array)
        return TriangleList(numpy_array, color_array)

    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    triangleStrip = vtk.vtkTriangleStrip()
    triangleStrip.GetPointIds().SetNumberOfIds(len_array)
    for i in xrange(len_array):
        triangleStrip.GetPointIds().SetId(i,i)
     
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(triangleStrip)
     
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetStrips(cells)

    mapper = vtk.vtkDataSetMapper()
    mapper.SetInput(polydata)
     
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def TriangleList(numpy_array, color_array=None):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    cells = vtk.vtkCellArray()
    num_triangles = len_array/3
    triangle_count = 0
    for i in xrange(0, len_array, 3):
        triangle = vtk.vtkTriangle()
        triangle.GetPointIds().SetNumberOfIds(3)
        triangle.GetPointIds().SetId(0,i)
        triangle.GetPointIds().SetId(1,i+1)
        triangle.GetPointIds().SetId(2,i+2)
        cells.InsertNextCell(triangle)
        triangle_count += 1
        if triangle_count == num_triangles:
            break
     
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(cells)

    # color
    if color_array is not None:
        if np.shape(color_array)[0] == 3:
            color_array = np.transpose(color_array)
        colors = nps.numpy_to_vtk(color_array, deep=1)
        colors.SetName("Colors")
        polydata.GetCellData().SetScalars(colors)

    mapper = vtk.vtkDataSetMapper()
    mapper.SetInput(polydata)
     
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Quad(numpy_array):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)

    print numpy_array

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(4)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    quad = vtk.vtkQuad()
    quad.GetPointIds().SetId(0,0)
    quad.GetPointIds().SetId(1,1)
    quad.GetPointIds().SetId(2,2)
    quad.GetPointIds().SetId(3,3)

    quads = vtk.vtkCellArray()
    quads.InsertNextCell(quad)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(quads)

    mapper = vtk.vtkDataSetMapper()
    mapper.SetInput(polydata)
     
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def TexturedQuad(numpy_array, image_path):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)

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

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(4)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    quad = vtk.vtkQuad()
    quad.GetPointIds().SetId(0,0)
    quad.GetPointIds().SetId(1,1)
    quad.GetPointIds().SetId(2,2)
    quad.GetPointIds().SetId(3,3)

    quads = vtk.vtkCellArray()
    quads.InsertNextCell(quad)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(quads)

    texCoords = vtk.vtkFloatArray()
    texCoords.SetNumberOfComponents(2)
    texCoords.SetName("TextureCoordinates")
    texCoords.InsertNextTuple((0.0, 0.0))
    texCoords.InsertNextTuple((1.0, 0.0))
    texCoords.InsertNextTuple((1.0, 1.0))
    texCoords.InsertNextTuple((0.0, 1.0))

    polydata.GetPointData().SetTCoords(texCoords)

    mapper = vtk.vtkDataSetMapper()
    mapper.SetInput(polydata)
     
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetTexture(texture)

    return actor

def PointCloud(numpy_array, color_array=None):
    if np.shape(numpy_array)[0] == 3:
        numpy_array = np.transpose(numpy_array)
    len_array = np.shape(numpy_array)[0]

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len_array)
    points.SetData(nps.numpy_to_vtk(numpy_array, deep=1))

    verts = vtk.vtkCellArray()
    for i in xrange(len_array):
        verts.InsertNextCell(1)
        verts.InsertCellPoint(i)
     
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetVerts(verts)

    # color
    if color_array is not None:
        if np.shape(color_array)[0] == 3:
            color_array = np.transpose(color_array)
        colors = nps.numpy_to_vtk(color_array, deep=1)
        colors.SetName("Colors")
        polydata.GetCellData().SetScalars(colors)

    mapper = vtk.vtkDataSetMapper()
    mapper.SetInput(polydata)
     
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Model(model_path, model_image_path=None):
    if model_path[-4:].lower() == '.obj':
        reader = vtk.vtkOBJReader()
        reader.SetFileName(model_path)
        reader.Update()

        polyData = reader.GetOutput()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInput(polyData)

        objActor = vtk.vtkActor()
        objActor.SetMapper(mapper)

        # texture from image
        if model_image_path is not None:
            texture = vtk.vtkTexture()

            if model_image_path[-4:].lower() == '.jpg' or model_image_path[-4:].lower() == '.jpeg':
                jpgReader = vtk.vtkJPEGReader()
                jpgReader.SetFileName(model_image_path)
                jpgReader.Update()
                texture.SetInputConnection(jpgReader.GetOutputPort())
            elif model_image_path[-4:].lower() == '.png':
                pngReader = vtk.vtkPNGReader()
                pngReader.SetFileName(model_image_path)
                pngReader.Update()
                texture.SetInputConnection(pngReader.GetOutputPort())

            objActor.SetTexture(texture)

        return objActor

def CustomTextBillboard(text, text_size=None):
    actor = Billboards.TextBillboard(text, text_size)

    return actor

def CustomImageBillboard(image_path, width=None, height=None):
    actor = Billboards.ImageBillboard(image_path, width, height)

    return actor

def Grid(full_length, cell_length):
    num_cols = int(math.ceil(float(full_length)/cell_length))
    if (num_cols%2) == 1:   # even number of cols/rows
        num_cols += 1
    full_length = num_cols*cell_length

    gridPoints = vtk.vtkPoints()
    for i in xrange(num_cols+1):
        p0 = [-full_length/2.0, -full_length/2.0 + i*cell_length, 0.0]
        p1 = [full_length/2.0, -full_length/2.0 + i*cell_length, 0.0]
        if (i%2) == 1:
            gridPoints.InsertNextPoint(p0)
            gridPoints.InsertNextPoint(p1)
        else:
            gridPoints.InsertNextPoint(p1)
            gridPoints.InsertNextPoint(p0)
    for i in xrange(num_cols+1):
        p0 = [-full_length/2.0 + i*cell_length, -full_length/2.0, 0.0]
        p1 = [-full_length/2.0 + i*cell_length, full_length/2.0, 0.0]
        if (i%2) == 1:
            gridPoints.InsertNextPoint(p0)
            gridPoints.InsertNextPoint(p1)
        else:
            gridPoints.InsertNextPoint(p1)
            gridPoints.InsertNextPoint(p0)

    baseGrid = vtk.vtkStructuredGrid()
    baseGrid.SetDimensions(4*(num_cols+1),1,1)
    baseGrid.SetPoints(gridPoints)

    outlineFilter = vtk.vtkStructuredGridGeometryFilter()
    outlineFilter.SetInputConnection(baseGrid.GetProducerPort())
    outlineFilter.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(outlineFilter.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Axes():
    axes = vtk.vtkAxesActor()
    axes.SetShaftTypeToCylinder()
    axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().ItalicOff()
    axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().ShadowOff()
    axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().BoldOn()
    axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().ItalicOff()
    axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().ShadowOff()
    axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().BoldOn()
    axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().ItalicOff()
    axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().ShadowOff()
    axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().BoldOn()
    return axes

def Arrow(res):
    arrowSource = vtk.vtkArrowSource()
    arrowSource.SetTipResolution(res)
    arrowSource.SetShaftResolution(res)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(arrowSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Box(x_length, y_length, z_length):
    cubeSource = vtk.vtkCubeSource()
    cubeSource.SetXLength(x_length)
    cubeSource.SetYLength(y_length)
    cubeSource.SetZLength(z_length)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(cubeSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Sphere(radius, tres, pres):
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetRadius(radius)
    sphereSource.SetThetaResolution(tres)
    sphereSource.SetPhiResolution(pres)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphereSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Cylinder(radius, height, res):
    cylinderSource = vtk.vtkCylinderSource()
    cylinderSource.SetRadius(radius)
    cylinderSource.SetHeight(height)
    cylinderSource.SetResolution(res)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(cylinderSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Ellipsoid(radius_x, radius_y, radius_z):
    ellipsoid = vtk.vtkParametricEllipsoid()
    ellipsoid.SetXRadius(radius_x)
    ellipsoid.SetYRadius(radius_y)
    ellipsoid.SetZRadius(radius_z)
    parametricSource = vtk.vtkParametricFunctionSource()
    parametricSource.SetParametricFunction(ellipsoid)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(parametricSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Ellipsoid_deprecated(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, bounds, sample_dims):
    #create an ellipsoid using a implicit quadric
    quadric = vtk.vtkQuadric()
    quadric.SetCoefficients(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9)
     
    # The sample function generates a distance function from the implicit
    # function. This is then contoured to get a polygonal surface.
    sample = vtk.vtkSampleFunction()
    sample.SetImplicitFunction(quadric)
    sample.SetModelBounds(-bounds, bounds, -bounds, bounds, -bounds, bounds)
    sample.SetSampleDimensions(sample_dims, sample_dims, sample_dims)
    sample.ComputeNormalsOff()
     
    # contour
    surface = vtk.vtkContourFilter()
    surface.SetInputConnection(sample.GetOutputPort())
    surface.GenerateValues(1.0, 1.0, 1.0);
     
    # mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(surface.GetOutputPort())
    mapper.ScalarVisibilityOff()
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Cone(radius, height, res):
    coneSource = vtk.vtkConeSource()
    coneSource.SetRadius(radius)
    coneSource.SetHeight(height)
    coneSource.SetResolution(res)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(coneSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def Torus(radius_ring, radius_cross_section):
    ellipsoid = vtk.vtkParametricTorus()
    ellipsoid.SetRingRadius(radius_ring)
    ellipsoid.SetCrossSectionRadius(radius_cross_section)
    parametricSource = vtk.vtkParametricFunctionSource()
    parametricSource.SetParametricFunction(ellipsoid)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(parametricSource.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor