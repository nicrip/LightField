#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
main window
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt4 import QtCore, QtGui, uic

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

class TreeObject(object):
    def __init__(self, actor, object_type, axes, transform):
        self.actor = actor
        self.object_type = object_type
        self.axes = axes
        self.transform = transform
        self.axes_visible = False

class MainApp(QtGui.QMainWindow):
    ''' the main Qt4 window '''
    def __init__(self):
        super(MainApp,self).__init__()
        # dictionary to keep track of actors/icons that correspond to tree widget items - used to handle Qt4->vtk scene changes
        self.tree_widget_items_to_objects = {}
        self.ui = None
        self.vtk_main_canvas = None
        self.setup()

    def setup(self):
        # the main Qt4 window has been designed in QtCreator
        import LightFieldVisualizer_ui
        
        # setup Qt4 UI
        self.ui = LightFieldVisualizer_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QtGui.QIcon('icons/icon.png'))
        self.setWindowTitle('LightField - The Lightweight Field Robotics Visualizer')

        # setup the actor properties dock widget
        self.ui.actorPropertiesDock = GUIActorPropertiesDock.ActorPropertiesDock(self)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.ui.actorPropertiesDock)

        # place vtk canvas inside the Qt4 vtk panel
        self.vtk_main_canvas = MainFrame(self.ui.vtk_panel)
        self.ui.vtk_layout = QtGui.QHBoxLayout()
        self.ui.vtk_layout.addWidget(self.vtk_main_canvas)
        self.ui.vtk_layout.setContentsMargins(0,0,0,0)
        self.ui.vtk_panel.setLayout(self.ui.vtk_layout)

        # setup triggers/countertriggers for background shade selection
        self.ui.actionBGLight.triggered.connect(self.vtk_main_canvas.setBackgroundLight)
        self.ui.actionBGDark.triggered.connect(self.vtk_main_canvas.setBackgroundDark)
        self.vtk_main_canvas.background_light_signal.connect(self.setBackgroundCheckMarks)

        # setup triggers/countertriggers for camera selection
        self.ui.actionCameraPerspective.triggered.connect(self.vtk_main_canvas.perspectiveCamera)
        self.ui.actionCameraTopDown.triggered.connect(self.vtk_main_canvas.topDownCamera)
        self.vtk_main_canvas.camera_perspective_signal.connect(self.setCameraSelectionCheckMarks)

        # setup trigger for camera reset
        self.ui.actionCameraReset.triggered.connect(self.vtk_main_canvas.resetCamera)

        # setup countertrigger for adding actors to the vtk canvas
        self.vtk_main_canvas.add_actor_signal.connect(self.addActor)
        self.vtk_main_canvas.add_orientation_axes_signal.connect(self.addActor)

        # setup triggers for QTreeWidget
        self.ui.treeWidgetActors.setColumnHidden(1, True)                                       # hide ID column
        self.ui.treeWidgetActors.setHeaderLabels(['Name','ID',''])                              # set last header label (type) to nothing
        self.ui.treeWidgetActors.header().setStretchLastSection(False)                          # don't stretch the last column
        self.ui.treeWidgetActors.setColumnWidth(2, 24)                                          # fix the last column width
        self.ui.treeWidgetActors.header().setResizeMode(0, QtGui.QHeaderView.Stretch)
        self.ui.treeWidgetActors.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.ui.treeWidgetActors.customContextMenuRequested.connect(self.treeItemContextMenu)
        self.ui.treeWidgetActors.header().setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.ui.treeWidgetActors.header().customContextMenuRequested.connect(self.treeWidgetContextMenu)
        self.ui.treeWidgetActors.itemChanged.connect(self.treeItemChanged)
        self.treeWidgetTextEditing = False
        self.treeWidgetPreviousText = None
        self.treeWidgetPreviousSelected = None
        self.connect(self.ui.treeWidgetActors, QtCore.SIGNAL("itemClicked(QTreeWidgetItem*, int)"), self.treeItemSelected)


    def start(self, timer_update=False, timer_fps=30):
        # startup the vtk canvas
        if timer_update:
            self.vtk_main_canvas.setupTimerCallback(timer_fps)
        self.vtk_main_canvas.start()

    ###### trigger functions #######
    def treeItemSelected(self):
        treeWidgetItems = self.ui.treeWidgetActors.selectedItems()
        if len(treeWidgetItems) == 0:
            # no object selected
            self.ui.actorPropertiesDock.display('empty')
            return
        treeWidgetItem = treeWidgetItems[0]
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        if actor_tree_widget_ID == 'orientation axes':
            # we dont allow any changes to the orientation axes except for visibility
            self.ui.actorPropertiesDock.display('empty')
            return
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            self.ui.actorPropertiesDock.display('actor', tree_object)
        else:
            self.ui.actorPropertiesDock.display('empty')

    def treeItemChanged(self, treeWidgetItem):
        self.ui.treeWidgetActors.blockSignals(True)
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        # checking visibility changes
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            if treeWidgetItem.checkState(0) == 0:
                actor_visible = False
            else:
                actor_visible = True
            actor = tree_object.actor
            self.vtk_main_canvas.setActorVisibility(actor, actor_visible)
        self.ui.treeWidgetActors.blockSignals(False)

    def treeWidgetContextMenu(self, position):
        # create context menu when right-clicking on the tree widget - add primitives and directories
        menu = QtGui.QMenu()
        addPrimitiveAction = menu.addAction("Add Primitive (top level)")
        addPrimitiveAction.triggered.connect(lambda: self.treeItemAddPrimitive(self.ui.treeWidgetActors.invisibleRootItem()))
        addDirectoryAction = menu.addAction("Add Directory (top level)")
        addDirectoryAction.triggered.connect(lambda: self.treeItemAddDirectory(self.ui.treeWidgetActors.invisibleRootItem()))
        menu.exec_(self.ui.treeWidgetActors.header().viewport().mapToGlobal(position))

    def treeItemContextMenu(self, position):
        # create context menu when right-clicking on a tree object - add primitives/directories, rename, set offset/orientation, apply transform, remove
        treeWidgetItems = self.ui.treeWidgetActors.selectedItems()
        if len(treeWidgetItems) == 0:
            # no object selected
            return
        treeWidgetItem = treeWidgetItems[0]
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        if actor_tree_widget_ID == 'orientation axes':
            # we dont allow any changes to the orientation axes except for visibility
            return
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        menu = QtGui.QMenu()
        renameAction = menu.addAction("Rename")
        renameAction.triggered.connect(lambda: self.treeItemRename(treeWidgetItem))
        menu.addSeparator()
        offsetOrientationAction = menu.addAction("Set Offset/Orientation")
        offsetOrientationAction.triggered.connect(lambda: self.treeItemSetOffsetOrientation(treeWidgetItem))
        transformAction = menu.addAction("Apply Transform")
        transformAction.triggered.connect(lambda: self.treeItemApplyTransform(treeWidgetItem))
        resetTransformAction = menu.addAction("Reset Transform")
        resetTransformAction.triggered.connect(lambda: self.treeItemRecurseResetTransform(treeWidgetItem))
        menu.addSeparator()
        if tree_object_type == DIR_TYPE:
            addPrimitiveAction = menu.addAction("Add Primitive")
            addPrimitiveAction.triggered.connect(lambda: self.treeItemAddPrimitive(treeWidgetItem))
            addDirectoryAction = menu.addAction("Add Directory")
            addDirectoryAction.triggered.connect(lambda: self.treeItemAddDirectory(treeWidgetItem))
            menu.addSeparator()
        removeAction = menu.addAction("Remove")
        removeAction.triggered.connect(lambda: self.treeItemRemove(treeWidgetItem))
        menu.exec_(self.ui.treeWidgetActors.viewport().mapToGlobal(position))

    def treeItemSetOffsetOrientation(self, treeWidgetItem):
        # startup the set offset/orientation widget
        offsetOrientationDialog = GUIOffsetOrientationDialog.OffsetOrientationDialog()
        offsetOrientationDialog.exec_()
        return_dict = offsetOrientationDialog.return_dict
        if len(return_dict) > 0: 
            self.treeItemRecurseSetOffsetOrientation(treeWidgetItem, return_dict['xoffset'], return_dict['yoffset'], return_dict['zoffset'], return_dict['rollrotate'], return_dict['pitchrotate'], return_dict['yawrotate'])

    def treeItemRecurseSetOffsetOrientation(self, treeWidgetItem, x_offset, y_offset, z_offset, roll, pitch, yaw):
        # recurse through the tree, setting the offset/orientation of all children of the current tree object
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == DIR_TYPE:
            for i in xrange(treeWidgetItem.childCount()):
                child_tree_widget = treeWidgetItem.child(i)
                self.treeItemRecurseSetOffsetOrientation(child_tree_widget, x_offset, y_offset, z_offset, roll, pitch, yaw)
        else:
            actor = tree_object.actor
            self.vtk_main_canvas.setOffset(actor, x_offset, y_offset, z_offset)
            self.vtk_main_canvas.setOrientation(actor, roll, pitch, yaw)

    def treeItemApplyTransform(self, treeWidgetItem):
        # startup the apply transformation widget - applying a transformation means we concatenate a new transform to the current transform
        transformDialog = GUITransformDialog.TransformDialog()
        transformDialog.exec_()
        return_dict = transformDialog.return_dict
        if len(return_dict) > 0:
            if return_dict['stack'] == 1:
                tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
                tree_object.transform = TransformUtils.ApplyTransformationEuler(tree_object.transform, return_dict['xtranslate'], return_dict['ytranslate'], return_dict['ztranslate'], return_dict['rollrotate'], return_dict['pitchrotate'], return_dict['yawrotate'], return_dict['order'])
                self.treeItemRecurseComposeTransform(treeWidgetItem)
            else:
                new_transform = vtk.vtkTransform()
                new_transform = TransformUtils.ApplyTransformationEuler(new_transform, return_dict['xtranslate'], return_dict['ytranslate'], return_dict['ztranslate'], return_dict['rollrotate'], return_dict['pitchrotate'], return_dict['yawrotate'], return_dict['order'])
                self.treeItemSetTransform(treeWidgetItem, new_transform)

    def treeItemSetTransform(self, treeWidgetItem, transform):
        # setting a transformation means we reset the curent transform to the given transform
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object.transform = transform
        self.treeItemRecurseComposeTransform(treeWidgetItem)

    def treeItemRecurseComposeTransform(self, treeWidgetItem):
        # recurse through the tree, find all children/grandchildren of the tree widget, and finally compose the entire transformation for the leaves of the tree
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == DIR_TYPE:
            for i in xrange(treeWidgetItem.childCount()):
                child_tree_widget = treeWidgetItem.child(i)
                self.treeItemRecurseComposeTransform(child_tree_widget)
        else:
            self.treeItemComposeTransform(treeWidgetItem)

    def treeItemComposeTransform(self, treeWidgetItem):
        # this function is only called for leaf treeWidgetItems - recurse back up the tree, composing the full transform
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == DIR_TYPE:
            return
        tree_object_transform = tree_object.transform
        composed_transform = vtk.vtkTransform()
        composed_transform.PostMultiply()
        composed_transform.SetMatrix(tree_object_transform.GetMatrix())
        parent_tree_widget = treeWidgetItem.parent()
        while parent_tree_widget is not None:
            parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget]
            parent_tree_object_transform = parent_tree_object.transform
            composed_transform.Concatenate(parent_tree_object_transform)
            parent_tree_widget = parent_tree_widget.parent()
        tree_object.actor.SetUserTransform(composed_transform)
        tree_object.axes.SetUserTransform(composed_transform)

    def treeItemRecurseResetTransform(self, treeWidgetItem):
        # reset the transform associated with this tree object, then determine the full transform of all the objects affected by this change
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object.transform = vtk.vtkTransform()
        self.treeItemRecurseComposeTransform(treeWidgetItem)

    def treeItemAddPrimitive(self, treeWidgetItem):
        # startup the add primitive widget
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        primitiveDialog = GUIPrimitiveDialog.PrimitiveDialog()
        primitiveDialog.exec_()
        return_type = primitiveDialog.return_dict['type']
        if return_type == None:
            return
        elif return_type == Primitives.PRIMITIVE_GRID:
            new_grid_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_grid_level_list = new_grid_name.split('/')
            new_grid_length = primitiveDialog.return_dict['width']
            new_grid_cell_length = primitiveDialog.return_dict['cell']
            self.vtk_main_canvas.addGrid(new_grid_level_list, new_grid_length, new_grid_cell_length)
        elif return_type == Primitives.PRIMITIVE_AXES:
            new_axes_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_axes_level_list = new_axes_name.split('/')
            self.vtk_main_canvas.addAxes(new_axes_level_list)
        elif return_type == Primitives.PRIMITIVE_ARROW:
            new_arrow_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_arrow_level_list = new_arrow_name.split('/')
            new_arrow_res = primitiveDialog.return_dict['res']   
            self.vtk_main_canvas.addArrow(new_arrow_level_list, new_arrow_res)
        elif return_type == Primitives.PRIMITIVE_BOX:
            new_box_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_box_level_list = new_box_name.split('/')
            new_box_x = primitiveDialog.return_dict['x']
            new_box_y = primitiveDialog.return_dict['y']
            new_box_z = primitiveDialog.return_dict['z']
            self.vtk_main_canvas.addBox(new_box_level_list, new_box_x, new_box_y, new_box_z)
        elif return_type == Primitives.PRIMITIVE_SPHERE:
            new_sphere_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_sphere_level_list = new_sphere_name.split('/')
            new_sphere_radius = primitiveDialog.return_dict['radius']
            new_sphere_tres = primitiveDialog.return_dict['tres']
            new_sphere_pres = primitiveDialog.return_dict['pres']
            self.vtk_main_canvas.addSphere(new_sphere_level_list, new_sphere_radius, new_sphere_tres, new_sphere_pres)
        elif return_type == Primitives.PRIMITIVE_CYLINDER:
            new_cylinder_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_cylinder_level_list = new_cylinder_name.split('/')
            new_cylinder_radius = primitiveDialog.return_dict['radius']
            new_cylinder_height = primitiveDialog.return_dict['height']
            new_cylinder_res = primitiveDialog.return_dict['res']            
            self.vtk_main_canvas.addCylinder(new_cylinder_level_list, new_cylinder_radius, new_cylinder_height, new_cylinder_res)
        elif return_type == Primitives.PRIMITIVE_ELLIPSOID:
            new_ellipsoid_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_ellipsoid_level_list = new_ellipsoid_name.split('/')
            new_ellipsoid_xradius = primitiveDialog.return_dict['xradius']
            new_ellipsoid_yradius = primitiveDialog.return_dict['yradius']
            new_ellipsoid_zradius = primitiveDialog.return_dict['zradius']
            self.vtk_main_canvas.addEllipsoid(new_ellipsoid_level_list, new_ellipsoid_xradius, new_ellipsoid_yradius, new_ellipsoid_zradius)
        elif return_type == Primitives.PRIMITIVE_CONE:
            new_cone_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_cone_level_list = new_cone_name.split('/')
            new_cone_radius = primitiveDialog.return_dict['radius']
            new_cone_height = primitiveDialog.return_dict['height']
            new_cone_res = primitiveDialog.return_dict['res']            
            self.vtk_main_canvas.addCone(new_cone_level_list, new_cone_radius, new_cone_height, new_cone_res)
        elif return_type == Primitives.PRIMITIVE_TORUS:
            new_torus_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_torus_level_list = new_torus_name.split('/')
            new_torus_ring_radius = primitiveDialog.return_dict['ringradius']
            new_torus_cs_radius = primitiveDialog.return_dict['csradius']
            self.vtk_main_canvas.addTorus(new_torus_level_list, new_torus_ring_radius, new_torus_cs_radius)

    def treeItemAddDirectory(self, treeWidgetItem):
        # startup the add directory widget
        self.ui.treeWidgetActors.blockSignals(True)
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        newDirDialog = QtGui.QInputDialog()
        newDirDialog.setFocus()
        dirName, ok = newDirDialog.getText(self, 'Add Directory', 'Enter directory name:')
        if ok:
            if dirName == '' or '/' in dirName:
                warnings.warn("add directory failed: the new directory name is an empty string or contains /; directory not added", RuntimeWarning)
                self.ui.treeWidgetActors.blockSignals(False)
                return
            else:
                new_actor_tree_widget_ID = actor_tree_widget_ID + str(dirName) + '/'
                # we should check if this directory already exists before adding it
                actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                if len(actor_tree_widget_list) > 0:
                    warnings.warn("add directory failed: a directory of that ID already exists; directory not added", RuntimeWarning)
                    self.ui.treeWidgetActors.blockSignals(False)
                    return
                new_tree_widget_item = QtGui.QTreeWidgetItem(treeWidgetItem)
                new_tree_widget_item.setText(0, dirName)
                new_tree_widget_item.setText(1, new_actor_tree_widget_ID)
                new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/folder.png'))
                new_tree_widget_item.setText(2, DIR_TYPE)
                new_tree_widget_item.setFlags(new_tree_widget_item.flags() | QtCore.Qt.ItemIsTristate | QtCore.Qt.ItemIsUserCheckable)
                new_tree_widget_item.setExpanded(True)
                new_tree_widget_item.setCheckState(0, QtCore.Qt.Checked)
                # add a reference to this directory in our dict
                self.tree_widget_items_to_objects[new_tree_widget_item] = TreeObject(None, DIR_TYPE, None, vtk.vtkTransform())
        self.ui.treeWidgetActors.blockSignals(False)

    def treeItemRename(self, treeWidgetItem):
        # startup the rename tree object widget
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            # if the tree object is an actor, startup the rename actor widget
            renameDialog = QtGui.QInputDialog()
            renameDialog.setFocus()
            newName, ok = renameDialog.getText(self, 'Rename Actor', 'Enter new name:')
            if ok:
                if newName == '' or '/' in newName:
                    warnings.warn("rename failed: the new name is an empty string or contains /; actor not renamed", RuntimeWarning)
                    return
                else:
                    level_list = actor_tree_widget_ID.split('/')
                    level_list[-1] = str(newName)
                    new_actor_tree_widget_ID = '/'.join(level_list)
                    actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                    if len(actor_tree_widget_list) > 0:
                        warnings.warn("rename failed: an actor of that ID already exists; actor not renamed", RuntimeWarning)
                        return
                    treeWidgetItem.setText(0, newName)
                    treeWidgetItem.setText(1, new_actor_tree_widget_ID)
        elif tree_object_type == DIR_TYPE:
            # if the tree object is a directory, startup the rename directory widget
            renameDialog = QtGui.QInputDialog()
            renameDialog.setFocus()
            newName, ok = renameDialog.getText(self, 'Rename Directory', 'Enter new name:')
            if ok:
                if newName == '' or '/' in newName:
                    warnings.warn("rename failed: the new name is an empty string or contains /; directory not renamed", RuntimeWarning)
                    return
                else:
                    depth = self.getTreeItemDepth(treeWidgetItem)
                    level_list = actor_tree_widget_ID.split('/')
                    level_list[depth] = str(newName)
                    new_actor_tree_widget_ID = '/'.join(level_list)
                    # we should check if this directory already exists before recursing through structure and renaming all children
                    actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                    if len(actor_tree_widget_list) > 0:
                        warnings.warn("rename failed: a directory of that ID already exists; directory not renamed", RuntimeWarning)
                        return
                    treeWidgetItem.setText(0, newName)
                    # recurse through children of this tree widget, renaming the corresponding directory in its ID
                    self.treeItemRecurseRenameID(treeWidgetItem, newName, depth)
                        
    def treeItemRecurseRenameID(self, treeWidgetItem, newName, depth):
        # recurse through the tree, renaming all children of this directory
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        level_list = actor_tree_widget_ID.split('/')
        level_list[depth] = str(newName)
        new_actor_tree_widget_ID = '/'.join(level_list)
        treeWidgetItem.setText(1, new_actor_tree_widget_ID)
        for i in xrange(treeWidgetItem.childCount()):
            child_tree_widget = treeWidgetItem.child(i)
            self.treeItemRecurseRenameID(child_tree_widget, newName, depth)

    def getTreeItemDepth(self, treeWidgetItem):
        # returns the depth from the root of a tree object
        depth = 0
        treeWidgetItem = treeWidgetItem.parent()
        while treeWidgetItem is not None:
            treeWidgetItem = treeWidgetItem.parent()
            depth += 1
        return depth

    def treeItemRemove(self, treeWidgetItem):
        # recursively remove a tree object from the tree
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            actor = tree_object.actor
            self.vtk_main_canvas.removeActor(actor)
            axes = tree_object.axes
            self.vtk_main_canvas.removeActorFrameAxes(axes)
            del self.tree_widget_items_to_objects[treeWidgetItem]
        elif tree_object_type == DIR_TYPE:
            # recurse through children of this tree widget, removing them
            while treeWidgetItem.childCount() != 0:
                child_tree_widget = treeWidgetItem.child(0)
                self.treeItemRemove(child_tree_widget)
            del self.tree_widget_items_to_objects[treeWidgetItem]
        parent_tree_widget = treeWidgetItem.parent()
        if parent_tree_widget is None:
            # this tree widget is seated at the top level
            self.ui.treeWidgetActors.invisibleRootItem().removeChild(treeWidgetItem)
        else:
            parent_tree_widget.removeChild(treeWidgetItem)
    ################################

    ### countertrigger functions ###
    def setBackgroundCheckMarks(self, value):
        if value:
            self.ui.actionBGLight.setChecked(True)
            self.ui.actionBGDark.setChecked(False)
        else:
            self.ui.actionBGLight.setChecked(False)
            self.ui.actionBGDark.setChecked(True)

    def setCameraSelectionCheckMarks(self, value):
        if value:
            self.ui.actionCameraPerspective.setChecked(True)
            self.ui.actionCameraTopDown.setChecked(False)
        else:
            self.ui.actionCameraPerspective.setChecked(False)
            self.ui.actionCameraTopDown.setChecked(True)

    def addActor(self, level_list, actor, add_bool=True):
        self.ui.treeWidgetActors.blockSignals(True)
        # an empty tree, or a tree which contains an empty string, or a non-string type, or string with / char, is malformed
        if len(level_list) == 0:
            warnings.warn("addActor failed: the actor tree is malformed; actor not added to the scene", RuntimeWarning)
            self.ui.treeWidgetActors.blockSignals(False)
            return
        for i in level_list:
            if i == '' or type(i) is not str or '/' in i:
                warnings.warn("addActor failed: the actor tree is malformed; actor not added to the scene", RuntimeWarning)
                self.ui.treeWidgetActors.blockSignals(False)
                return
            
        # the name of the actor is the last element of the tree
        actor_name = level_list[-1]

        # get the actor tree widget ID by concatenating all level_list elements
        actor_tree_widget_ID = '/'.join(level_list)

        # if the actor tree widget already exists, then it is invalid
        actor_tree_widget_list = self.ui.treeWidgetActors.findItems(actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
        if len(actor_tree_widget_list) > 0:
            warnings.warn("addActor failed: the actor tree is malformed because an actor of that ID already exists; actor not added to the scene", RuntimeWarning)
            self.ui.treeWidgetActors.blockSignals(False)
            return

        # recurse through the tree widget structure setting or creating parent widgets as we go
        parent_tree_widget_ID = ''
        parent_tree_widget = self.ui.treeWidgetActors
        for level in level_list[:-1]:
            parent_tree_widget_ID += level + '/'
            parent_tree_widget_list = self.ui.treeWidgetActors.findItems(parent_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
            if len(parent_tree_widget_list) > 0:
                # a parent tree widget was found!
                parent_tree_widget = parent_tree_widget_list[0]
            else:
                # no parent tree widget exists - create a new one
                new_tree_widget_item = QtGui.QTreeWidgetItem(parent_tree_widget)
                new_tree_widget_item.setText(0, level)
                new_tree_widget_item.setText(1, parent_tree_widget_ID)
                new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/folder.png'))
                new_tree_widget_item.setText(2, DIR_TYPE)
                new_tree_widget_item.setFlags(new_tree_widget_item.flags() | QtCore.Qt.ItemIsTristate | QtCore.Qt.ItemIsUserCheckable)
                new_tree_widget_item.setExpanded(True)
                parent_tree_widget = new_tree_widget_item
                parent_tree_widget.setCheckState(0, QtCore.Qt.Checked)
                # add a reference to this directory in our dict
                parent_tree_widget_item = new_tree_widget_item.parent()
                self.tree_widget_items_to_objects[new_tree_widget_item] = TreeObject(None, DIR_TYPE, None, vtk.vtkTransform())

        # add a tree widget for the actor
        new_tree_widget_item = QtGui.QTreeWidgetItem(parent_tree_widget)
        new_tree_widget_item.setText(0, actor_name)
        new_tree_widget_item.setText(1, actor_tree_widget_ID)
        if actor.GetClassName() == "vtkAxesActor":
            new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/axes.png'))
            new_tree_widget_item.setText(2, AXES_TYPE)
            tree_object_type = AXES_TYPE
        elif actor.GetClassName() == "vtkTexturedActor2D" or actor.GetClassName() == "vtkTextActor":
            new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/billboard.png'))
            new_tree_widget_item.setText(2, BILLBOARD_TYPE)
            tree_object_type = BILLBOARD_TYPE
        else:
            new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/object.png'))
            new_tree_widget_item.setText(2, ACTOR_TYPE)
            tree_object_type = ACTOR_TYPE
        new_tree_widget_item.setCheckState(0, QtCore.Qt.Checked)

        # add the actor to our dict mapping tree widgets to actors
        parent_tree_widget_item = new_tree_widget_item.parent()
        self.tree_widget_items_to_objects[new_tree_widget_item] = TreeObject(actor, tree_object_type, None, vtk.vtkTransform())

        # finally, all checks passed, so add the actor to the vtk scene
        if add_bool:
            self.vtk_main_canvas.addActor(new_tree_widget_item, actor)
            # add an axes for this actor to indicate the orientation of its frame
            self.addActorFrameAxes(new_tree_widget_item)
            # determine the actor's/axes' transform based on its parents/grandparents
            self.treeItemComposeTransform(new_tree_widget_item)
        self.ui.treeWidgetActors.blockSignals(False)

    def addActorFrameAxes(self, tree_widget_item):
        # add the axes to the vtk scene
        axes = self.vtk_main_canvas.addActorFrameAxes(tree_widget_item)

        # add the axes to our dict mapping tree widgets to actors
        self.tree_widget_items_to_objects[tree_widget_item].axes = axes
    ################################

class MainFrame(QtGui.QFrame):
    ''' the Qt4 frame that holds the main VTK canvas '''

    # class attributes - setup Qt4 callback signals
    background_light_signal = QtCore.pyqtSignal(bool)
    camera_perspective = False
    camera_perspective_signal = QtCore.pyqtSignal(bool)
    add_actor_signal = QtCore.pyqtSignal(list, object)
    add_orientation_axes_signal = QtCore.pyqtSignal(list, object, bool)

    def __init__(self, parent):
        super(MainFrame,self).__init__(parent)
        # dictionary to keep track of tree widget items that correspond to actors/icons - used to handle vtk->Qt4 scene changes
        self.actors_to_tree_widget_items = {}

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
        self.addOrientationAxes()
        
        # setup the default base grid - 1x1km, 10m squares
        self.addGrid(['grids', '1km x km, 10m'], 1000, 10)

        ''' TESTING ACTOR TYPES!!! '''
        points = np.random.uniform(-10,10,(5000,3))
        colors = np.random.uniform(0,255,(5000,3))
        colors = colors.astype(np.uint8)
        actor1 = Primitives.PointCloud(points, colors)
        actor1.GetProperty().SetPointSize(4)
        self.add_actor_signal.emit(['point clouds','1'], actor1)

        points = np.random.uniform(-10,10,(20,3))
        colors = np.random.uniform(0,255,(20,3))
        colors = colors.astype(np.uint8)
        actor2 = Primitives.TriangleStrip(points, colors)
        self.add_actor_signal.emit(['triangle strip','1'], actor2)

        actor3 = Primitives.Model("./fishobj/trout.obj", "./fishobj/RB_TROUT.JPG")
        self.add_actor_signal.emit(['models','trout'], actor3)

        # t = vtk.vtkTransform()
        # t2 = vtk.vtkTransform()
        # actor1.SetUserTransform(t2)
        # t.Translate(50,0,0)
        # t2.Concatenate(transformations.translation_matrix([50, 0, 0]).flatten().tolist())
        # print t
        # print t2
        # t.RotateZ(90)
        # t2.Concatenate(transformations.euler_matrix(math.radians(0), math.radians(0), math.radians(90)).flatten().tolist())
        # print t
        # print t2
        # t.Translate(50,0,0)
        # t2.Concatenate(transformations.translation_matrix([50, 0, 0]).flatten().tolist())
        # print t
        # print t2
        # t.RotateZ(-90)
        # t2.Concatenate(transformations.euler_matrix(math.radians(0), math.radians(0), math.radians(-90)).flatten().tolist())
        # print t
        # print t2
        # t.Translate(50,0,0)
        # t2.Concatenate(transformations.translation_matrix([50, 0, 0]).flatten().tolist())
        # print t
        # print t2

        actor2 = Primitives.CustomImageBillboard("./fishobj/RB_TROUT.JPG", 64, 64)
        self.add_actor_signal.emit(['icons','trout'], actor2)

        actor3 = Primitives.CustomTextBillboard("sandshark_auv", 20)
        self.add_actor_signal.emit(['icons','sandshark_auv'], actor3)
        
        # actor4 = Primitives.TexturedQuad(np.array([[-2001,-2000.5,-0.1],[1999,-2000.5,-0.1],[1999,1979.5,-0.1],[-2001,1979.5,-0.1]], dtype=np.float), "./map_images/MIT_aerial_x_-2001_2000.5_y_-2000.5_1979.5.png")
        # self.add_actor_signal.emit(['texture','quad'], actor4)

        # actor1.SetPosition(10,0,0)
        # print actor1.GetPosition()
        # m0 = transformations.translation_matrix([0,0,-20])
        # m1 = transformations.translation_matrix([20,0,0])
        # # m2 = transformations.euler_matrix(math.radians(0), math.radians(45), math.radians(0))
        # # m = m0.dot(m2)
        # t = actor1.GetUserTransform()
        # print actor1
        # t.PostMultiply()
        # # # t.Concatenate(m2.flatten().tolist())
        # t.Concatenate(m1.flatten().tolist())
        # # # t.Concatenate(m0.flatten().tolist())
        # # t.Concatenate(m.flatten().tolist())
        # # actor1.SetUserTransform(t)
        # print actor1.GetUserTransform()

        # actor = Primitives.Axes()
        # actor.AxisLabelsOff()
        # TransformUtils.ApplyTransformationEuler(actor, 20, 20, 0, 0, 45, 0, order=0)
        # self.add_actor_signal.emit(['asdf','test_axes'], actor)

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

    def addOrientationAxes(self):
        axes = Primitives.Axes()
        self.orientation_axes = vtk.vtkOrientationMarkerWidget()
        self.orientation_axes.SetOrientationMarker(axes)
        self.orientation_axes.SetInteractor(self.vtk_interactor)
        self.orientation_axes.EnabledOn()
        self.orientation_axes.InteractiveOff()
        self.add_orientation_axes_signal.emit(['orientation axes'], axes, False)

    def defaultPerspectiveCamera(self):
        self.camera_perspective_signal.emit(True)
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
        self.camera_perspective_signal.emit(True)
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
        self.camera_perspective_signal.emit(False)
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
        self.camera_perspective_signal.emit(False)
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
        self.background_light_signal.emit(True)
        self.vtk_renderer.SetBackground2(220.0/255,225.0/255,235.0/255)
        self.vtk_renderer.SetBackground(105.0/255,135.0/255,155.0/255)
        self.vtk_interactor.Render()

    def setBackgroundDark(self):
        self.background_light_signal.emit(False)
        self.vtk_renderer.SetBackground(0.0/255,10.0/255,15.0/255)
        self.vtk_renderer.SetBackground2(60.0/255,80.0/255,110.0/255)
        self.vtk_interactor.Render()

    def addGrid(self, level_list, full_length, cell_length):
        actor = Primitives.Grid(full_length, cell_length)
        self.add_actor_signal.emit(level_list, actor)

    def addAxes(self, level_list):
        actor = Primitives.Axes()
        self.add_actor_signal.emit(level_list, actor)

    def addArrow(self, level_list, res):
        actor = Primitives.Arrow(res)
        self.add_actor_signal.emit(level_list, actor)

    def addBox(self, level_list, x, y, z):
        actor = Primitives.Box(x, y, z)
        self.add_actor_signal.emit(level_list, actor)

    def addSphere(self, level_list, r, t_res, p_res):
        actor = Primitives.Sphere(r, t_res, p_res)
        self.add_actor_signal.emit(level_list, actor)

    def addCylinder(self, level_list, r, h, res):
        actor = Primitives.Cylinder(r, h, res)
        self.add_actor_signal.emit(level_list, actor)

    def addEllipsoid(self, level_list, xr, yr, zr):
        actor = Primitives.Ellipsoid(xr, yr, zr)
        self.add_actor_signal.emit(level_list, actor)

    def addCone(self, level_list, r, h, res):
        actor = Primitives.Cone(r, h, res)
        self.add_actor_signal.emit(level_list, actor)

    def addTorus(self, level_list, ring_r, cross_section_r):
        actor = Primitives.Torus(ring_r, cross_section_r)
        self.add_actor_signal.emit(level_list, actor)

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

    def setOffset(self, actor, x_offset, y_offset, z_offset):
        if actor.__class__ == Billboards.TextBillboard or actor.__class__ == Billboards.ImageBillboard:
            # billboards are unaffected by offset changes
            pass
        else:
            actor.SetPosition(x_offset,y_offset,z_offset)

    def setOrientation(self, actor, roll, pitch, yaw):
        actor.SetOrientation(roll, pitch, yaw)



if __name__ == "__main__":

    # Recompile ui
    with open("LightFieldVisualizer.ui") as ui_file:
        with open("LightFieldVisualizer_ui.py","w") as py_ui_file:
            uic.compileUi(ui_file,py_ui_file)

    app = QtGui.QApplication([])
    main_window = MainApp()
    main_window.show()
    main_window.start()
    app.exec_()