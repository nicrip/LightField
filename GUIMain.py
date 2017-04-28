#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
main UI window
'''

''' standard libs '''
import math
import warnings
import numpy as np

''' VTK and PyQt4 '''
import vtk
from PyQt4 import QtCore, QtGui, uic

''' custom libs '''
from GUIVTKCanvas import VTKCanvas
import Primitives
import Billboards
import TransformUtils
import GUIPrimitiveDialog
import GUITransformDialog
import GUIOffsetOrientationDialog
import GUIActorPropertiesDock
import Status

DIR_TYPE = 'directory'
ACTOR_TYPE = 'actor'
BILLBOARD_TYPE = 'billboard'
AXES_TYPE = 'axes'

class TreeObject(object):
    ''' object that holds actor information in the Qt4 tree '''
    def __init__(self):
        self.object_type = None     # DIR_TYPE, ACTOR_TYPE, BILLBOARD_TYPE, AXES_TYPE

        self.actor = None           # VTK Actor
        self.actor_type = None      # Actor type (e.g. Primitives.PRIMITIVE_BOX)
        self.actor_visible = None
        self.alpha = None
        self.point_size = None
        self.line_width = None
        self.scale = None
        self.mode = None
        self.color = None
        self.offset = None
        self.orientation = None
        self.transform = None

        self.axes = None            # VTK Actor for its origin represented as an Axes
        self.axes_scale = None
        self.axes_visible = None

class MainApp(QtGui.QMainWindow):
    ''' the main Qt4 window '''

    # API signals
    add_dir_signal = QtCore.pyqtSignal(list)
    add_actor_signal = QtCore.pyqtSignal(list, object, str)
    set_actor_signal = QtCore.pyqtSignal(list, object, str)
    remove_actor_signal = QtCore.pyqtSignal(list)
    remove_dir_signal = QtCore.pyqtSignal(list)
    set_actor_offset_orientation_signal = QtCore.pyqtSignal(list, object, object)
    set_actor_transform_signal = QtCore.pyqtSignal(list, object, object, object)
    apply_actor_transform_signal = QtCore.pyqtSignal(list, object, object, object)
    reset_actor_transform_signal = QtCore.pyqtSignal(list)
    set_directory_offset_orientation_signal = QtCore.pyqtSignal(list, object, object)
    set_directory_transform_signal = QtCore.pyqtSignal(list, object, object, object)
    apply_directory_transform_signal = QtCore.pyqtSignal(list, object, object, object)
    reset_directory_transform_signal = QtCore.pyqtSignal(list)
    set_actor_color_signal = QtCore.pyqtSignal(list, object)
    set_actor_alpha_signal = QtCore.pyqtSignal(list, float)
    set_actor_scale_signal = QtCore.pyqtSignal(list, float)
    set_actor_pointsize_signal = QtCore.pyqtSignal(list, float)
    set_actor_linewidth_signal = QtCore.pyqtSignal(list, float)
    set_actor_visibility_signal = QtCore.pyqtSignal(list, bool)
    set_actor_mode_signal = QtCore.pyqtSignal(list, str)

    # GUI signals
    background_light_signal = QtCore.pyqtSignal(bool)
    camera_perspective_signal = QtCore.pyqtSignal(bool)

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
        self.vtk_main_canvas = VTKCanvas(self.ui.vtk_panel, self)
        self.ui.vtk_layout = QtGui.QHBoxLayout()
        self.ui.vtk_layout.addWidget(self.vtk_main_canvas)
        self.ui.vtk_layout.setContentsMargins(0,0,0,0)
        self.ui.vtk_panel.setLayout(self.ui.vtk_layout)

        # setup triggers for background shade selection
        self.ui.actionBGLight.triggered.connect(self.vtk_main_canvas.setBackgroundLight)
        self.ui.actionBGDark.triggered.connect(self.vtk_main_canvas.setBackgroundDark)
        self.background_light_signal.connect(self.setBackgroundCheckMarks)

        # setup triggers for camera selection
        self.ui.actionCameraPerspective.triggered.connect(self.vtk_main_canvas.perspectiveCamera)
        self.ui.actionCameraTopDown.triggered.connect(self.vtk_main_canvas.topDownCamera)
        self.camera_perspective_signal.connect(self.setCameraSelectionCheckMarks)

        # setup trigger for camera reset
        self.ui.actionCameraReset.triggered.connect(self.vtk_main_canvas.resetCamera)

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
        self.ui.treeWidgetActors.selectionModel().selectionChanged.connect(self.treeItemSelected)

        # setup API triggers
        self.add_dir_signal.connect(self.addDirectory)
        self.add_actor_signal.connect(self.addActor)
        self.set_actor_signal.connect(self.setActor)
        self.remove_actor_signal.connect(self.removeActor)
        self.remove_dir_signal.connect(self.removeDirectory)
        self.set_actor_offset_orientation_signal.connect(self.setActorOffsetOrientation)
        self.set_actor_transform_signal.connect(self.setActorTransform)
        self.apply_actor_transform_signal.connect(self.applyActorTransform)
        self.reset_actor_transform_signal.connect(self.resetActorTransform)
        self.set_directory_transform_signal.connect(self.setDirectoryTransform)
        self.apply_directory_transform_signal.connect(self.applyDirectoryTransform)
        self.reset_directory_transform_signal.connect(self.resetDirectoryTransform)
        self.set_actor_color_signal.connect(self.setActorColor)
        self.set_actor_alpha_signal.connect(self.setActorOpacity)
        self.set_actor_scale_signal.connect(self.setActorScale)
        self.set_actor_pointsize_signal.connect(self.setActorPointSize)
        self.set_actor_linewidth_signal.connect(self.setActorLineWidth)
        self.set_actor_visibility_signal.connect(self.setActorVisibility)
        self.set_actor_mode_signal.connect(self.setActorMode)

    def start(self, timer_update=False, timer_fps=30):
        # startup the vtk canvas
        if timer_update:
            self.vtk_main_canvas.setupTimerCallback(timer_fps)
        self.vtk_main_canvas.start()



    #####################################################
    ### trigger functions for GUI object manipulation ###
    #####################################################
    def treeItemSelected(self):
        # on selection of a tree widget item
        treeWidgetItems = self.ui.treeWidgetActors.selectedItems()
        if len(treeWidgetItems) == 0:
            # no object selected, return
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
        # type of tree widget item - actor or directory
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            self.ui.actorPropertiesDock.display('actor', tree_object)
        else:
            self.ui.actorPropertiesDock.display('empty')

    def treeItemChanged(self, treeWidgetItem):
        # on modification of a tree widget item (i.e. visibility checkbox)
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
            tree_object.actor_visible = actor_visible
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
        if tree_object_type != DIR_TYPE:
            offsetOrientationAction = menu.addAction("Set Offset/Orientation")
            offsetOrientationAction.triggered.connect(lambda: self.treeItemSetOffsetOrientation(treeWidgetItem))
            menu.addSeparator()
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
            self.vtk_main_canvas.setActorOffset(actor, x_offset, y_offset, z_offset)
            self.vtk_main_canvas.setActorOrientation(actor, roll, pitch, yaw)
            tree_object.offset = [x_offset, y_offset, z_offset]
            tree_object.orientation = [roll, pitch, yaw]

    def treeItemApplyTransform(self, treeWidgetItem):
        # startup the apply transformation widget
        transformDialog = GUITransformDialog.TransformDialog()
        transformDialog.exec_()
        return_dict = transformDialog.return_dict
        if len(return_dict) > 0:
            if return_dict['stack'] == 1:
                stack = True
            else:
                stack = False
            self.treeItemRecurseApplyTransform(treeWidgetItem, return_dict['xtranslate'], return_dict['ytranslate'], return_dict['ztranslate'], return_dict['rollrotate'], return_dict['pitchrotate'], return_dict['yawrotate'], return_dict['order'], stack)

    def treeItemRecurseApplyTransform(self, treeWidgetItem, x_translate, y_translate, z_translate, roll_rotate, pitch_rotate, yaw_rotate, order, stack):
            tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
            if stack:
                # stacking transformations composes transforms through the tree structure
                tree_object.transform = TransformUtils.ApplyTransformationEuler(tree_object.transform, x_translate, y_translate, z_translate, roll_rotate, pitch_rotate, yaw_rotate, order)
            else:
                # not stacking sets transforms through the tree structure
                self.treeItemRecurseResetTransform(treeWidgetItem)
                tree_object.transform = TransformUtils.ApplyTransformationEuler(tree_object.transform, x_translate, y_translate, z_translate, roll_rotate, pitch_rotate, yaw_rotate, order)
            self.vtk_main_canvas.requestUpdate(None, None)  # need to request update, because transform changes to directories do not directly modify actors in the scene

    def treeItemRecurseResetTransform(self, treeWidgetItem):
        # reset the transform associated with this tree object, then determine the full transform of all the objects affected by this change
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object.transform.Identity()
        parent_tree_widget_item = treeWidgetItem.parent()
        if parent_tree_widget_item is not None:
            parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget_item]
            tree_object.transform.PostMultiply()
            tree_object.transform.Concatenate(parent_tree_object.transform)
        self.vtk_main_canvas.requestUpdate(None, None)  # need to request update, because transform changes to directories do not directly modify actors in the scene

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
            self.addGrid(new_grid_level_list, new_grid_length, new_grid_cell_length)
        elif return_type == Primitives.PRIMITIVE_AXES:
            new_axes_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_axes_level_list = new_axes_name.split('/')
            self.addAxes(new_axes_level_list)
        elif return_type == Primitives.PRIMITIVE_ARROW:
            new_arrow_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_arrow_level_list = new_arrow_name.split('/')
            new_arrow_res = primitiveDialog.return_dict['res']   
            self.addArrow(new_arrow_level_list, new_arrow_res)
        elif return_type == Primitives.PRIMITIVE_BOX:
            new_box_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_box_level_list = new_box_name.split('/')
            new_box_x = primitiveDialog.return_dict['x']
            new_box_y = primitiveDialog.return_dict['y']
            new_box_z = primitiveDialog.return_dict['z']
            self.addBox(new_box_level_list, new_box_x, new_box_y, new_box_z)
        elif return_type == Primitives.PRIMITIVE_SPHERE:
            new_sphere_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_sphere_level_list = new_sphere_name.split('/')
            new_sphere_radius = primitiveDialog.return_dict['radius']
            new_sphere_tres = primitiveDialog.return_dict['tres']
            new_sphere_pres = primitiveDialog.return_dict['pres']
            self.addSphere(new_sphere_level_list, new_sphere_radius, new_sphere_tres, new_sphere_pres)
        elif return_type == Primitives.PRIMITIVE_CYLINDER:
            new_cylinder_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_cylinder_level_list = new_cylinder_name.split('/')
            new_cylinder_radius = primitiveDialog.return_dict['radius']
            new_cylinder_height = primitiveDialog.return_dict['height']
            new_cylinder_res = primitiveDialog.return_dict['res']            
            self.addCylinder(new_cylinder_level_list, new_cylinder_radius, new_cylinder_height, new_cylinder_res)
        elif return_type == Primitives.PRIMITIVE_ELLIPSOID:
            new_ellipsoid_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_ellipsoid_level_list = new_ellipsoid_name.split('/')
            new_ellipsoid_xradius = primitiveDialog.return_dict['xradius']
            new_ellipsoid_yradius = primitiveDialog.return_dict['yradius']
            new_ellipsoid_zradius = primitiveDialog.return_dict['zradius']
            self.addEllipsoid(new_ellipsoid_level_list, new_ellipsoid_xradius, new_ellipsoid_yradius, new_ellipsoid_zradius)
        elif return_type == Primitives.PRIMITIVE_CONE:
            new_cone_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_cone_level_list = new_cone_name.split('/')
            new_cone_radius = primitiveDialog.return_dict['radius']
            new_cone_height = primitiveDialog.return_dict['height']
            new_cone_res = primitiveDialog.return_dict['res']            
            self.addCone(new_cone_level_list, new_cone_radius, new_cone_height, new_cone_res)
        elif return_type == Primitives.PRIMITIVE_TORUS:
            new_torus_name = actor_tree_widget_ID + primitiveDialog.return_dict['name']
            new_torus_level_list = new_torus_name.split('/')
            new_torus_ring_radius = primitiveDialog.return_dict['ringradius']
            new_torus_cs_radius = primitiveDialog.return_dict['csradius']
            self.addTorus(new_torus_level_list, new_torus_ring_radius, new_torus_cs_radius)

    def treeItemAddDirectory(self, treeWidgetItem):
        # startup the add directory widget
        self.ui.treeWidgetActors.blockSignals(True)
        actor_tree_widget_ID = str(treeWidgetItem.text(1))
        newDirDialog = QtGui.QInputDialog()
        newDirDialog.setFocus()
        dirName, ok = newDirDialog.getText(self, 'Add Directory', 'Enter directory name:')
        if ok:
            if dirName == '' or '/' in dirName:
                warn_str = "add directory failed: the new directory name is an empty string or contains /: " + str(dirName) + "; directory not added"
                warnings.warn(warn_str, RuntimeWarning)
                self.ui.treeWidgetActors.blockSignals(False)
                return
            else:
                new_actor_tree_widget_ID = actor_tree_widget_ID + str(dirName) + '/'
                # we should check if this directory already exists before adding it
                actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                if len(actor_tree_widget_list) > 0:
                    warn_str = "add directory failed: a directory of that ID: " + str(dirName) + " already exists; directory not added"
                    warnings.warn(warn_str, RuntimeWarning)
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
                new_tree_object = TreeObject()
                new_tree_object.object_type = DIR_TYPE
                new_tree_object.transform = vtk.vtkTransform()
                parent_tree_widget_item = new_tree_widget_item.parent()
                if parent_tree_widget_item is not None:
                    parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget_item]
                    new_tree_object.transform.PostMultiply()
                    new_tree_object.transform.Concatenate(parent_tree_object.transform)
                self.tree_widget_items_to_objects[new_tree_widget_item] = new_tree_object
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
                    warn_str = "rename failed: the new name is an empty string or contains /: " + str(newName) + "; actor not renamed"
                    warnings.warn(warn_str, RuntimeWarning)
                    return
                else:
                    level_list = actor_tree_widget_ID.split('/')
                    level_list[-1] = str(newName)
                    new_actor_tree_widget_ID = '/'.join(level_list)
                    actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                    if len(actor_tree_widget_list) > 0:
                        warn_str = "rename failed: an actor of that ID: " + str(level_list) + " already exists; actor not renamed"
                        warnings.warn(warn_str, RuntimeWarning)
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
                    warn_str = "rename failed: the new name is an empty string or contains /: " + str(newName) + "; directory not renamed"
                    warnings.warn(warn_str, RuntimeWarning)
                    return
                else:
                    depth = self.getTreeItemDepth(treeWidgetItem)
                    level_list = actor_tree_widget_ID.split('/')
                    level_list[depth] = str(newName)
                    new_actor_tree_widget_ID = '/'.join(level_list)
                    # we should check if this directory already exists before recursing through structure and renaming all children
                    actor_tree_widget_list = self.ui.treeWidgetActors.findItems(new_actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
                    if len(actor_tree_widget_list) > 0:
                        warn_str = "rename failed: a directory of that ID: " + str(level_list) + " already exists; directory not renamed"
                        warnings.warn(warn_str, RuntimeWarning)
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



    ##########################################################
    ### convenience functions for add primitive dialog box ###
    ##########################################################
    def addGrid(self, level_list, full_length, cell_length):
        actor = Primitives.Grid(full_length, cell_length)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_GRID)

    def addAxes(self, level_list):
        actor = Primitives.Axes()
        self.addActor(level_list, actor, Primitives.PRIMITIVE_AXES)

    def addArrow(self, level_list, res):
        actor = Primitives.Arrow(res)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_ARROW)

    def addBox(self, level_list, x, y, z):
        actor = Primitives.Box(x, y, z)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_BOX)

    def addSphere(self, level_list, r, t_res, p_res):
        actor = Primitives.Sphere(r, t_res, p_res)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_SPHERE)

    def addCylinder(self, level_list, r, h, res):
        actor = Primitives.Cylinder(r, h, res)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_CYLINDER)

    def addEllipsoid(self, level_list, xr, yr, zr):
        actor = Primitives.Ellipsoid(xr, yr, zr)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_ELLIPSOID)

    def addCone(self, level_list, r, h, res):
        actor = Primitives.Cone(r, h, res)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_CONE)

    def addTorus(self, level_list, ring_r, cross_section_r):
        actor = Primitives.Torus(ring_r, cross_section_r)
        self.addActor(level_list, actor, Primitives.PRIMITIVE_TORUS)



    ###################################################################
    ### convenience function for VTKCanvas special orientation axes ###
    ###################################################################
    def addOrientationAxes(self):
        axes = Primitives.Axes()
        self.orientation_axes = vtk.vtkOrientationMarkerWidget()
        self.orientation_axes.SetOrientationMarker(axes)
        self.orientation_axes.SetInteractor(self.vtk_main_canvas.vtk_interactor)
        self.orientation_axes.EnabledOn()
        self.orientation_axes.InteractiveOff()
        self.addActor(['orientation axes'], axes, Primitives.PRIMITIVE_AXES, False)



    ############################################
    ### trigger functions for LightField API ###
    ############################################
    def _defaultActorObject(self, actor, actor_type, object_type):
        # setup TreeObject default properties for an actor
        default_tree_object = TreeObject()
        default_tree_object.actor = actor
        default_tree_object.actor_type = actor_type
        default_tree_object.actor_visible = True
        default_tree_object.object_type = object_type
        default_tree_object.alpha = 1.0
        default_tree_object.point_size = 1.0
        default_tree_object.line_width = 1.0
        default_tree_object.scale = 1.0
        default_tree_object.mode = 'Surface'
        default_tree_object.color = [1.0, 1.0, 1.0]
        default_tree_object.offset = [0.0, 0.0, 0.0]
        default_tree_object.orientation = [0.0, 0.0, 0.0]
        default_tree_object.transform = vtk.vtkTransform()
        return default_tree_object

    def _getActorTreeObjectFromLevelList(self, level_list):
        # given a level list, return the associated actor TreeObject, if it exists
        actor_tree_widget_ID = '/'.join(level_list)
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            return None
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == ACTOR_TYPE or tree_object_type == BILLBOARD_TYPE or tree_object_type == AXES_TYPE:
            return tree_object

    def _getActorTreeWidgetItemFromLevelList(self, level_list):
        # given a level list, return the associated actor tree widget, if it exists
        actor_tree_widget_ID = '/'.join(level_list)
        tree_widget_list = self.ui.treeWidgetActors.findItems(actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
        if len(tree_widget_list) > 0:
            treeWidgetItem = tree_widget_list[0]
            return treeWidgetItem
        warn_str = "failed: an actor tree widget with that level list ID: " + str(level_list) + " does not exist"
        warnings.warn(warn_str, RuntimeWarning)
        return None

    def _getDirectoryTreeObjectFromLevelList(self, level_list):
        # given a level list, return the associated directory TreeObject, if it exists
        actor_tree_widget_ID = '/'.join(level_list) + '/'
        treeWidgetItem = self._getDirectoryTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            return None
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        if tree_object_type == DIR_TYPE:
            return tree_object

    def _getDirectoryTreeWidgetItemFromLevelList(self, level_list):
        # given a level list, return the associated directory tree widget, if it exists
        actor_tree_widget_ID = '/'.join(level_list) + '/'
        tree_widget_list = self.ui.treeWidgetActors.findItems(actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
        if len(tree_widget_list) > 0:
            treeWidgetItem = tree_widget_list[0]
            return treeWidgetItem
        warn_str = "failed: a directory tree widget with that level list ID: " + str(level_list) + " does not exist"
        warnings.warn(warn_str, RuntimeWarning)
        return None

    def addActor(self, level_list, actor, actor_type, add_bool=True):
        # given a level list, actor, and actor type, add the associated tree widget to the GUI and add the actor to the VTK scene
        self.ui.treeWidgetActors.blockSignals(True)
        # an empty tree, or a tree which contains an empty string, or a non-string type, or string with / char, is malformed
        if len(level_list) == 0:
            warn_str = "addActor failed: the actor tree is malformed, it might be empty: " + str(level_list) + "; actor not added to the scene"
            warnings.warn(warn_str, RuntimeWarning)
            self.ui.treeWidgetActors.blockSignals(False)
            self.emit(QtCore.SIGNAL('addActorStatus'), Status.MALFORMED_PATH)
            return
        for i in level_list:
            if i == '' or type(i) is not str or '/' in i:
                warn_str = "addActor failed: the actor tree is malformed, it might contain unsupported characters: " + str(i) + "; actor not added to the scene"
                warnings.warn(warn_str, RuntimeWarning)
                self.ui.treeWidgetActors.blockSignals(False)
                self.emit(QtCore.SIGNAL('addActorStatus'), Status.MALFORMED_PATH)
                return
            
        # the name of the actor is the last element of the tree
        actor_name = level_list[-1]

        # get the actor tree widget ID by concatenating all level_list elements
        actor_tree_widget_ID = '/'.join(level_list)

        # if the actor tree widget already exists, then it is invalid
        actor_tree_widget_list = self.ui.treeWidgetActors.findItems(actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
        if len(actor_tree_widget_list) > 0:
            warn_str = "addActor failed: the actor tree is malformed because an actor at that level list already exists: " + str(level_list) + "; actor not added to the scene"
            warnings.warn(warn_str, RuntimeWarning)
            self.ui.treeWidgetActors.blockSignals(False)
            self.emit(QtCore.SIGNAL('addActorStatus'), Status.EXISTING_PATH)
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
                new_tree_object = TreeObject()
                new_tree_object.object_type = DIR_TYPE
                new_tree_object.transform = vtk.vtkTransform()
                if parent_tree_widget_item is not None:
                    parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget_item]
                    new_tree_object.transform.PostMultiply()
                    new_tree_object.transform.Concatenate(parent_tree_object.transform)
                self.tree_widget_items_to_objects[new_tree_widget_item] = new_tree_object

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
        new_tree_object = self._defaultActorObject(actor, actor_type, tree_object_type)
        self.tree_widget_items_to_objects[new_tree_widget_item] = new_tree_object

        # finally, all checks passed, so add the actor to the vtk scene
        if add_bool:
            self.vtk_main_canvas.addActor(new_tree_widget_item, actor)
            # add an axes for this actor to indicate the orientation of its frame
            self.addActorFrameAxes(new_tree_widget_item)
            # setup actor and axes transforms
            parent_tree_widget_item = new_tree_widget_item.parent()
            if parent_tree_widget_item is not None:
                parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget_item]
                new_tree_object.transform.PostMultiply()
                new_tree_object.transform.Concatenate(parent_tree_object.transform)
                new_tree_object.actor.SetUserTransform(new_tree_object.transform)
                new_tree_object.axes.SetUserTransform(new_tree_object.transform)
        self.ui.treeWidgetActors.blockSignals(False)
        self.emit(QtCore.SIGNAL('addActorStatus'), Status.OK)

    def addActorFrameAxes(self, tree_widget_item):
        # add the axes to the vtk scene
        axes = self.vtk_main_canvas.addActorFrameAxes(tree_widget_item)
        # add the axes to our dict mapping tree widgets to actors
        self.tree_widget_items_to_objects[tree_widget_item].axes = axes
        self.tree_widget_items_to_objects[tree_widget_item].axes_scale = 1.0
        self.tree_widget_items_to_objects[tree_widget_item].axes_visible = False

    def setActor(self, level_list, actor, actor_type):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActor failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorStatus'), Status.NONEXISTING_PATH)
            return
        old_actor = tree_object.actor
        self.vtk_main_canvas.replaceActor(old_actor, treeWidgetItem, actor)
        tree_object.actor = actor
        tree_object.actor_type = actor_type
        tree_object.actor.SetUserTransform(tree_object.transform)
        self.vtk_main_canvas.setActorVisibility(tree_object.actor, tree_object.actor_visible)
        self.vtk_main_canvas.setActorOpacity(tree_object.actor, tree_object.alpha)
        self.vtk_main_canvas.setActorPointSize(tree_object.actor, tree_object.point_size)
        self.vtk_main_canvas.setActorLineWidth(tree_object.actor, tree_object.line_width)
        self.vtk_main_canvas.setActorScale(tree_object.actor, tree_object.scale)
        self.vtk_main_canvas.setActorColor(tree_object.actor, tree_object.color[0], tree_object.color[1], tree_object.color[2])
        self.vtk_main_canvas.setActorOffset(tree_object.actor, tree_object.offset[0], tree_object.offset[1], tree_object.offset[2])
        self.vtk_main_canvas.setActorOrientation(tree_object.actor, tree_object.orientation[0], tree_object.orientation[1], tree_object.orientation[2])
        if tree_object.mode == 'Surface':
            self.vtk_main_canvas.setActorToSurface(tree_object.actor)
        elif tree_object.mode == 'Wireframe':
            self.vtk_main_canvas.setActorToWireframe(tree_object.actor)
        elif tree_object.mode == 'Surface & Edges':
            self.vtk_main_canvas.setActorToSurfaceEdges(tree_object.actor)
        elif tree_object.mode == 'Points':
            self.vtk_main_canvas.setActorToPoints(tree_object.actor)
        self.emit(QtCore.SIGNAL('setActorStatus'), Status.OK)

    def removeActor(self, level_list):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "removeActor failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('removeActorStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        self.vtk_main_canvas.removeActor(actor)
        axes = tree_object.axes
        self.vtk_main_canvas.removeActorFrameAxes(axes)
        del self.tree_widget_items_to_objects[treeWidgetItem]
        parent_tree_widget = treeWidgetItem.parent()
        if parent_tree_widget is None:
            self.ui.treeWidgetActors.invisibleRootItem().removeChild(treeWidgetItem)
        else:
            parent_tree_widget.removeChild(treeWidgetItem)
        self.emit(QtCore.SIGNAL('removeActorStatus'), Status.OK)

    def addDirectory(self, level_list):
        for name in level_list:
            if name == '' or '/' in name:
                warn_str = "addDirectory failed: the new directory level list contains an empty string or contains /: " + str(level_list) +  "; directory not added"
                warnings.warn(warn_str, RuntimeWarning)
                self.emit(QtCore.SIGNAL('addDirectoryStatus'), Status.MALFORMED_PATH)
                return
        actor_tree_widget_ID = '/'.join(level_list) + '/'
        actor_tree_widget_list = self.ui.treeWidgetActors.findItems(actor_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
        if len(actor_tree_widget_list) > 0:
            warn_str = "addDirectory failed: a directory at that level list already exists: " + str(level_list) + "; directory not added"
            warnings.warn(warn_str, RuntimeWarning)
            self.ui.treeWidgetActors.blockSignals(False)
            self.emit(QtCore.SIGNAL('addDirectoryStatus'), Status.EXISTING_PATH)
            return
        self.ui.treeWidgetActors.blockSignals(True)
        parent_tree_widget_ID = ''
        parent_tree_widget = self.ui.treeWidgetActors
        for level in level_list:
            parent_tree_widget_ID += level + '/'
            parent_tree_widget_list = self.ui.treeWidgetActors.findItems(parent_tree_widget_ID, QtCore.Qt.MatchExactly|QtCore.Qt.MatchRecursive, 1)
            if len(parent_tree_widget_list) > 0:
                parent_tree_widget = parent_tree_widget_list[0]
            else:
                new_tree_widget_item = QtGui.QTreeWidgetItem(parent_tree_widget)
                new_tree_widget_item.setText(0, level)
                new_tree_widget_item.setText(1, parent_tree_widget_ID)
                new_tree_widget_item.setIcon(2, QtGui.QIcon('icons/folder.png'))
                new_tree_widget_item.setText(2, DIR_TYPE)
                new_tree_widget_item.setFlags(new_tree_widget_item.flags() | QtCore.Qt.ItemIsTristate | QtCore.Qt.ItemIsUserCheckable)
                new_tree_widget_item.setExpanded(True)
                parent_tree_widget = new_tree_widget_item
                parent_tree_widget.setCheckState(0, QtCore.Qt.Checked)
                parent_tree_widget_item = new_tree_widget_item.parent()
                new_tree_object = TreeObject()
                new_tree_object.object_type = DIR_TYPE
                new_tree_object.transform = vtk.vtkTransform()
                if parent_tree_widget_item is not None:
                    parent_tree_object = self.tree_widget_items_to_objects[parent_tree_widget_item]
                    new_tree_object.transform.PostMultiply()
                    new_tree_object.transform.Concatenate(parent_tree_object.transform)
                self.tree_widget_items_to_objects[new_tree_widget_item] = new_tree_object
        self.ui.treeWidgetActors.blockSignals(False)
        self.emit(QtCore.SIGNAL('addDirectoryStatus'), Status.OK)

    def removeDirectory(self, level_list):
        treeWidgetItem = self._getDirectoryTreeWidgetItemFromLevelList(level_list)
        tree_object = self._getDirectoryTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "removeDirectory failed: a directory does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('removeDirectoryStatus'), Status.NONEXISTING_PATH)
            return
        tree_object = self.tree_widget_items_to_objects[treeWidgetItem]
        tree_object_type = tree_object.object_type
        # recurse through children of this tree widget, removing them
        while treeWidgetItem.childCount() != 0:
            child_tree_widget = treeWidgetItem.child(0)
            self.treeItemRemove(child_tree_widget)
        del self.tree_widget_items_to_objects[treeWidgetItem]
        parent_tree_widget = treeWidgetItem.parent()
        if parent_tree_widget is None:
            self.ui.treeWidgetActors.invisibleRootItem().removeChild(treeWidgetItem)
        else:
            parent_tree_widget.removeChild(treeWidgetItem)
        self.emit(QtCore.SIGNAL('removeDirectoryStatus'), Status.OK)

    def setActorOffsetOrientation(self, level_list, offset, orientation_euler):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorOffsetOrientation failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorOffsetOrientationStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseSetOffsetOrientation(treeWidgetItem, offset[0], offset[1], offset[2], orientation_euler[0], orientation_euler[1], orientation_euler[2])
        self.emit(QtCore.SIGNAL('setActorOffsetOrientationStatus'), Status.OK)

    def setActorTransform(self, level_list, translation, rotationeuler, order):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "setActorTransform failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseApplyTransform(treeWidgetItem, translation[0], translation[1], translation[2], rotationeuler[0], rotationeuler[1], rotationeuler[2], order, False)
        self.emit(QtCore.SIGNAL('setActorTransformStatus'), Status.OK)

    def applyActorTransform(self, level_list, translation, rotationeuler, order):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "applyActorTransform failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('applyActorTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseApplyTransform(treeWidgetItem, translation[0], translation[1], translation[2], rotationeuler[0], rotationeuler[1], rotationeuler[2], order, True)
        self.emit(QtCore.SIGNAL('applyActorTransformStatus'), Status.OK)

    def resetActorTransform(self, level_list):
        treeWidgetItem = self._getActorTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "resetActorTransform failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('resetActorTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseResetTransform(treeWidgetItem)
        self.emit(QtCore.SIGNAL('resetActorTransformStatus'), Status.OK)

    def setDirectoryTransform(self, level_list, translation, rotationeuler, order):
        treeWidgetItem = self._getDirectoryTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "setDirectoryTransform failed: a directory does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setDirectoryTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseApplyTransform(treeWidgetItem, translation[0], translation[1], translation[2], rotationeuler[0], rotationeuler[1], rotationeuler[2], order, False)
        self.emit(QtCore.SIGNAL('setDirectoryTransformStatus'), Status.OK)

    def applyDirectoryTransform(self, level_list, translation, rotationeuler, order):
        treeWidgetItem = self._getDirectoryTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "applyDirectoryTransform failed: a directory does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('applyDirectoryTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseApplyTransform(treeWidgetItem, translation[0], translation[1], translation[2], rotationeuler[0], rotationeuler[1], rotationeuler[2], order, True)
        self.emit(QtCore.SIGNAL('applyDirectoryTransformStatus'), Status.OK)

    def resetDirectoryTransform(self, level_list):
        treeWidgetItem = self._getDirectoryTreeWidgetItemFromLevelList(level_list)
        if treeWidgetItem is None:
            warn_str = "resetDirectoryTransform failed: a directory does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('resetDirectoryTransformStatus'), Status.NONEXISTING_PATH)
            return
        self.treeItemRecurseResetTransform(treeWidgetItem)
        self.emit(QtCore.SIGNAL('resetDirectoryTransformStatus'), Status.OK)

    def setActorVisibility(self, level_list, visibility):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorVisible failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorVisibleStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorVisibility(actor, visibility)
            tree_object.actor_visible = visibility
        self.emit(QtCore.SIGNAL('setActorVisibleStatus'), Status.OK)

    def setActorOpacity(self, level_list, opacity):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorOpacity failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorOpacityStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorOpacity(actor, opacity)
            tree_object.alpha = opacity
        self.emit(QtCore.SIGNAL('setActorOpacityStatus'), Status.OK)

    def setActorPointSize(self, level_list, point_size):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorScale failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorPointSizeStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorPointSize(actor, point_size)
            tree_object.point_size = point_size
        self.emit(QtCore.SIGNAL('setActorPointSizeStatus'), Status.OK)

    def setActorLineWidth(self, level_list, line_width):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorScale failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorLineWidthStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorLineWidth(actor, line_width)
            tree_object.line_width = line_width
        self.emit(QtCore.SIGNAL('setActorLineWidthStatus'), Status.OK)

    def setActorScale(self, level_list, scale):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorScale failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorScaleStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorScale(actor, scale)
            tree_object.scale = scale
        self.emit(QtCore.SIGNAL('setActorScaleStatus'), Status.OK)

    def setActorMode(self, level_list, mode):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorScale failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorModeStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            if mode == 'Surface':
                self.vtk_main_canvas.setActorToSurface(actor)
                tree_object.mode = mode
            elif mode == 'Wireframe':
                self.vtk_main_canvas.setActorToWireframe(actor)
                tree_object.mode = mode
            elif mode == 'Surface & Edges':
                self.vtk_main_canvas.setActorToSurfaceEdges(actor)
                tree_object.mode = mode
            elif mode == 'Points':
                self.vtk_main_canvas.setActorToPoints(actor)
                tree_object.mode = mode
            else:
                self.vtk_main_canvas.setActorToSurface(actor)
                tree_object.mode = 'Surface'
        self.emit(QtCore.SIGNAL('setActorModeStatus'), Status.OK)

    def setActorColor(self, level_list, rgb):
        tree_object = self._getActorTreeObjectFromLevelList(level_list)
        if tree_object is None:
            warn_str = "setActorColor failed: an actor does not exist at the level list: " + str(level_list)
            warnings.warn(warn_str, RuntimeWarning)
            self.emit(QtCore.SIGNAL('setActorColorStatus'), Status.NONEXISTING_PATH)
            return
        actor = tree_object.actor
        if actor is not None:
            self.vtk_main_canvas.setActorColor(actor, rgb[0], rgb[1], rgb[2])
            tree_object.color = [rgb[0], rgb[1], rgb[2]]
        self.emit(QtCore.SIGNAL('setActorColorStatus'), Status.OK)