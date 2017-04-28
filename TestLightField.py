#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

'''
The LightField Visualizer - a VTK/PyQt4-based lightweight field robotics viewer
-------------------------------------------------------------------------------
LightField API simple usage via threading
'''

from LightFieldAPI import LightFieldAPI
import time, threading, datetime

LightFieldAPI = LightFieldAPI()

def testLightField():
    level_list = ['robot']
    LightFieldAPI.addBox(level_list + ['torso'], 1, 2, 3)
    LightFieldAPI.setActorColor(level_list + ['torso'], [0, 0, 1])
    LightFieldAPI.addBox(level_list + ['leftarm'], 0.6, 0.6, 2)
    LightFieldAPI.setActorColor(level_list + ['leftarm'], [1, 0, 0])
    LightFieldAPI.setActorOffsetOrientation(level_list + ['leftarm'], [0, 0, -0.75], [0, 0, 0])
    LightFieldAPI.setActorTransform(level_list + ['leftarm'], [0, 1.3, 1.25], [0, 0, 0])
    LightFieldAPI.addBox(level_list + ['rightarm'], 0.6, 0.6, 2)
    LightFieldAPI.setActorColor(level_list + ['rightarm'], [1, 0, 0])
    LightFieldAPI.setActorOffsetOrientation(level_list + ['rightarm'], [0, 0, -0.75], [0, 0, 0])
    LightFieldAPI.setActorTransform(level_list + ['rightarm'], [0, -1.3, 1.25], [0, 0, 0])
    LightFieldAPI.addEllipsoid(level_list + ['head', 'skull'], 0.7, 0.7, 0.8)
    LightFieldAPI.setActorColor(level_list + ['head', 'skull'], [0, 1, 0])
    LightFieldAPI.addCone(level_list + ['head', 'nose'], 0.2, 0.5, 20.0)
    LightFieldAPI.setActorColor(level_list + ['head', 'nose'], [0.8, 0.8, 0.8])
    LightFieldAPI.setActorOffsetOrientation(level_list + ['head', 'nose'], [0.8, 0, 0], [0, 0, 0])
    LightFieldAPI.addCylinder(level_list + ['head', 'ears'], 0.3, 2, 20.0)
    LightFieldAPI.setActorColor(level_list + ['head', 'ears'], [0.8, 0.8, 0.8])
    LightFieldAPI.setDirectoryTransform(level_list + ['head'], [0.0, 0, 2.5], [0, 0, 0])

    add_heading = True
    heading_count = 50
    add_arm = True
    arm_count = 25
    add_head = True
    head_count = 10
    for i in range(0,1000):
        heading_count += 1
        if heading_count == 100:
            heading_count = 0
            add_heading = not add_heading
        arm_count += 1
        if arm_count == 50:
            arm_count = 0
            add_arm = not add_arm
        head_count += 1
        if head_count == 20:
            head_count = 0
            add_head = not add_head
        if add_heading:
            LightFieldAPI.applyDirectoryTransform(level_list, [0.2, 0, 0], [0, 0, 0.9])
        else:
            LightFieldAPI.applyDirectoryTransform(level_list, [0.2, 0, 0], [0, 0, -0.9])
        if add_arm:
            LightFieldAPI.applyActorTransform(level_list + ['rightarm'], [0.0, 0, 0], [0.9, 0.9, 0])
            LightFieldAPI.applyActorTransform(level_list + ['leftarm'], [0.0, 0, 0], [-0.9, -0.9, 0])
        else:
            LightFieldAPI.applyActorTransform(level_list + ['rightarm'], [0.0, 0, 0], [-0.9, -0.9, 0])
            LightFieldAPI.applyActorTransform(level_list + ['leftarm'], [0.0, 0, 0], [0.9, 0.9, 0])
        if add_head:
            LightFieldAPI.applyDirectoryTransform(level_list + ['head'], [0.0, 0, 0], [0, 2, 0])
        else:
            LightFieldAPI.applyDirectoryTransform(level_list + ['head'], [0.0, 0, 0], [0, -2, 0])
        
        

thread = threading.Thread(target=testLightField)
thread.daemon = True
thread.start()

LightFieldAPI.start()