#!/usr/bin/env python

import time
import math
import serial
import string
import numpy as np
import pymoos

from LightFieldAPI import LightFieldAPI
import vtk
from vtk.util import numpy_support as nps
from PyQt4 import QtCore, QtGui, uic
import vtkVelodyneHDLPython as vv
import threading, datetime

class MOOSLightField(object):

    def __init__(self):
        ''' MOOSApp Attributes '''
        self.server_host = 'localhost'      #MOOSDB IP - eventually a .moos parser should be written for Python to do this stuff
        self.server_port = 9000             #MOOSDB Port
        self.moos_app_name = 'LightField'   #MOOSApp name
        self.time_warp = 1                  #timewarp for simulation

        self.LightFieldAPI = LightFieldAPI()

        ''' Initialize Python-MOOS Communications '''
        self.comms = pymoos.comms()
        self.comms.set_on_connect_callback(self.on_connect)
        self.comms.add_active_queue('test', self.test)
        self.comms.add_message_route_to_active_queue('test', 'TEST')        #route 'DAQ_BINARY_DATA' messages to on_new_acoustic_binary_data function
        self.comms.run(self.server_host, self.server_port, self.moos_app_name)
        pymoos.set_moos_timewarp(self.time_warp)

    def on_connect(self):
        ''' On connection to MOOSDB, register for desired MOOS variables (allows for * regex) e.g. register('variable', 'community', 'interval')
        self.comms.register('NODE_*_PING','NODE_*',0) '''
        self.comms.register('TEST', 0)
        return True

    def test(self, msg):
        msg = msg.string()
        level_list = msg.split('/')

    def startVelo(self):
        velo_target_fps = 30.0
        velo_source = vv.vtkVelodyneHDLSource()
        velo_source.Update()
        velo_vtk_data = velo_source.GetOutput()

        # mapper = vtk.vtkDataSetMapper()
        # mapper.SetInput(velo_vtk_data)
         
        # actor = vtk.vtkActor()
        # actor.SetMapper(mapper)
        # self.main_window.addActor(['velo','cloud'], actor)

        def updateVeloSource():
            next_call = time.time()
            self.LightFieldAPI.addPointCloud(['velodyne', 'point cloud'], np.zeros((1,3)))
            self.LightFieldAPI.setActorOpacity(['velodyne', 'point cloud'], 0.5)
            self.LightFieldAPI.setActorPointSize(['velodyne', 'point cloud'], 3.0)
            self.LightFieldAPI.setActorScale(['velodyne', 'point cloud'], 5.0)
            self.LightFieldAPI.setActorMode(['velodyne', 'point cloud'], 'Points')
            while True:
                velo_source.Poll()
                velo_source.UpdateInformation()

                e = velo_source.GetExecutive()
                inf = e.GetOutputInformation().GetInformationObject(0)
                numTimeSteps = e.TIME_STEPS().Length(inf)
                if not numTimeSteps:
                    time.sleep(1.0/velo_target_fps)
                    continue

                lastTimeStep = e.TIME_STEPS().Get(inf, numTimeSteps-1)
                e.SetUpdateTimeStep(0, lastTimeStep)
                #self.LightFieldAPI.addPointCloud(['velodyne', 'point cloud'], np.zeros((1,3)))
                #velo_source.SetCorrectionsFile('/home/rypkema/Workspace/RobotX_2014/VelodyneCalibrationFiles/cal2.xml')
                velo_source.Update()
                velo_vtk_data = velo_source.GetOutput()
                pointcloud = nps.vtk_to_numpy(velo_vtk_data.GetPoints().GetData())
                if velo_vtk_data.GetNumberOfPoints() > 0:
                    intensity = nps.vtk_to_numpy(velo_vtk_data.GetPointData().GetArray(0))
                    laser_id = nps.vtk_to_numpy(velo_vtk_data.GetPointData().GetArray(1))
                    azimuth = nps.vtk_to_numpy(velo_vtk_data.GetPointData().GetArray(2))
                    distance_m = nps.vtk_to_numpy(velo_vtk_data.GetPointData().GetArray(3))
                    timestamp = nps.vtk_to_numpy(velo_vtk_data.GetPointData().GetArray(4))
                    pointcloud = nps.vtk_to_numpy(velo_vtk_data.GetPoints().GetData())
                    pointcolor = self.charToColor(intensity)
                    #pointcolor = self.rangeToColor(distance_m, 0.0, 110.0)
                    #pointcolor = self.rangeToColor(laser_id, 0.0, 31.0)
                    nps.numpy_to_vtk(pointcolor)
                    self.LightFieldAPI.setPointCloud(['velodyne', 'point cloud'], pointcloud, pointcolor)
                    
                    # intens = velo_vtk_data.GetPointData().GetArray('intensity')
                    # print intens.GetRange()
                    # velo_vtk_data.GetPointData().SetActiveScalars('intensity')
                    # actor.GetMapper().SetColorModeToMapScalars()
                    # self.main_window.vtk_main_canvas.requestUpdate(None, None)
                time.sleep(1.0/velo_target_fps)

        updateThread = threading.Thread(target=updateVeloSource)
        updateThread.daemon = True
        updateThread.start()
        velo_source.Start()

    def charToColor(self, char_array):
        char_array = char_array/255.0
        red = self.red(char_array)
        green = self.green(char_array)
        blue = self.blue(char_array)
        rgb_array = np.zeros((len(char_array), 3))
        rgb_array[:,0] = red
        rgb_array[:,1] = green
        rgb_array[:,2] = blue
        return rgb_array

    def rangeToColor(self, range_array, range_min=0.0, range_max=1.0):
        range_array = (range_array - range_min)/(range_max - range_min)
        range_array[range_array < range_min] = 0.0
        range_array[range_array > range_max] = 1.0
        red = self.red(range_array)
        green = self.green(range_array)
        blue = self.blue(range_array)
        rgb_array = np.zeros((len(range_array), 3))
        rgb_array[:,0] = red
        rgb_array[:,1] = green
        rgb_array[:,2] = blue
        return rgb_array

    ###########################################
    ### conversion from single value to RGB ###
    ###########################################
    def interpolate(self, val, y0, x0, y1, x1):
        ret = (val-x0)*(y1-y0)/(x1-x0) + y0
        return ret

    def base(self, val):
        new_val = np.zeros(val.shape)
        l_m075 = (val <= -0.75)
        l_m025 = (val > -0.75) & (val <= -0.25)
        l_p025 = (val > -0.25) & (val <= 0.25)
        l_p075 = (val > 0.25) & (val <= 0.75)
        g_p075 = (val > 0.75)
        new_val[l_m075] = 0.0
        new_val[l_m025] = self.interpolate(val[l_m025], 0.0, -0.75, 1.0, -0.25)
        new_val[l_p025] = 1.0
        new_val[l_p075] = self.interpolate(val[l_p075], 1.0, 0.25, 0.0, 0.75)
        new_val[g_p075] = 0.0
        return new_val

    def red(self, gray_array):
        red = self.base(gray_array - 0.725)
        return red

    def green(self, gray_array):
        green = self.base(gray_array)
        return green

    def blue(self, gray_array):
        blue = self.base(gray_array + 0.725)
        return blue
    ###########################################
    ###########################################
    ###########################################

    def run(self):
        self.LightFieldAPI.start()

if __name__ == '__main__':
    LightField = MOOSLightField()
    LightField.startVelo()
    LightField.run()