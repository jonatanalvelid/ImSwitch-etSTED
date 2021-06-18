# -*- coding: utf-8 -*-
"""
Created on Thu Jan 07 14:11:00 2021

@author: jonatanalvelid
"""

import numpy as np
import matplotlib.pyplot as plt
from pyicic import IC_ImagingControl


class CameraTIS:
    def __init__(self, cameraNo):
        super().__init__()

        ic_ic = IC_ImagingControl.IC_ImagingControl()
        ic_ic.init_library()
        cam_names = ic_ic.get_unique_device_names()
        self.model = cam_names[cameraNo]
        self.cam = ic_ic.get_device(cam_names[cameraNo])

        self.cam.open()

        self.shape = (0,0)
        self.flipimage = (False, False)
        self.cam.colorenable = 0
        #self.cam.gain.auto = False
        #self.cam.exposure.auto = False
        self.cam.enable_continuous_mode(True)  # image in continuous mode
        self.cam.enable_trigger(False)  # camera will wait for trigger
        #TODO: is the below really necessary? try without it
        #if not self.cam.callback_registered:
        #    self.cam.register_frame_ready_callback()  # needed to wait for frame ready callback
        #self.prepare_live()

        #TODO: CHECK IF OK: removed ".encode('utf-8')" from all str in cam.function(str)s, add back in case necessary. Depends on conda environment/package versions it seems like.
        # EXAMPLE: self.roi_filter = self.cam.create_frame_filter('ROI'.encode('utf-8'))
        # initiate one and only frame filter
        self.roi_filter = self.cam.create_frame_filter('ROI')#.encode('utf-8'))
        self.cam.add_frame_filter_to_device(self.roi_filter)
        #self.cam.frame_filter_set_parameter(self.roi_filter, 'Top'.encode('utf-8'), 0)
        #self.cam.frame_filter_set_parameter(self.roi_filter, 'Left'.encode('utf-8'), 0)
        #self.cam.frame_filter_set_parameter(self.roi_filter, 'Height'.encode('utf-8'), 1000)
        #self.cam.frame_filter_set_parameter(self.roi_filter, 'Width'.encode('utf-8'), 1000)

    def setROI(self, hpos, vpos, hsize, vsize):
        hsize = max(hsize, 256)  # minimum ROI size
        vsize = max(vsize, 24)  # minimum ROI size
        #print(f'{self.model}: setROI started with {hsize}x{vsize} at {hpos},{vpos}.')
        self.cam.frame_filter_set_parameter(self.roi_filter, 'Top', vpos)
        self.cam.frame_filter_set_parameter(self.roi_filter, 'Left', hpos)
        self.cam.frame_filter_set_parameter(self.roi_filter, 'Height', vsize)
        self.cam.frame_filter_set_parameter(self.roi_filter, 'Width', hsize)
        top = self.cam.frame_filter_get_parameter(self.roi_filter, 'Top')
        left = self.cam.frame_filter_get_parameter(self.roi_filter, 'Left')
        hei = self.cam.frame_filter_get_parameter(self.roi_filter, 'Height')
        wid = self.cam.frame_filter_get_parameter(self.roi_filter, 'Width')
        #print(f'{self.model}: setROI finished, following params are set: {wid}x{hei} at {left},{top}')

    def start_live(self):
        #print(f'{self.model}: start_live started.')
        self.cam.start_live()  # start imaging
        #print(f'{self.model}: start_live finished.')

    def stop_live(self):
        #print(f'{self.model}: stop_live started.')
        self.cam.stop_live()  # stop imaging
        #print(f'{self.model}: stop_live finished.')

    def suspend_live(self):
        #print(f'{self.model}: suspend_live started.')
        self.cam.suspend_live()  # suspend imaging into prepared state
        #print(f'{self.model}: suspend_live finished.')

    def prepare_live(self):
        #print(f'{self.model}: prepare_live started.')
        self.cam.prepare_live()  # prepare prepared state for live imaging
        #print(f'{self.model}: prepare_live finished.')

    def grabFrame(self):
        #self.cam.reset_frame_ready()  # reset frame ready flag
        #self.cam.send_trigger()
        self.cam.wait_til_frame_ready(100)  # wait for frame ready
        frame, width, height, depth = self.cam.get_image_data()
        frame = np.array(frame, dtype='float64')
        # Check if below is giving the right dimensions out
        #TODO: do this smarter, as I can just take every 3rd value instead of creating a reshaped 3D array and taking the first plane of that
        frame = np.reshape(frame,(height, width, depth))[:,:,0]
       # print(np.shape(frame))
        return frame

    def setPropertyValue(self, property_name, property_value):
        # Check if the property exists.
        if property_name == "gain":
            self.cam.gain = property_value
        elif property_name == "brightness":
            self.cam.brightness = property_value
        elif property_name == "exposure":
            self.cam.exposure = property_value
        elif property_name == 'image_height':
            self.shape = (self.shape[0], property_value)
        elif property_name == 'image_width':
            self.shape = (property_value, self.shape[1])
        elif property_name == "fliplr":
            self.flipimage = (property_value, self.flipimage[1])
        elif property_name == "flipud":
            self.flipimage = (self.flipimage[0], property_value)
        else:
            print('Property', property_name, 'does not exist')
            return False
        return property_value

    def getPropertyValue(self, property_name):
        # Check if the property exists.
        if property_name == "gain":
            property_value = self.cam.gain.value
        elif property_name == "brightness":
            property_value = self.cam.brightness.value
        elif property_name == "exposure":
            property_value = self.cam.exposure.values
        elif property_name == "image_width":
            property_value = self.shape[0]
        elif property_name == "image_height":
            property_value = self.shape[1]
        elif property_name == "fliplr":
            property_value = self.flipimage[0]
        elif property_name == "flipud":
            property_value = self.flipimage[1]
        else:
            print('Property', property_name, 'does not exist')
            return False
        return property_value

    def reset_properties(self):
        self.cam.reset_properties()

    def show_dialog(self):
        self.cam.show_property_dialog()
