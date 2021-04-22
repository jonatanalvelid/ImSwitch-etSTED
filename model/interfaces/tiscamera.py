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
        self.cam.colorenable = 0
        self.cam.gain.auto = False
        self.cam.exposure.auto = False
        self.cam.enable_continuous_mode(True)  # image in continuous mode

    def setROI(self, hpos, vpos, hsize, vsize):
        roi_filter = self.cam.create_frame_filter('ROI'.encode('utf-8'))
        self.cam.add_frame_filter_to_device(roi_filter)
        self.cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), vpos)
        self.cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), hpos)
        self.cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), vsize)
        self.cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), hsize)

    def start_live(self):
        self.cam.start_live(show_display=False)  # start imaging
        # self.cam.enable_trigger(True)  # camera will wait for trigger
        # self.cam.send_trigger()
        #TODO: is the below really necessary? try without it
        if not self.cam.callback_registered:
            self.cam.register_frame_ready_callback()  # needed to wait for frame ready callback

    def stop_live(self):
        self.cam.stop_live()  # stop imaging

    def grabFrame(self):
        self.cam.reset_frame_ready()  # reset frame ready flag
        self.cam.send_trigger()
        # self.cam.wait_til_frame_ready(0)  # wait for frame ready due to trigger
        frame, width, height, depth = self.cam.get_image_data()
        # Prev: averaging the RGB image to a grayscale. Very slow for the big camera (2480x2048).
        #frame = np.average(frame, 2)
        # New: just take the R-component, this should anyway contain most information in both cameras. Change this if we want to look at another color, like GFP!
        frame = np.array(frame, dtype='float64')
        # Check if below is giving the right dimensions out
        #TODO: do this smarter, as I can just take every 3rd value instead of creating a reshaped 3D array and taking the first plane of that
        frame = np.reshape(frame,(height, width, depth))[:,:,0]
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
            self.shape = (property_value, self.shape[1])
        elif property_name == 'image_width':
            self.shape = (self.shape[0], property_value)
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
        else:
            print('Property', property_name, 'does not exist')
            return False
        return property_value

    def reset_properties(self):
        self.cam.reset_properties()

    def show_dialog(self):
        self.cam.show_property_dialog()
