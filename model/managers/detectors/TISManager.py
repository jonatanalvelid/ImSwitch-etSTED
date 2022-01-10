# -*- coding: utf-8 -*-
"""
Created on Fri Jan 08 14:00:00 2021

@author: jonatanalvelid
"""

import time

from datetime import datetime
import numpy as np

from .DetectorManager import (
    DetectorManager, DetectorNumberParameter, DetectorListParameter
)


class TISManager(DetectorManager):

    def __init__(self, webcamInfo, name, **_kwargs):
        self._camera = getTISObj(webcamInfo.managerProperties['cameraListIndex'])

        model = self._camera.model
        self._running = False
        self._adjustingParameters = False

        for propertyName, propertyValue in webcamInfo.managerProperties['tis'].items():
            self._camera.setPropertyValue(propertyName, propertyValue)
        
        fullShape = (self._camera.getPropertyValue('image_width'),
                     self._camera.getPropertyValue('image_height'))

        #self.startAcquisition()

        # Prepare parameters
        parameters = {
            'exposure': DetectorNumberParameter(group='Misc', value=100, valueUnits='ms', editable=True),
            'gain': DetectorNumberParameter(group='Misc', value=1, valueUnits='arb.u.', editable=True),
            'brightness': DetectorNumberParameter(group='Misc', value=1, valueUnits='arb.u.', editable=True)
        }

        super().__init__(name, fullShape, [1], model, parameters)

        self.crop(hpos=0, vpos=0, hsize=fullShape[0], vsize=fullShape[1])

    def getLatestFrame(self):
        #print('getLatestFrame')
        if not self._adjustingParameters:
            #dt = datetime.now()
            #time_curr_bef = round(dt.microsecond/1000)
            frame = self._camera.grabFrame()
            #dt = datetime.now()
            #time_curr_mid = round(dt.microsecond/1000)
            if self._camera.flipimage[0]:
                frame = np.fliplr(frame)
            if self._camera.flipimage[1]:
                frame = np.flipud(frame)
            self.__image = frame
            #dt = datetime.now()
            #time_curr_aft = round(dt.microsecond/1000)
            #print(f'Time for grab: {time_curr_mid-time_curr_bef} ms')
            #print(f'Time for flip: {time_curr_aft-time_curr_mid} ms')
        return self.__image

    def setParameter(self, name, value):
        """Sets a parameter value and returns the value.
        If the parameter doesn't exist, i.e. the parameters field doesn't
        contain a key with the specified parameter name, an error will be
        raised."""

        if name not in self._parameters:
            raise AttributeError(f'Non-existent parameter "{name}" specified')

        value = self._camera.setPropertyValue(name, value)
        return value

    def getParameter(self, name):
        """Gets a parameter value and returns the value.
        If the parameter doesn't exist, i.e. the parameters field doesn't
        contain a key with the specified parameter name, an error will be
        raised."""

        if name not in self._parameters:
            raise AttributeError(f'Non-existent parameter "{name}" specified')

        value = self._camera.getPropertyValue(name)
        return value

    def setBinning(self, binning):
        super().setBinning(binning)
    
    def getChunk(self):
        pass

    def flushBuffers(self):
        pass

    def startAcquisition(self):
        if not self._running:
            self._camera.start_live()
            self._running = True

    def stopAcquisition(self):
        if self._running:
            self._running = False
            self._camera.suspend_live()
    
    def stopAcquisitionForROIChange(self):
        if self._running:
            self._running = False
            self._camera.stop_live()

    @property
    def pixelSize(self):
        return tuple([1, 1, 1])

    def crop(self, hpos, vpos, hsize, vsize):
        #print(f"cropping! {hpos},{vpos},{hsize},{vsize}")
        def cropAction():
            #print(f'{self._camera.model}: crop frame to {hsize}x{vsize} at {hpos},{vpos}.')
            self._camera.setROI(hpos, vpos, hsize, vsize)
        
        self._performSafeCameraAction(cropAction)
        #TODO: unsure if frameStart is needed? Try without.
        # This should be the only place where self.frameStart is changed
        self._frameStart = (vpos, hpos)
        # Only place self.__shape is changed
        self.shape = (vsize, hsize)

    def show_dialog(self):
        "Manager: open camera settings dialog."
        self._camera.show_dialog()

    def _performSafeCameraAction(self, function):
        """ This method is used to change those camera properties that need
        the camera to be idle to be able to be adjusted.
        """
        self._adjustingParameters = True
        wasrunning = self._running
        if self._running:
            self.stopAcquisitionForROIChange()
        function()
        if wasrunning:
            #print('TISManager: performSafe: camera was running')
            #self._camera.cam.open()
            self.startAcquisition()
        #else:
            #print('TISManager: performSafe: camera was not running')
        self._adjustingParameters = False

        
def getTISObj(cameraId):
    try:
        lib = __import__('pyicic')
        from model.interfaces.tiscamera import CameraTIS
        print('Trying to import camera', cameraId)
        camera = CameraTIS(cameraId)
        print('Initialized TIS Camera Object, model: ', camera.model)
        return camera
    except (OSError, IndexError, ModuleNotFoundError):
        print('Initializing Mock TIS')
        from model.interfaces.mockers import MockCameraTIS
        return MockCameraTIS()
