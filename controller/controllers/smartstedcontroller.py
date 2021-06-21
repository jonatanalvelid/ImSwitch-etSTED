"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
from numpy.lib.function_base import asarray_chkfinite
import constants
import os
import ctypes
import importlib
import h5py

from datetime import datetime
from inspect import signature
from tkinter import Tk
from tkinter.filedialog import askopenfilename
from scipy.optimize import least_squares
import scipy.ndimage as ndi

import pyqtgraph as pg
import numpy as np

from .basecontrollers import WidgetController
import view.guitools as guitools

# HIGH-RES TIMING FUNCTIONS FROM https://stackoverflow.com/questions/38319606/how-can-i-get-millisecond-and-microsecond-resolution-timestamps-in-python/38319607#38319607
def micros():
    "return a timestamp in microseconds (us)"
    tics = ctypes.c_int64()
    freq = ctypes.c_int64()

    #get ticks on the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics)) 
    #get the actual freq. of the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))  
    
    t_us = tics.value*1e6/freq.value
    return t_us
    
def millis():
    "return a timestamp in milliseconds (ms)"
    tics = ctypes.c_int64()
    freq = ctypes.c_int64()

    #get ticks on the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics)) 
    #get the actual freq. of the internal ~2MHz QPC clock 
    ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq)) 
    
    t_ms = tics.value*1e3/freq.value
    return t_ms

class SmartSTEDController(WidgetController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fastDetector = self._setupInfo.smartSTED.fastDetector
        self.slowDetector = self._setupInfo.smartSTED.slowDetector
        self.fastLaser = self._setupInfo.smartSTED.fastLaser

        self._widget.initControls(self._setupInfo.positioners, self._setupInfo.getTTLDevices())

        self.analysisDir = os.path.join(constants.rootFolderPath, 'analysispipelines')

        self.__saveFolder = "C:\\DataImswitch\\logs_smartsted"

        # create a helper controller for the coordinate transform pop-out widget
        self.__coordTransformHelper = SmartSTEDCoordTransformHelper(self, self._widget.coordTransformWidget, self.__saveFolder)

        # Initiate coordinate transform coeffs
        self.__transformCoeffs = np.ones(20)

        # Connect SmartSTEDWidget and communication channel signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)
        self._widget.coordTransfCalibButton.clicked.connect(self.__coordTransformHelper.calibrationLaunch)
        self._widget.recordBinaryMaskButton.clicked.connect(self.initiateBinaryMask)
        self._widget.loadScanParametersButton.clicked.connect(self.setScanParameters)
        self._commChannel.sendScanParameters.connect(lambda analogParams, digitalParams: self.assignScanParameters(analogParams, digitalParams))

        # Create scatter plot item for sending to the viewbox while analysis is running
        self.__scatterPlot = pg.ScatterPlotItem()
        self.addScatter()
        
        self.__detLog = {
            "pipeline_start": "",
            "pipeline_end": "",
            "coord_transf_start": "",
            "fastscan_x_center": 0,
            "fastscan_y_center": 0,
            "slowscan_x_center": 0,
            "slowscan_y_center": 0
        }
        self.__running = False
        self.__busy = False
        self.__testmode = False
        self.__bkg = None
        self.__binary_mask = None
        self.__binary_stack = None
        self.__binary_frames = 10

    def initiate(self):
        if not self.__running:
            self.__param_vals = self.readParams()

            # Check if test mode, in case launch help widget
            self.__testmode = self._widget.visualizeOnlyCheck.isChecked()
            if self.__testmode:
                self.launchAnalysisHelpWidget()

            # Load coordinate transform pipeline of choice
            self.loadTransform()
            self.__transformCoeffs = self.__coordTransformHelper.getTransformCoeffs()

            # Connect communication channel signals
            #self._commChannel.toggleLiveview.emit(True)
            self._commChannel.toggleBlockScanWidget.emit(False)
            self._commChannel.updateImage.connect(self.runPipeline)
            self._commChannel.endScan.connect(self.endScan)
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(True))

            self.__scatterPlot.show()
            self._widget.initiateButton.setText('Stop')
            self.__running = True
        else:
            # Disconnect communication channel signals
            #self._commChannel.toggleLiveview.emit(False)
            self._commChannel.toggleBlockScanWidget.emit(True)
            self._commChannel.updateImage.disconnect(self.runPipeline)
            self._commChannel.endScan.disconnect(self.endScan)
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(False))
            
            self.__param_vals = list()
            self.__scatterPlot.hide()
            self._widget.initiateButton.setText('Initiate')
            self.__running = False
        
    def endScan(self):
        if self.timelapse:
            self.__detLog[f"scan_end_frame{self.frame}"] = datetime.now().strftime('%Ss%fus')
        else:
            self.__detLog[f"scan_end"] = datetime.now().strftime('%Ss%fus')
        self._commChannel.snapImage.emit(self.slowDetector)
        if self.timelapse:
            if self.frame < self.frames_tot:
                self.frame += 1
                self.runSlowScan()
            else:
                self.endRecording()
                self.continueFastModality()
        else:
            self.endRecording()
            self.continueFastModality()

    def endRecording(self):
        # save log file with temporal info of trigger event
        filename = datetime.utcnow().strftime('%Hh%Mm%Ss%fus')
        name = os.path.join(self.__saveFolder, filename) + '_log'
        savename = guitools.getUniqueName(name)
        timelog = [f'{key}: {self.__detLog[key]}' for key in self.__detLog]
        with open(f'{savename}.txt', 'w') as f:
            [f.write(f'{st}\n') for st in timelog]
        self.resetDetLog()
    
    def resetDetLog(self):
        self.__detLog = dict()
        self.__detLog = {
            "pipeline_start": "",
            "pipeline_end": "",
            "coord_transf_start": "",
            "fastscan_x_center": 0,
            "fastscan_y_center": 0,
            "slowscan_x_center": 0,
            "slowscan_y_center": 0
        }        

    def pauseFastModality(self):
        if self.__running:
            self._commChannel.updateImage.disconnect(self.runPipeline)
            #self._commChannel.toggleLiveview.emit(False)
            #self._widget.initiateButton.setText('Paused')
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(False))
            #self._widget.initiateButton.setEnabled(False)
            self.__running = False
        
    def continueFastModality(self):
        if self._widget.endlessScanCheck.isChecked() and not self.__running:
            # Connect communication channel signals
            #self._commChannel.toggleLiveview.emit(True)
            self._commChannel.updateImage.connect(self.runPipeline)
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(True))

            self.__scatterPlot.show()
            self._widget.initiateButton.setText('Stop')
            self.__running = True
        elif not self._widget.endlessScanCheck.isChecked():
            self._widget.initiateButton.setText('Initiate')
            self._commChannel.endScan.disconnect(self.endScan)
            self._commChannel.toggleBlockScanWidget.emit(True)
            self.__running = False
            self.__param_vals = list()

    def loadPipeline(self):
        pipelineidx = self._widget.analysisPipelinePar.currentIndex()
        pipelinename = self._widget.analysisPipelines[pipelineidx]
        
        self.pipeline = getattr(importlib.import_module(f'smartsted.analysis_pipelines.{pipelinename}'), f'{pipelinename}')
        pipelineparams = signature(self.pipeline).parameters

        self._widget.initParamFields(pipelineparams)

    def loadTransform(self):
        transformidx = self._widget.transformPipelinePar.currentIndex()
        transformname = self._widget.transformPipelines[transformidx]
        self.transform = getattr(importlib.import_module(f'smartsted.transform_pipelines.{transformname}'), f'{transformname}')
        
    def runPipeline(self, im, init):
        if not self.__busy:
            self.__detLog["pipeline_start"] = datetime.now().strftime('%Ss%fus')

            self.__busy = True
            
            #t_pre = datetime.now()
            t_pre = millis()
            if self.__testmode:
                coords_detected, img_ana = self.pipeline(im, self.__bkg, self.__binary_mask, self.__testmode, *self.__param_vals)
            else:
                coords_detected = self.pipeline(im, self.__bkg, self.__binary_mask, self.__testmode, *self.__param_vals)
            #t_post = datetime.now()
            t_post = millis()
            self.__detLog["pipeline_end"] = datetime.now().strftime('%Ss%fus')
            #self.time_curr_bef = round(t_pre.microsecond/1000)
            #self.time_curr_aft = round(t_post.microsecond/1000)
            #print(f'Time for pipeline: {round(t_post.microsecond/1000)-round(t_pre.microsecond/1000)} ms')
            print(f'Time for pipeline: {t_post-t_pre} ms')

            self.__busy = False
            if self.__testmode:
                self.updateScatter(coords_detected, clear=True)
                self.setAnalysisHelpImg(img_ana)
            elif coords_detected.size != 0:
                #print(self.fastDetector)
                if np.size(coords_detected) > 2:
                    coords_scan = coords_detected[0,:]
                else:
                    coords_scan = coords_detected
                print(coords_scan)
                #self.triggeredImage = im
                # TODO: consider putting this after the scan if it slows down the time until the scan, just make sure I get the correct frame (which I have here in im)
                self._commChannel.snapImage.emit(self.fastDetector)
                self.pauseFastModality()
                self.__detLog["coord_transf_start"] = datetime.now().strftime('%Ss%fus')
                coords_center_scan = self.transform(coords_scan, self.__transformCoeffs)
                self.__detLog["fastscan_x_center"] = coords_scan[0]
                self.__detLog["fastscan_y_center"] = coords_scan[1]
                self.__detLog["slowscan_x_center"] = coords_center_scan[0]
                self.__detLog["slowscan_y_center"] = coords_center_scan[1]
                self.__detLog["scan_initiate"] = datetime.now().strftime('%Ss%fus')
                self.timelapse = self._widget.timelapseScanCheck.isChecked()
                if self.timelapse:
                    self.frame = 0
                    self.frames_tot = int(self._widget.timelapse_reps_edit.text())
                self.initiateSlowScan(position=coords_center_scan)
                self.runSlowScan()
                self.updateScatter(coords_detected, clear=True)
            self.__bkg = im

    def assignScanParameters(self, analogDict, digitalDict):
        self._analogParameterDict = analogDict
        self._digitalParameterDict = digitalDict
        print(self._analogParameterDict)
        print(self._digitalParameterDict)

    def addImgBinStack(self, im):
        if self.__binary_stack is None:
            self.__binary_stack = im
        elif len(self.__binary_stack) == self.__binary_frames:
            self._commChannel.updateImage.disconnect(self.addImgBinStack)
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(False))
            self.calculateBinaryMask()
        else:
            if np.ndim(self.__binary_stack) == 2:
                self.__binary_stack = np.stack((self.__binary_stack, im))
            else:
                self.__binary_stack = np.concatenate((self.__binary_stack, [im]), axis=0)

    def initiateBinaryMask(self):
        self.__binary_stack = None
        self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(True))
        self._commChannel.updateImage.connect(self.addImgBinStack)
        self._widget.recordBinaryMaskButton.setText('Recording...')

    def calculateBinaryMask(self):
        img_mean = np.mean(self.__binary_stack,0)
        img_bin = ndi.filters.gaussian_filter(img_mean, np.float(self._widget.bin_smooth_edit.text())) 
        self.__binary_mask = np.array(img_bin > np.float(self._widget.bin_thresh_edit.text()))
        self._widget.recordBinaryMaskButton.setText('Record binary mask')
        self.setAnalysisHelpImg(self.__binary_mask)
        self.launchAnalysisHelpWidget()

    def launchAnalysisHelpWidget(self):
        self._widget.launchHelpWidget(self._widget.analysisHelpWidget, init=True)

    def setAnalysisHelpImg(self, img_ana):
        self._widget.analysisHelpWidget.img.setOnlyRenderVisible(True, render=False)
        self._widget.analysisHelpWidget.img.setImage(img_ana, autoLevels=True, autoDownsample=False)
        img_shape = np.shape(img_ana)
        guitools.setBestImageLimits(self._widget.analysisHelpWidget.imgVb, img_shape[1], img_shape[0])
        self._widget.analysisHelpWidget.img.render()

    def updateScatter(self, coords, clear=True):
        #TODO: fix so that it clears sometimes as well.
        if np.size(coords) > 0:
            self.__scatterPlot.setData(x=coords[:,0], y=coords[:,1], pen=pg.mkPen(None), brush='g', symbol='x', size=25)
            if np.size(coords) > 2:
                coord_primary = coords[0,:]
                self.__scatterPlot.addPoints(x=[coord_primary[0]], y=[coord_primary[1]], pen=pg.mkPen(None), brush='r', symbol='x', size=25)

    def addScatter(self):
        """ Adds the scatter points from pipeline output to ImageWidget viewbox through the CommunicationChannel. """
        self.__scatterPlot.setData
        self._commChannel.addItemTovb.emit(self.__scatterPlot)

    def readParams(self):
        param_vals = list()
        for item in self._widget.param_edits:
            param_vals.append(np.float(item.text()))
        return param_vals

    def initiateSlowScan(self, position=[0.0,0.0,0.0]):
        dt = datetime.now()
        time_curr_bef = round(dt.microsecond/1000)
        self.setCenterScanParameter(position)
        time_curr_mid = round(dt.microsecond/1000)
        try:
            self.signalDic, self.scanInfoDict = self._master.scanManager.makeFullScan(
                self._analogParameterDict, self._digitalParameterDict, self._setupInfo,
                staticPositioner=False
            )
        except:
            #TODO: should raise an error here probably, but that does not crash the program.
            return
        dt = datetime.now()
        time_curr_aft = round(dt.microsecond/1000)
        print(f'Time for curve parameters: {time_curr_mid-time_curr_bef} ms')
        print(f'Time for signal curve generation: {time_curr_aft-time_curr_mid} ms')

    def runSlowScan(self):
        if self.timelapse:
            print(f'Scan #{self.frame} in time lapse of {self.frames_tot} scans')
            self.__detLog[f"scan_start_frame{self.frame}"] = datetime.now().strftime('%Ss%fus')
        else:
            self.__detLog[f"scan_start"] = datetime.now().strftime('%Ss%fus')
        self._master.nidaqManager.runScan(self.signalDic, self.scanInfoDict)

    def setScanParameters(self):
        self._commChannel.requestScanParameters.emit()

    def setCenterScanParameter(self, position):
        self._analogParameterDict['axis_centerpos'] = []
        for index, (positionerName, positionerInfo) in enumerate(self._setupInfo.positioners.items()):
            if positionerInfo.managerProperties['scanner']:
                self._analogParameterDict['target_device'].append(positionerName)
                if positionerName != 'None':
                    center = position[index]
                    if index==0:
                        center = self.addFastAxisShift(center)
                    self._analogParameterDict['axis_centerpos'].append(center)
                else:
                    self._analogParameterDict['axis_centerpos'].append(0.0)      

    def addFastAxisShift(self, center):
        """ Based on second-degree curved surface fit to 2D-sampling of dwell time and pixel size induced shifts. """
        dwell_time = float(self._analogParameterDict['sequence_time'])
        px_size = float(self._analogParameterDict['axis_step_size'][0])
        #C = np.array([-3.31703795,  3.57475083,  0.68279051])  # first order plane fit
        #params = np.array([px_size, dwell_time, 1])  # for use with first order plane fit
        #TODO: add these parameters to the .json configuration file somehow?
        C = np.array([-5.06873628, -80.6978355, 104.06976744, -7.12113356, 8.0065076, 0.68227188])  # second order plane fit
        params = np.array([px_size**2, dwell_time**2, px_size*dwell_time, px_size, dwell_time, 1])  # for use with second order plane fit
        shift_compensation = np.sum(params*C)
        print(center)
        center -= shift_compensation
        print(center)
        return center


class SmartSTEDCoordTransformHelper():
    def __init__(self, smartSTEDController, coordTransformWidget, saveFolder, *args, **kwargs):

        self.smartSTEDController = smartSTEDController
        self._widget = coordTransformWidget
        self.__saveFolder = saveFolder
        
        # Initiate coordinate transform parameters
        self.__transformCoeffs = np.ones(20)
        self.__loResCoords = list()
        self.__hiResCoords = list()
        self.__loResCoordsPx = list()
        self.__hiResCoordsPx = list()
        self.__hiResPxSize = 1
        self.__loResPxSize = 1
        self.__hiResSize = 1

        # connect signals from main widget
        self._widget.saveCalibButton.clicked.connect(self.calibrationFinish)
        self._widget.resetCoordsButton.clicked.connect(self.resetCalibrationCoords)
        self._widget.loadLoResButton.clicked.connect(lambda: self.loadCalibImage('lo'))
        self._widget.loadHiResButton.clicked.connect(lambda: self.loadCalibImage('hi'))
        self._widget.loResVb.scene().sigMouseClicked.connect(self.mouseClickedCoordTransformLo)
        self._widget.hiResVb.scene().sigMouseClicked.connect(self.mouseClickedCoordTransformHi)
        
    def getTransformCoeffs(self):
        return self.__transformCoeffs

    def calibrationLaunch(self):
        self.smartSTEDController._widget.launchHelpWidget(self.smartSTEDController._widget.coordTransformWidget, init=True)

    def calibrationFinish(self):
        self.coordinateTransformCalibrate()
        name = datetime.utcnow().strftime('%Hh%Mm%Ss%fus')
        filename = os.path.join(self.__saveFolder, name) + '_transformCoeffs.txt'
        np.savetxt(fname=filename, X=self.__transformCoeffs)
        print(self.__transformCoeffs)

        # plot the resulting transformed low-res coordinates on the hi-res image
        coords_transf = []
        loResData = np.array([*self.__loResCoords]).astype(np.float32)
        for i in range(0,len(loResData)):
            pos = self.poly_thirdorder_transform(self.__transformCoeffs, loResData[i])
            pos_px = (np.around((pos[0] + self.__hiResSize/2)/self.__hiResPxSize, 0), np.around((-1 * pos[1] + self.__hiResSize/2)/self.__hiResPxSize, 0))
            coords_transf.append(pos_px)
        coords_transf = np.array(coords_transf)
        self._widget.transformScatterPlot.setData(x=coords_transf[:,0], y=coords_transf[:,1], pen=pg.mkPen(None), brush='b', symbol='x', size=20)

    def mouseClickedCoordTransformLo(self, event):
        clickPos = self._widget.loResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__loResPxSize, 3), np.around(pos_px[1]*self.__loResPxSize, 3))
        self.__loResCoordsPx.append(pos_px)
        self.__loResCoords.append(pos)
        self._widget.loResScatterPlot.setData(x=np.array(self.__loResCoordsPx)[:,0], y=np.array(self.__loResCoordsPx)[:,1], pen=pg.mkPen(None), brush='g', symbol='x', size=25)

    def mouseClickedCoordTransformHi(self, event):
        clickPos = self._widget.hiResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__hiResPxSize - self.__hiResSize/2, 3), -1 * np.around(pos_px[1]*self.__hiResPxSize - self.__hiResSize/2, 3))
        self.__hiResCoordsPx.append(pos_px)
        self.__hiResCoords.append(pos)
        self._widget.hiResScatterPlot.setData(x=np.array(self.__hiResCoordsPx)[:,0], y=np.array(self.__hiResCoordsPx)[:,1], pen=pg.mkPen(None), brush='r', symbol='x', size=25)

    def resetCalibrationCoords(self):
        self.__hiResCoords = list()
        self.__loResCoords = list()
        self.__hiResCoordsPx = list()
        self.__loResCoordsPx = list()
        self._widget.loResScatterPlot.clear()
        self._widget.hiResScatterPlot.clear()
        self._widget.transformScatterPlot.clear()

    def loadCalibImage(self, modality):
        # open gui to choose file
        img_filename = self.openFolder()
        # load img data from file
        with h5py.File(img_filename, "r") as f:
            img_key = list(f.keys())[0]
            pixelsize = f.attrs['element_size_um'][1]
            img_data = np.array(f[img_key])
            imgsize = pixelsize*np.size(img_data,0)
        # view data in corresponding viewbox
        self.updateCalibImage(img_data, modality)
        if modality == 'hi':
            self.__hiResCoords = list()
            self.__hiResPxSize = pixelsize
            self.__hiResSize = imgsize
        elif modality == 'lo':
            self.__loResCoords = list()
            self.__loResPxSize = pixelsize

    def updateCalibImage(self, img_data, modality):
        """ Update new image in the viewbox. """
        if modality == 'hi':
            img_box = self._widget.hiResImg
        elif modality == 'lo':
            img_box = self._widget.loResImg

        img_box.setOnlyRenderVisible(True, render=False)
        img_box.setImage(img_data, autoLevels=True, autoDownsample=False)
        img_shape = np.shape(img_data)
        self.adjustFrame(img_shape[1], img_shape[0], modality)

    def openFolder(self):
        """ Opens current folder in File Explorer. """
        Tk().withdraw()
        filename = askopenfilename()
        return filename

    def adjustFrame(self, width, height, modality):
        """ Adjusts the viewbox to a new width and height. """
        if modality == 'hi':
            img_vb = self._widget.hiResVb
            img_box = self._widget.hiResImg
        elif modality == 'lo':
            img_vb = self._widget.loResVb
            img_box = self._widget.loResImg
        guitools.setBestImageLimits(img_vb, width, height)
        img_box.render()

    def coordinateTransformCalibrate(self):
        """ Third-order polynomial fitting with least-squares Levenberg-Marquart algorithm.
        """
        # prepare data and init guess
        c_init = np.hstack([np.zeros(10), np.zeros(10)])
        xdata = np.array([*self.__loResCoords]).astype(np.float32)
        ydata = np.array([*self.__hiResCoords]).astype(np.float32)
        initguess = c_init.astype(np.float32)
        
        # fit
        res_lsq = least_squares(self.poly_thirdorder, initguess, args=(xdata, ydata), method='lm')
        transformCoeffs = res_lsq.x
        self.__transformCoeffs = transformCoeffs

    def poly_thirdorder(self, a, x, y):
        """ Polynomial function that will be fit in the least-squares fit. 
        """
        res = []
        for i in range(0, len(x)):
            c1 = x[i,0]
            c2 = x[i,1]
            x_i1 = a[0]*c1**3 + a[1]*c2**3 + a[2]*c2*c1**2 + a[3]*c1*c2**2 + a[4]*c1**2 + a[5]*c2**2 + a[6]*c1*c2 + a[7]*c1 + a[8]*c2 + a[9]
            x_i2 = a[10]*c1**3 + a[11]*c2**3 + a[12]*c2*c1**2 + a[13]*c1*c2**2 + a[14]*c1**2 + a[15]*c2**2 + a[16]*c1*c2 + a[17]*c1 + a[18]*c2 + a[19]
            res.append(x_i1 - y[i,0])
            res.append(x_i2 - y[i,1])
        return res
    
    def poly_thirdorder_transform(self, a, x):
        """ Use for plotting the least-squares fit results.
        """
        c1 = x[0]
        c2 = x[1]
        x_i1 = a[0]*c1**3 + a[1]*c2**3 + a[2]*c2*c1**2 + a[3]*c1*c2**2 + a[4]*c1**2 + a[5]*c2**2 + a[6]*c1*c2 + a[7]*c1 + a[8]*c2 + a[9]
        x_i2 = a[10]*c1**3 + a[11]*c2**3 + a[12]*c2*c1**2 + a[13]*c1*c2**2 + a[14]*c1**2 + a[15]*c2**2 + a[16]*c1*c2 + a[17]*c1 + a[18]*c2 + a[19]
        return (x_i1, x_i2)