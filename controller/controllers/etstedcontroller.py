from numpy.lib.function_base import asarray_chkfinite
#from pyicic import IC_ImagingControl
import constants
import os
import ctypes
import importlib
import h5py
from collections import deque

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

class EtSTEDController(WidgetController):
    """ Controller for the etSTED widget and method. """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fastDetector = self._setupInfo.etSTED.fastDetector
        self.slowDetector = self._setupInfo.etSTED.slowDetector
        self.fastLaser = self._setupInfo.etSTED.fastLaser

        self._widget.initControls(self._setupInfo.positioners, self._setupInfo.getTTLDevices())

        self.analysisDir = os.path.join(constants.rootFolderPath, 'analysispipelines')

        self.__saveFolder = "C:\\DataImswitch\\logs_etsted"

        # create a helper controller for the coordinate transform pop-out widget
        self.__coordTransformHelper = EtSTEDCoordTransformHelper(self, self._widget.coordTransformWidget, self.__saveFolder)

        # Initiate coordinate transform coeffs
        self.__transformCoeffs = np.ones(20)

        # Connect EtSTEDWidget and communication channel signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)
        self._widget.coordTransfCalibButton.clicked.connect(self.__coordTransformHelper.calibrationLaunch)
        self._widget.recordBinaryMaskButton.clicked.connect(self.initiateBinaryMask)
        self._widget.loadScanParametersButton.clicked.connect(self.setScanParameters)
        self._widget.setUpdatePeriodButton.clicked.connect(self.setFastUpdatePeriod)
        self._widget.setBusyFalseButton.clicked.connect(self.setBusyFalse)
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
        # initiate flags and params
        self.__running = False
        self.__validating = False
        self.__busy = False
        self.__visualizeMode = False
        self.__validateMode = False
        self.__bkg = None
        self.__prevFrames = deque(maxlen=10)
        self.__prevAnaFrames = deque(maxlen=10)
        self.__binary_mask = None
        self.__binary_stack = None
        self.__binary_frames = 10
        self.__init_frames = 5
        self.__validationFrames = 0
        self.__frame = 0

        self.t_call = 0

    def initiate(self):
        """ Initiate or stop an etSTED experiment. """
        if not self.__running:
            self.__param_vals = self.readParams()

            # Check if visualization mode, in case launch help widget
            self.__visualizeMode = self._widget.visualizeCheck.isChecked()
            # check if validation mode
            self.__validateMode = self._widget.validateCheck.isChecked()
            if self.__visualizeMode or self.__validateMode:
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
            self.__validating = False
            self.__frame = 0
        
    def endScan(self):
        """ End an etSTED slow method scan. """
        if self.timelapse:
            self.__detLog[f"scan_end_frame{self.timelapse_frame}"] = datetime.now().strftime('%Ss%fus')
        else:
            self.__detLog[f"scan_end"] = datetime.now().strftime('%Ss%fus')
        self._commChannel.snapImage.emit(self.slowDetector)
        if self.timelapse:
            if self.timelapse_frame < self.timelapse_frames_tot:
                self.timelapse_frame += 1
                self.runSlowScan()
                return
        self.endRecording()
        self.continueFastModality()
        self.__frame = 0

    def endRecording(self):
        """ Save an etSTED slow method scan. """
        self.__detLog["pipeline"] = self.getPipelineName()
        self.logPipelineParamVals()
        # save log file with temporal info of trigger event
        filename = datetime.utcnow().strftime('%Hh%Mm%Ss%fus')
        name = os.path.join(self.__saveFolder, filename) + '_log'
        savename = guitools.getUniqueName(name)
        log = [f'{key}: {self.__detLog[key]}' for key in self.__detLog]
        with open(f'{savename}.txt', 'w') as f:
            [f.write(f'{st}\n') for st in log]
        self.resetDetLog()
    
    def logPipelineParamVals(self):
        """ Put analysis pipeline parameters in the log file. """
        param_names = list()
        for pipeline_param_name, _ in self.__pipeline_params.items():
            if pipeline_param_name not in ['img', 'bkg', 'binary_mask', 'testmode']:
                param_names.append(pipeline_param_name)      
        for name, val in zip(param_names, self.__param_vals):
            self.__detLog[f'{name}'] = val

    def resetDetLog(self):
        """ Reset the log file. """
        self.__detLog = dict()
        self.__detLog = {
            "pipeline": "",
            "pipeline_start": "",
            "pipeline_end": "",
            "coord_transf_start": "",
            "fastscan_x_center": 0,
            "fastscan_y_center": 0,
            "slowscan_x_center": 0,
            "slowscan_y_center": 0
        }

    def setFastUpdatePeriod(self):
        """ Set the update period for the fast method. """
        self.__updatePeriod = int(self._widget.update_period_edit.text())  # update period in ms
        self._master.detectorsManager.setUpdatePeriod(self.__updatePeriod)

    def pauseFastModality(self):
        """ Pause the fast method, when an event has been detected. """
        if self.__running:
            self._commChannel.updateImage.disconnect(self.runPipeline)
            #self._commChannel.toggleLiveview.emit(False)
            #self._widget.initiateButton.setText('Paused')
            self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(False))
            #self._widget.initiateButton.setEnabled(False)
            self.__running = False
        
    def continueFastModality(self):
        """ Continue the fast method, after an event scan has been performed. """
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

    def setBusyFalse(self):
        self.__busy = False

    def getPipelineName(self):
        """ Get the name of the pipeline. """
        pipelineidx = self._widget.analysisPipelinePar.currentIndex()
        pipelinename = self._widget.analysisPipelines[pipelineidx]
        return pipelinename

    def loadPipeline(self):
        """ Load the selected analysis pipeline and its parameters. """
        pipelinename = self.getPipelineName()
        self.pipeline = getattr(importlib.import_module(f'etsted.analysis_pipelines.{pipelinename}'), f'{pipelinename}')
        self.__pipeline_params = signature(self.pipeline).parameters

        self._widget.initParamFields(self.__pipeline_params)

    def loadTransform(self):
        """ Load a previously saved coordinate transform. """
        transformidx = self._widget.transformPipelinePar.currentIndex()
        transformname = self._widget.transformPipelines[transformidx]
        self.transform = getattr(importlib.import_module(f'etsted.transform_pipelines.{transformname}'), f'{transformname}')
        
    def runPipeline(self, im, init):
        """ Run the analyis pipeline, called after every fast method frame. """
        #print(f'runPipeline called: self.__busy status: {self.__busy}')
        if not self.__busy:
            t_sincelastcall = millis() - self.t_call
            #print(f'Time since last runPipeline call: {t_sincelastcall} ms')
            self.t_call = millis()

            self.__detLog["pipeline_rep_period"] = str(t_sincelastcall)
            self.__detLog["pipeline_start"] = datetime.now().strftime('%Ss%fus')

            self.__busy = True
            
            #t_pre = millis()
            if self.__visualizeMode or self.__validateMode:
                coords_detected, img_ana = self.pipeline(im, self.__bkg, self.__binary_mask, (self.__visualizeMode or self.__validateMode), *self.__param_vals)
            else:
                coords_detected = self.pipeline(im, self.__bkg, self.__binary_mask, self.__visualizeMode, *self.__param_vals)
            #t_post = millis()
            self.__detLog["pipeline_end"] = datetime.now().strftime('%Ss%fus')
            #print(f'Time for pipeline: {t_post-t_pre} ms')

            # run if the initial frames have passed
            if self.__frame > self.__init_frames:
                if self.__visualizeMode:
                    # if visualization mode
                    self.updateScatter(coords_detected, clear=True)
                    self.setAnalysisHelpImg(img_ana)
                elif self.__validateMode:
                    # if validation mode
                    self.updateScatter(coords_detected, clear=True)
                    self.setAnalysisHelpImg(img_ana)
                    if self.__validating:
                        if self.__validationFrames > 5:
                            self.saveValidationImages(prev=True, prevAna=True)
                            self.pauseFastModality()
                            self.endRecording()
                            self.continueFastModality()
                            self.__frame = 0
                            self.__validating = False
                        self.__validationFrames += 1
                    elif coords_detected.size != 0:
                        # if some events were detected
                        if np.size(coords_detected) > 2:
                            coords_scan = coords_detected[0,:]
                        else:
                            coords_scan = coords_detected[0]
                        #print(coords_scan)
                        # save detected center coordinate in the log
                        self.__detLog["fastscan_x_center"] = coords_scan[0]
                        self.__detLog["fastscan_y_center"] = coords_scan[1]
                        # save all detected coordinates in the log
                        if np.size(coords_detected) > 2:
                            for i in range(np.size(coords_detected,0)):
                                self.__detLog[f"det_coord_x_{i}"] = coords_detected[i,0]
                                self.__detLog[f"det_coord_y_{i}"] = coords_detected[i,1]
                        self.__validating = True
                        self.__validationFrames = 0
                elif coords_detected.size != 0:
                    # if some events were detected
                    if np.size(coords_detected) > 2:
                        coords_scan = coords_detected[0,:]
                    else:
                        coords_scan = coords_detected[0]
                    print(coords_scan)
                    self.__detLog["prepause"] = datetime.now().strftime('%Ss%fus')
                    self.pauseFastModality()
                    self.__detLog["postpause"] = datetime.now().strftime('%Ss%fus')

                    self.__detLog["coord_transf_start"] = datetime.now().strftime('%Ss%fus')
                    coords_center_scan = self.transform(coords_scan, self.__transformCoeffs)
                    self.__detLog["fastscan_x_center"] = coords_scan[0]
                    self.__detLog["fastscan_y_center"] = coords_scan[1]
                    self.__detLog["slowscan_x_center"] = coords_center_scan[0]
                    self.__detLog["slowscan_y_center"] = coords_center_scan[1]
                    self.__detLog["scan_initiate"] = datetime.now().strftime('%Ss%fus')
                    # save all detected coordinates in the log
                    if np.size(coords_detected) > 2:
                        for i in range(np.size(coords_detected,0)):
                            self.__detLog[f"det_coord_x_{i}"] = coords_detected[i,0]
                            self.__detLog[f"det_coord_y_{i}"] = coords_detected[i,1]

                    self.timelapse = self._widget.timelapseScanCheck.isChecked()
                    if self.timelapse:
                        self.timelapse_frame = 0
                        self.timelapse_frames_tot = int(self._widget.timelapse_reps_edit.text())

                    self.initiateSlowScan(position=coords_center_scan)
                    self.runSlowScan()

                    # update scatter plot of event coordinates in the shown fast method image
                    self.updateScatter(coords_detected, clear=True)

                    self.__prevFrames.append(im)
                    self.saveValidationImages(prev=True, prevAna=False)
                    self.__busy = False
                    return
            self.__bkg = im
            self.__prevFrames.append(im)
            if self.__validateMode:
                self.__prevAnaFrames.append(img_ana)
            self.__frame += 1

            self.__busy = False

            #t_finalizerunpipe = millis() - self.t_call
            #print(f'Time between runPipeline call and finish: {t_finalizerunpipe} ms')

    def saveValidationImages(self, prev=True, prevAna=True):
        """ Save the images of a validation experiment event detection. """
        if prev:
            idx = 0
            for img in self.__prevFrames:
                self._commChannel.snapImagePrev.emit(self.fastDetector, img, f'raw{idx}')
                idx += 1
            self.__prevFrames.clear()
        if prevAna:
            idx = 0
            for img in self.__prevAnaFrames:
                self._commChannel.snapImagePrev.emit(self.fastDetector, img, f'ana{idx}')
                idx += 1
            self.__prevAnaFrames.clear()

    def assignScanParameters(self, analogDict, digitalDict):
        """ Assign scan parameters from the scanning widget. """
        self._analogParameterDict = analogDict
        self._digitalParameterDict = digitalDict
        print(self._analogParameterDict)
        print(self._digitalParameterDict)

    def addImgBinStack(self, im):
        """ Add image to the stack of images used to calculate a binary mask of the region of interest. """
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
        """ Initiate the process of calculating a binary mask of the region of interest. """
        self.__binary_stack = None
        self._master.lasersManager.execOn(self.fastLaser, lambda l: l.setEnabled(True))
        self._commChannel.updateImage.connect(self.addImgBinStack)
        self._widget.recordBinaryMaskButton.setText('Recording...')

    def calculateBinaryMask(self):
        """ Calculate the binary mask of the region of interest. """
        img_mean = np.mean(self.__binary_stack,0)
        img_bin = ndi.filters.gaussian_filter(img_mean, np.float(self._widget.bin_smooth_edit.text())) 
        self.__binary_mask = np.array(img_bin > np.float(self._widget.bin_thresh_edit.text()))
        self._widget.recordBinaryMaskButton.setText('Record binary mask')
        self.setAnalysisHelpImg(self.__binary_mask)
        self.launchAnalysisHelpWidget()

    def launchAnalysisHelpWidget(self):
        """ Launch help widget that shows the preprocessed images in real-time. """
        self._widget.launchHelpWidget(self._widget.analysisHelpWidget, init=True)

    def setAnalysisHelpImg(self, img_ana):
        """ Set the preprocessed image in the analysis help widget. """
        self._widget.analysisHelpWidget.img.setOnlyRenderVisible(True, render=False)
        if self.__frame < self.__init_frames + 3:
            self._widget.analysisHelpWidget.img.setImage(img_ana, autoLevels=True, autoDownsample=False)
        else:
            self._widget.analysisHelpWidget.img.setImage(img_ana, autoLevels=False, autoDownsample=False)
        infotext = f'Min: {np.min(img_ana)}, max: {np.max(img_ana/10000)} (rel. change)'
        self._widget.analysisHelpWidget.info_label.setText(infotext)
        img_shape = np.shape(img_ana)
        if self.__frame < self.__init_frames + 1:
            guitools.setBestImageLimits(self._widget.analysisHelpWidget.imgVb, img_shape[1], img_shape[0])
        self._widget.analysisHelpWidget.img.render()

    def updateScatter(self, coords, clear=True):
        """ Update the scatter plot of detected event coordinates. """
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
        """ Read user-provided analysis pipeline parameter values. """
        param_vals = list()
        for item in self._widget.param_edits:
            param_vals.append(np.float(item.text()))
        return param_vals

    def initiateSlowScan(self, position=[0.0,0.0,0.0]):
        """ Initiate a STED scan. """
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
            return
        self.scanInfoDict['throw_delay'] = np.float(self._widget.throw_delay_edit.text())
        dt = datetime.now()
        time_curr_aft = round(dt.microsecond/1000)
        print(f'Time for curve parameters: {time_curr_mid-time_curr_bef} ms')
        print(f'Time for signal curve generation: {time_curr_aft-time_curr_mid} ms')

    def runSlowScan(self):
        """ Run a STED scan. """
        if self.timelapse:
            print(f'Scan #{self.timelapse_frame} in time lapse of {self.timelapse_frames_tot} scans')
            self.__detLog[f"scan_start_frame{self.timelapse_frame}"] = datetime.now().strftime('%Ss%fus')
        else:
            self.__detLog[f"scan_start"] = datetime.now().strftime('%Ss%fus')
        self._master.nidaqManager.runScan(self.signalDic, self.scanInfoDict)

    def setScanParameters(self):
        """ Load STED scan parameters from the scanning widget. """
        self._commChannel.requestScanParameters.emit()

    def setCenterScanParameter(self, position):
        """ Set the scanning center from the detect event coordinates. """
        if self._analogParameterDict:
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
        else:
            print('No analog parameter dict. Load scan parameters before initiating etSTED.')

    def addFastAxisShift(self, center):
        """ Add a scanning-method and microscope-specific shift to the fast axis scanning.
        Based on second-degree curved surface fit to 2D-sampling of dwell time and pixel size induced shifts. """
        dwell_time = float(self._analogParameterDict['sequence_time'])
        px_size = float(self._analogParameterDict['axis_step_size'][0])
        C = np.array([-5.06873628, -80.6978355, 104.06976744, -7.12113356, 8.0065076, 0.68227188])  # second order plane fit
        params = np.array([px_size**2, dwell_time**2, px_size*dwell_time, px_size, dwell_time, 1])  # for use with second order plane fit
        shift_compensation = np.sum(params*C)
        print(center)
        center -= shift_compensation
        print(center)
        return center


class EtSTEDCoordTransformHelper():
    """ Coordinate transform help widget controller. """
    def __init__(self, etSTEDController, coordTransformWidget, saveFolder, *args, **kwargs):

        self.etSTEDController = etSTEDController
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
        """ Get transformation coefficients. """
        return self.__transformCoeffs

    def calibrationLaunch(self):
        """ Launch calibration. """
        self.etSTEDController._widget.launchHelpWidget(self.etSTEDController._widget.coordTransformWidget, init=True)

    def calibrationFinish(self):
        """ Finish calibration. """
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
        """ Detect mouse clicked coordinates in the low resolution image. """
        clickPos = self._widget.loResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__loResPxSize, 3), np.around(pos_px[1]*self.__loResPxSize, 3))
        self.__loResCoordsPx.append(pos_px)
        self.__loResCoords.append(pos)
        self._widget.loResScatterPlot.setData(x=np.array(self.__loResCoordsPx)[:,0], y=np.array(self.__loResCoordsPx)[:,1], pen=pg.mkPen(None), brush='g', symbol='x', size=25)

    def mouseClickedCoordTransformHi(self, event):
        """ Detect mouse clicked coordinates in the high resolution image. """
        clickPos = self._widget.hiResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__hiResPxSize - self.__hiResSize/2, 3), -1 * np.around(pos_px[1]*self.__hiResPxSize - self.__hiResSize/2, 3))
        self.__hiResCoordsPx.append(pos_px)
        self.__hiResCoords.append(pos)
        self._widget.hiResScatterPlot.setData(x=np.array(self.__hiResCoordsPx)[:,0], y=np.array(self.__hiResCoordsPx)[:,1], pen=pg.mkPen(None), brush='r', symbol='x', size=25)

    def resetCalibrationCoords(self):
        """ Reset all selected coordinates. """
        self.__hiResCoords = list()
        self.__loResCoords = list()
        self.__hiResCoordsPx = list()
        self.__loResCoordsPx = list()
        self._widget.loResScatterPlot.clear()
        self._widget.hiResScatterPlot.clear()
        self._widget.transformScatterPlot.clear()

    def loadCalibImage(self, modality):
        """ Load low or high resolution calibration image. """
        # open gui to choose file
        img_filename = self.openFolder()
        # load img data from file
        with h5py.File(img_filename, "r") as f:
            img_key = list(f.keys())[0]
            pixelsize = f.attrs['element_size_um'][1]
            print(pixelsize)
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