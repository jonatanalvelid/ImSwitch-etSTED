"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
from numpy.lib.function_base import asarray_chkfinite
import constants
import os
import importlib
import h5py

from datetime import datetime
from inspect import signature
from tkinter import Tk
from tkinter.filedialog import askopenfilename

import pyqtgraph as pg
import numpy as np

from .basecontrollers import WidgetController
import view.guitools as guitools

class SmartSTEDController(WidgetController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fastDetector = self._setupInfo.smartSTED.fastDetector

        self._widget.initControls(self._setupInfo.positioners, self._setupInfo.getTTLDevices())

        self.analysisDir = os.path.join(constants.rootFolderPath, 'analysispipelines')

        self.__savefolder = "C:\\DataImswitch\\logs"

        # Initiate coordinate transform parameters
        self.__transformCoeffs = np.ones(20)
        self.__loResCoords = list()
        self.__hiResCoords = list()
        self.__loResCoordsPx = list()
        self.__hiResCoordsPx = list()
        self.__hiResPxSize = 1
        self.__loResPxSize = 1
        self.__hiResSize = 1

        # Connect SmartSTEDWidget and communication channel signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)
        self._widget.coordTransfCalibButton.clicked.connect(self.calibrationLaunch)
        self._widget.coordTransformWidget.saveCalibButton.clicked.connect(self.calibrationFinish)
        self._widget.coordTransformWidget.resetCoordsButton.clicked.connect(self.resetCalibrationCoords)
        self._widget.coordTransformWidget.loadLoResButton.clicked.connect(lambda: self.loadCalibImage('lo'))
        self._widget.coordTransformWidget.loadHiResButton.clicked.connect(lambda: self.loadCalibImage('hi'))
        self._widget.coordTransformWidget.loResVb.scene().sigMouseClicked.connect(self.mouseClickedCoordTransformLo)
        self._widget.coordTransformWidget.hiResVb.scene().sigMouseClicked.connect(self.mouseClickedCoordTransformHi)

        # Create scatter plot item for sending to the viewbox while analysis is running
        self.__scatterPlot = pg.ScatterPlotItem()
        self.addScatter()
        
        self.__timelog = {
            "pipeline_start": "",
            "pipeline_end": "",
            "coord_transf_start": "",
            "coord_transf_end": "",
            "scan_start": "",
            "scan_end": ""
        }
        self.__running = False
        self.__busy = False
        self.time_curr_bef = 0

    def mouseClickedCoordTransformLo(self, event):
        #print(event.pos())
        clickPos = self._widget.coordTransformWidget.loResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__loResPxSize, 3), np.around(pos_px[1]*self.__loResPxSize, 3))
        self.__loResCoordsPx.append(pos_px)
        self.__loResCoords.append(pos)
        #print(f'X,Y: {pos}')
        #print(self.__loResCoords)
        self._widget.coordTransformWidget.loResScatterPlot.setData(x=np.array(self.__loResCoordsPx)[:,0], y=np.array(self.__loResCoordsPx)[:,1], pen=pg.mkPen(None), brush='g', symbol='x', size=25)

    def mouseClickedCoordTransformHi(self, event):
        #print(event.pos())
        clickPos = self._widget.coordTransformWidget.hiResVb.mapSceneToView(event.pos())
        pos_px = (np.around(clickPos.x()), np.around(clickPos.y()))
        pos = (np.around(pos_px[0]*self.__hiResPxSize - self.__hiResSize/2, 3), -1 * np.around(pos_px[1]*self.__hiResPxSize - self.__hiResSize/2, 3))
        self.__hiResCoordsPx.append(pos_px)
        self.__hiResCoords.append(pos)
        #print(f'X,Y: {pos}')
        #print(self.__hiResCoords)
        self._widget.coordTransformWidget.hiResScatterPlot.setData(x=np.array(self.__hiResCoordsPx)[:,0], y=np.array(self.__hiResCoordsPx)[:,1], pen=pg.mkPen(None), brush='r', symbol='x', size=25)

    def resetCalibrationCoords(self):
        self.__hiResCoords = list()
        self.__loResCoords = list()
        self.__hiResCoordsPx = list()
        self.__loResCoordsPx = list()
        self._widget.coordTransformWidget.loResScatterPlot.clear()
        self._widget.coordTransformWidget.hiResScatterPlot.clear()
        self._widget.coordTransformWidget.transformScatterPlot.clear()

    def initiate(self):
        if not self.__running:
            self.__param_vals = self.readParams()

            # Load coordinate transform pipeline of choice
            self.loadTransform()

            # Connect communication channel signals
            #self._commChannel.toggleLiveview.emit(True)
            self._commChannel.toggleBlockScanWidget.emit(False)
            self._commChannel.updateImage.connect(self.runPipeline)
            self._commChannel.endScan.connect(self.saveScan)
            self._commChannel.endScan.connect(self.continueFastModality)
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(True))

            self.__scatterPlot.show()
            self._widget.initiateButton.setText('Stop')
            self.__running = True
        else:
            # Disconnect communication channel signals
            #self._commChannel.toggleLiveview.emit(False)
            self._commChannel.toggleBlockScanWidget.emit(True)
            self._commChannel.updateImage.disconnect(self.runPipeline)
            self._commChannel.endScan.disconnect(self.saveScan)
            self._commChannel.endScan.disconnect(self.continueFastModality)
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(False))
            
            self.__param_vals = list()
            self.__scatterPlot.hide()
            self._widget.initiateButton.setText('Initiate')
            self.__running = False
        
    def saveScan(self):
        self.__timelog["scan_end"] = datetime.now().strftime('%Ss%fus')
        self._commChannel.snapImage.emit()
        # save log file with temporal info of trigger event
        filename = datetime.utcnow().strftime('%Hh%Mm%Ss%fus')
        name = os.path.join(self.__savefolder, filename) + '_log'
        savename = guitools.getUniqueName(name)
        timelog = [f'{key}: {self.__timelog[key]}' for key in self.__timelog]
        with open(f'{savename}.txt', 'w') as f:
            [f.write(f'{st}\n') for st in timelog]

    def pauseFastModality(self):
        if self.__running:
            self._commChannel.updateImage.disconnect(self.runPipeline)
            #self._commChannel.toggleLiveview.emit(False)
            #self._widget.initiateButton.setText('Paused')
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(False))
            #self._widget.initiateButton.setEnabled(False)
            self.__running = False
        
    def continueFastModality(self):
        if self._widget.endlessScanCheck.isChecked() and not self.__running:
            # Connect communication channel signals
            #self._commChannel.toggleLiveview.emit(True)
            self._commChannel.updateImage.connect(self.runPipeline)
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(True))

            self.__scatterPlot.show()
            self._widget.initiateButton.setText('Stop')
            self.__running = True
        elif not self._widget.endlessScanCheck.isChecked():
            self._widget.initiateButton.setText('Initiate')
            self._commChannel.endScan.disconnect(self.saveScan)
            self._commChannel.endScan.disconnect(self.continueFastModality)
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

    def calibrationLaunch(self):
        self._widget.launchCoordTransform()

    def calibrationFinish(self):
        self.coordinateTransformCalibrate()
        name = datetime.utcnow().strftime('%Hh%Mm%Ss%fus')
        filename = os.path.join(self.__savefolder, name) + '_transformCoeffs.txt'
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
        self._widget.coordTransformWidget.transformScatterPlot.setData(x=coords_transf[:,0], y=coords_transf[:,1], pen=pg.mkPen(None), brush='b', symbol='x', size=20)

    def openFolder(self):
        """ Opens current folder in File Explorer. """
        Tk().withdraw()
        filename = askopenfilename()
        return filename

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
            img_box = self._widget.coordTransformWidget.hiResImg
        elif modality == 'lo':
            img_box = self._widget.coordTransformWidget.loResImg

        img_box.setOnlyRenderVisible(True, render=False)
        img_box.setImage(img_data, autoLevels=True, autoDownsample=False)
        img_shape = np.shape(img_data)
        self.adjustFrame(img_shape[1], img_shape[0], modality)

    def adjustFrame(self, width, height, modality):
        """ Adjusts the viewbox to a new width and height. """
        if modality == 'hi':
            img_vb = self._widget.coordTransformWidget.hiResVb
            img_box = self._widget.coordTransformWidget.hiResImg
        elif modality == 'lo':
            img_vb = self._widget.coordTransformWidget.loResVb
            img_box = self._widget.coordTransformWidget.loResImg
        guitools.setBestImageLimits(img_vb, width, height)
        img_box.render()
        
    def runPipeline(self, im, init):
        if not self.__busy:
            self.__timelog["pipeline_start"] = datetime.now().strftime('%Ss%fus')

            self.__busy = True
            
            dt = datetime.now()
            self.time_curr_bef = round(dt.microsecond/1000)

            coords_detected = self.pipeline(im, *self.__param_vals)

            self.__timelog["pipeline_end"] = datetime.now().strftime('%Ss%fus')
            dt = datetime.now()
            self.time_curr_aft = round(dt.microsecond/1000)
            print(f'Time for pipeline: {self.time_curr_aft-self.time_curr_bef} ms')

            self.__busy = False
            if coords_detected.size != 0:
                self.pauseFastModality()
                self.__timelog["coord_transf_start"] = datetime.now().strftime('%Ss%fus')
                coords_center_scan = self.transform(coords_detected, self.__transformCoeffs)
                self.__timelog["coord_transf_end"] = datetime.now().strftime('%Ss%fus')
                self.__timelog["scan_start"] = datetime.now().strftime('%Ss%fus')
                self.runSlowScan(position=coords_center_scan)
            self.updateScatter(coords_detected)

    def updateScatter(self, coords):
        self.__scatterPlot.setData(x=coords[0,:], y=coords[1,:], pen=pg.mkPen(None), brush='g', symbol='x', size=25)

    def addScatter(self):
        """ Adds the scattered point from pipeline output to ImageWidget viewbox through the CommunicationChannel. """
        self.__scatterPlot.setData
        self._commChannel.addItemTovb.emit(self.__scatterPlot)

    def readParams(self):
        param_vals = list()
        for item in self._widget.param_edits:
            param_vals.append(np.float(item.text()))
        return param_vals

    def runSlowScan(self, position=[0.0,0.0,0.0]):
        dt = datetime.now()
        time_curr_bef = round(dt.microsecond/1000)
        analogParameterDict, digitalParameterDict = self.getScanParameters(position)
        time_curr_mid = round(dt.microsecond/1000)
        try:
            signalDic, scanInfoDict = self._master.scanManager.makeFullScan(
                analogParameterDict, digitalParameterDict, self._setupInfo,
                staticPositioner=False
            )
        except:
            #TODO: should raise an error here probably, but that does not crash the program.
            return
        dt = datetime.now()
        time_curr_aft = round(dt.microsecond/1000)
        print(f'Time for curve parameters: {time_curr_mid-time_curr_bef} ms')
        print(f'Time for signal curve generation: {time_curr_aft-time_curr_mid} ms')
        self._master.nidaqManager.runScan(signalDic, scanInfoDict)

    def getScanParameters(self, position):
        analogParameterDict = {
            'sample_rate': self._setupInfo.scan.stage.sampleRate,
            'return_time': self._setupInfo.scan.stage.returnTime
        }
        digitalParameterDict = {
            'sample_rate': self._setupInfo.scan.ttl.sampleRate
        }
        analogParameterDict['target_device'] = []
        analogParameterDict['axis_length'] = []
        analogParameterDict['axis_step_size'] = []
        analogParameterDict['axis_centerpos'] = []
        analogParameterDict['axis_startpos'] = []
        for index, (positionerName, positionerInfo) in enumerate(self._setupInfo.positioners.items()):
            if positionerInfo.managerProperties['scanner']:
                analogParameterDict['target_device'].append(positionerName)
                if positionerName != 'None':
                    size = float(self._widget.im_size_edit.text())
                    stepSize = float(self._widget.px_size_edit.text())
                    center = position[index]
                    if index==0:
                        center = self.addFastAxisShift(center)
                    start = 0.0
                    analogParameterDict['axis_length'].append(size)
                    analogParameterDict['axis_step_size'].append(stepSize)
                    analogParameterDict['axis_centerpos'].append(center)
                    analogParameterDict['axis_startpos'].append(start)
                else:
                    analogParameterDict['axis_length'].append(1.0)
                    analogParameterDict['axis_step_size'].append(1.0)
                    analogParameterDict['axis_centerpos'].append(0.0)
                    analogParameterDict['axis_startpos'].append(0.0)

        digitalParameterDict['target_device'] = []
        digitalParameterDict['TTL_start'] = []
        digitalParameterDict['TTL_end'] = []
        for deviceName, deviceInfo in self._setupInfo.getTTLDevices().items():
            digitalParameterDict['target_device'].append(deviceName)

            deviceStarts = '0'.split(',')
            digitalParameterDict['TTL_start'].append([
                float(deviceStart) / 1000 for deviceStart in deviceStarts if deviceStart
            ])

            deviceEnds = '1'.split(',')
            digitalParameterDict['TTL_end'].append([
                float(deviceEnd) / 1000 for deviceEnd in deviceEnds if deviceEnd
            ])

        digitalParameterDict['sequence_time'] = float(self._widget.dw_time_edit.text()) / 1000
        analogParameterDict['sequence_time'] = float(self._widget.dw_time_edit.text()) / 1000

        return analogParameterDict, digitalParameterDict

    def addFastAxisShift(self, center):
        dwell_time = float(self._widget.dw_time_edit.text()) / 1000
        px_size = float(self._widget.px_size_edit.text())
        C = np.array([-3.31703795,  3.57475083,  0.68279051])
        params = [px_size, dwell_time, 1]
        shift_compensation = np.sum(params*C)
        print(center)
        center -= shift_compensation
        print(center)
        return center

    def coordinateTransformCalibrate(self):
        """ Third-order polynomial fitting with least-squares Levenberg-Marquart algorithm.
        """
        from scipy.optimize import least_squares
        
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
         