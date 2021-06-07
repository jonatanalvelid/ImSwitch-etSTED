"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
from numpy.lib.function_base import asarray_chkfinite
import constants
import os
import importlib
import time

from datetime import datetime
from inspect import signature

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

        # Connect SmartSTEDWidget and communication channel signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)
        self._widget.coordTransfCalibButton.clicked.connect(self.calibrationLaunch)

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
        self._widget.coordTransformWidget.saveCalibButton.connect(self.calibrationFinish)

    def calibrationFinish(self):
        self.__transformCoeffs = self._widget.coordTransformWidget.transformParams
        self._widget.coordTransformWidget.saveCalibButton.disconnect(self.calibrationFinish)
        self._widget.hideCoordTransform()


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

    def coordinateTransformCalibrate(self, lo_res_coords, hi_res_coords):
        """ Third-order polynomial fitting with least-squares Levenberg-Marquart algorithm.
        """
        from scipy.optimize import least_squares
        
        # prepare data and init guess
        c_init = np.hstack([np.zeros(10), np.zeros(10)])
        xdata = lo_res_coords[:].astype(np.float32)
        ydata = hi_res_coords[:].astype(np.float32)
        initguess = c_init.astype(np.float32)
        
        # fit
        res_lsq = least_squares(poly_thirdorder, initguess, args=(xdata, ydata), method='lm')
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
    
    def poly_thirdorder_ret(a, x):
        """ Use for plotting the least-squares fit results.
        """
        c1 = x[0]
        c2 = x[1]
        x_i1 = a[0]*c1**3 + a[1]*c2**3 + a[2]*c2*c1**2 + a[3]*c1*c2**2 + a[4]*c1**2 + a[5]*c2**2 + a[6]*c1*c2 + a[7]*c1 + a[8]*c2 + a[9]
        x_i2 = a[10]*c1**3 + a[11]*c2**3 + a[12]*c2*c1**2 + a[13]*c1*c2**2 + a[14]*c1**2 + a[15]*c2**2 + a[16]*c1*c2 + a[17]*c1 + a[18]*c2 + a[19]
        return (x_i1, x_i2)
         