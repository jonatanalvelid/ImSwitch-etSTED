"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
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

        # Connect SmartSTEDWidget and communication channel signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)

        # Create scatter plot item for sending to the viewbox while analysis is running
        self.__scatterPlot = pg.ScatterPlotItem()
        self.addScatter()

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
            self._commChannel.endScan.disconnect(self.continueFastModality)
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(False))
            
            self.__param_vals = list()
            self.__scatterPlot.hide()
            self._widget.initiateButton.setText('Initiate')
            self.__running = False
        
    def pauseFastModality(self):
        if self.__running:
            self._commChannel.updateImage.disconnect(self.runPipeline)
            #self._commChannel.toggleLiveview.emit(False)
            self._widget.initiateButton.setText('Paused')
            self._master.lasersManager.execOn('488', lambda l: l.setEnabled(False))
            #self._widget.initiateButton.setEnabled(False)
            self.__running = False
            # the serial command automatically does sleep until a reply is gotten, which it gets after flip is finished
            #self._master.leicadmiManager.setCS()
        
    def continueFastModality(self):
        # the serial command automatically does sleep until a reply is gotten, which it gets after flip is finished
        #self._master.leicadmiManager.setFLUO()
        #self._master.leicadmiManager.setILshutter(1)
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
            self.__busy = True
            self.time_prev = self.time_curr_bef
            dt = datetime.now()
            self.time_curr_bef = round(dt.microsecond/1000)
            print(f'Time since last pipeline run: {self.time_curr_bef-self.time_prev} ms')

            coords_detected = self.pipeline(im, *self.__param_vals)

            dt = datetime.now()
            self.time_curr_aft = round(dt.microsecond/1000)
            print(f'Time for pipeline: {self.time_curr_aft-self.time_curr_bef} ms')

            self.__busy = False
            self.updateScatter(coords_detected)
            if coords_detected.size != 0:
                self.pauseFastModality()
                coords_center_scan = self.transform(coords_detected)
                #print(coords_detected)
                #print(coords_center_scan)
                self.runSlowScan(position=coords_center_scan)

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

