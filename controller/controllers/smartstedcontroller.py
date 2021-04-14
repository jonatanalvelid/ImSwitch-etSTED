"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
import constants
import os
import importlib

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

        # Connect SmartSTEDWidget signals
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

            # Connect communication channel signals
            self._commChannel.updateImage.connect(self.runPipeline)

            self.__scatterPlot.show()
            self._widget.initiateButton.setText('Stop')
            self.__running = True
        else:
            # Disconnect communication channel signals
            self._commChannel.updateImage.disconnect(self.runPipeline)
            
            self.__param_vals = list()
            self.__scatterPlot.hide()
            self._widget.initiateButton.setText('Initiate')
            self.__running = False
        
    def loadPipeline(self):
        pipelineidx = self._widget.analysisPipelinePar.currentIndex()
        pipelinename = self._widget.analysisPipelines[pipelineidx]
        
        self.pipeline = getattr(importlib.import_module(f'analysispipelines.{pipelinename}'), f'{pipelinename}')
        pipelineparams = signature(self.pipeline).parameters

        self._widget.initParamFields(pipelineparams)

    def runPipeline(self, im, init):
        if not self.__busy:
            self.time_prev = self.time_curr_bef
            dt = datetime.now()
            self.time_curr_bef = round(dt.microsecond/1000)
            print(f'Time since last pipeline run: {self.time_curr_bef-self.time_prev} ms')
            self.__busy = True
            output = self.pipeline(im, *self.__param_vals)
            dt = datetime.now()
            self.time_curr_aft = round(dt.microsecond/1000)
            print(f'Time for pipeline: {self.time_curr_aft-self.time_curr_bef} ms')
            self.__busy = False
            self.updateScatter(output)
    
    def updateScatter(self, coords):
        self.__scatterPlot.setData(x=coords[0,:], y=coords[1,:], pen=pg.mkPen(None), brush='r', symbol='x', size=15)

    def addScatter(self):
        """ Adds the scattered point from pipeline output to ImageWidget viewbox through the CommunicationChannel. """
        self.__scatterPlot.setData
        self._commChannel.addItemTovb.emit(self.__scatterPlot)

    def readParams(self):
        param_vals = list()
        for item in self._widget.param_edits:
            param_vals.append(np.float(item.text()))
        return param_vals

