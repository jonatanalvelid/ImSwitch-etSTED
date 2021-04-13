"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
import constants
import os
import importlib

from inspect import signature

from .basecontrollers import WidgetController
import view.guitools as guitools

class SmartSTEDController(WidgetController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._widget.initControls(self._setupInfo.positioners, self._setupInfo.getTTLDevices())

        self.analysisDir = os.path.join(constants.rootFolderPath, 'analysispipelines')

        # Connect SmartSTEDWidget signals
        self._widget.initiateButton.clicked.connect(self.initiate)
        self._widget.loadPipelineButton.clicked.connect(self.loadPipeline)

    def initiate(self):
        pipelineidx = self._widget.analysisPipelinePar.currentIndex()
        pipelinename = self._widget.analysisPipelines[pipelineidx]
        
        print(pipelinename)
        
    def loadPipeline(self):
        pipelineidx = self._widget.analysisPipelinePar.currentIndex()
        pipelinename = self._widget.analysisPipelines[pipelineidx]
        
        pipeline = getattr(importlib.import_module(f'analysispipelines.{pipelinename}'), f'{pipelinename}')
        #pipeline = importlib.import_module(f'{pipelinefile}.{pipelinename}')
        pipelineparams = signature(pipeline).parameters

        self._widget.initParamFields(pipelineparams)

