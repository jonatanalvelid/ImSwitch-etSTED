"""
Created on Tue Apr 13 2021

@author: jonatanalvelid
"""
import os
from inspect import signature

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

import constants
import view.guitools as guitools
from .basewidgets import Widget

class SmartSTEDWidget(Widget):
    ''' Widget for controlling the smartSTED implementation. '''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.analysisDir = os.path.join(constants.rootFolderPath, 'analysispipelines')

        if not os.path.exists(self.analysisDir):
            os.makedirs(self.analysisDir)

        self.param_names = list()
        self.param_edits = list()

        self.initiateButton = guitools.BetterPushButton('Initiate smartSTED')
        self.initiateButton.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Expanding)
        self.loadPipelineButton = guitools.BetterPushButton('Load pipeline')
        self.initiateButton.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Expanding)

        self.endlessCheck = QtGui.QCheckBox('Endless scanning')

        self.analysisPipelinePar = QtGui.QComboBox()

        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)
    
    def initControls(self, *args):
        # add all available analysis pipelines to the dropdown list
        self.analysisPipelines = list()
        for pipeline in os.listdir(self.analysisDir):
            if os.path.isfile(os.path.join(self.analysisDir, pipeline)):
                pipeline = pipeline.split('.')[0]
                self.analysisPipelines.append(pipeline)
        
        self.analysisPipelinePar.addItems(self.analysisPipelines)
        self.analysisPipelinePar.setCurrentIndex(0)
    
        currentRow = 0

        # add general buttons to grid
        self.grid.addWidget(self.initiateButton, currentRow, 0)
        self.grid.addWidget(self.loadPipelineButton, currentRow+1, 0)
        self.grid.addWidget(self.endlessCheck, currentRow, 1)
        self.grid.addWidget(self.analysisPipelinePar, currentRow, 2)

    def initParamFields(self, parameters: dict):
        # remove previous parameter fields for the previously loaded pipeline
        for param in self.param_names:
            self.grid.removeWidget(param)
        for param in self.param_edits:
            self.grid.removeWidget(param)

        # initiate parameter fields for all the parameters in the pipeline chosen
        currentRow = 2
        
        self.param_names = list()
        self.param_edits = list()
        for pipeline_param_name, pipeline_param_val in parameters.items():
            if pipeline_param_name != 'img':
                # create param for input
                param_name = QtGui.QLabel('{}'.format(pipeline_param_name))
                param_value = pipeline_param_val.default if pipeline_param_val.default is not pipeline_param_val.empty else 0
                param_edit = QtGui.QLineEdit(str(param_value))
                # add param name and param to grid
                self.grid.addWidget(param_name, currentRow, 0)
                self.grid.addWidget(param_edit, currentRow, 1)
                # add param name and param to object list of temp widgets
                self.param_names.append(param_name)
                self.param_edits.append(param_edit)

                currentRow += 1




        