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

        self.analysisDir = os.path.join(constants.rootFolderPath, 'smartsted', 'analysis_pipelines')
        self.transformDir = os.path.join(constants.rootFolderPath, 'smartsted', 'transform_pipelines')
        
        if not os.path.exists(self.analysisDir):
            os.makedirs(self.analysisDir)
        
        # add all available analysis pipelines to the dropdown list
        self.analysisPipelines = list()
        self.analysisPipelinePar = QtGui.QComboBox()
        for pipeline in os.listdir(self.analysisDir):
            if os.path.isfile(os.path.join(self.analysisDir, pipeline)):
                pipeline = pipeline.split('.')[0]
                self.analysisPipelines.append(pipeline)
        
        self.analysisPipelinePar.addItems(self.analysisPipelines)
        self.analysisPipelinePar.setCurrentIndex(0)
        
        self.transformPipelines = list()
        self.transformPipelinePar = QtGui.QComboBox()
        for transform in os.listdir(self.transformDir):
            if os.path.isfile(os.path.join(self.transformDir, transform)):
                transform = transform.split('.')[0]
                self.transformPipelines.append(transform)
        
        self.transformPipelinePar.addItems(self.transformPipelines)
        self.transformPipelinePar.setCurrentIndex(0)

        self.param_names = list()
        self.param_edits = list()

        self.initiateButton = guitools.BetterPushButton('Initiate smartSTED')
        self.initiateButton.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Expanding)
        self.loadPipelineButton = guitools.BetterPushButton('Load pipeline')
        
        self.coordTransfCalibButton = guitools.BetterPushButton('Transform calibration')
        self.recordBinaryMaskButton = guitools.BetterPushButton('Transform calibration')

        self.endlessScanCheck = QtGui.QCheckBox('Endless')
        self.visualizeOnlyCheck = QtGui.QCheckBox('Visualize only')

        self.bin_thresh_label = QtGui.QLabel('Bin. threshold')
        self.bin_thresh_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.bin_thresh_edit = QtGui.QLineEdit(str(100))
        self.bin_smooth_label = QtGui.QLabel('Bin. threshold')
        self.bin_smooth_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.bin_smooth_edit = QtGui.QLineEdit(str(2))

        self.im_param_label = QtGui.QLabel('ROI parameters')
        self.im_param_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.im_size_label = QtGui.QLabel('ROI size (µm)')
        self.im_size_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.im_size_edit = QtGui.QLineEdit(str(3))
        self.px_size_label = QtGui.QLabel('Step size (µm)')
        self.px_size_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.px_size_edit = QtGui.QLineEdit(str(0.03))
        self.dw_time_label = QtGui.QLabel('Dwell time (ms)')
        self.dw_time_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.dw_time_edit = QtGui.QLineEdit(str(0.01))

        self.coordTransformWidget = CoordTransformWidget(*args, **kwargs)
        self.coordTransformWidget.initControls()

        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)
    
    def initControls(self, *args):
        currentRow = 0

        # add general buttons to grid
        self.grid.addWidget(self.initiateButton, currentRow, 0)
        self.grid.addWidget(self.endlessScanCheck, currentRow, 1)
        self.grid.addWidget(self.visualizeOnlyCheck, currentRow, 2)
        self.grid.addWidget(self.recordBinaryMaskButton, currentRow, 5)
        
        currentRow += 2

        # add image and pixel size parameters to grid
        # add param name and param to grid
        self.grid.addWidget(self.im_param_label, currentRow, 0)
        self.grid.addWidget(self.im_size_label, currentRow-1, 1)
        self.grid.addWidget(self.px_size_label, currentRow-1, 2)
        self.grid.addWidget(self.dw_time_label, currentRow-1, 3)
        self.grid.addWidget(self.bin_thresh_label, currentRow-1, 4)
        self.grid.addWidget(self.bin_smooth_label, currentRow-1, 5)
        self.grid.addWidget(self.im_size_edit, currentRow, 1)
        self.grid.addWidget(self.px_size_edit, currentRow, 2)
        self.grid.addWidget(self.dw_time_edit, currentRow, 3)
        self.grid.addWidget(self.bin_thresh_edit, currentRow, 4)
        self.grid.addWidget(self.bin_smooth_edit, currentRow, 5)

        currentRow += 1

        self.grid.addWidget(self.loadPipelineButton, currentRow, 0)
        self.grid.addWidget(self.analysisPipelinePar, currentRow, 1)
        self.grid.addWidget(self.transformPipelinePar, currentRow, 2)
        self.grid.addWidget(self.coordTransfCalibButton, currentRow, 3)

    def initParamFields(self, parameters: dict):
        # remove previous parameter fields for the previously loaded pipeline
        for param in self.param_names:
            self.grid.removeWidget(param)
        for param in self.param_edits:
            self.grid.removeWidget(param)

        # initiate parameter fields for all the parameters in the pipeline chosen
        currentRow = 3
        
        self.param_names = list()
        self.param_edits = list()
        for pipeline_param_name, pipeline_param_val in parameters.items():
            #TODO: fix bkg better, let img be a stack of previous frames maybe?
            if pipeline_param_name != 'img' and pipeline_param_name != 'bkg' and pipeline_param_name != 'binary_mask':
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

    def launchCoordTransform(self):
        self.coordTransformWidget.show()

    def hideCoordTransform(self):
        self.coordTransformWidget.hide()


class CoordTransformWidget(Widget):
    ''' Pop-up widget for the coordinate transform between the two smartSTED modalities. '''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.loadLoResButton = guitools.BetterPushButton('Load low-res calibration image')
        self.loadHiResButton = guitools.BetterPushButton('Load high-res calibration image')
        self.saveCalibButton = guitools.BetterPushButton('Save calibration')
        self.resetCoordsButton = guitools.BetterPushButton('Reset coordinates')

        self.loResVbWidget = pg.GraphicsLayoutWidget()
        self.hiResVbWidget = pg.GraphicsLayoutWidget()
        self.loResVb = self.loResVbWidget.addViewBox(row=1, col=1)
        self.hiResVb = self.hiResVbWidget.addViewBox(row=1, col=1)

        self.loResImg = guitools.OptimizedImageItem(axisOrder = 'row-major')
        self.hiResImg = guitools.OptimizedImageItem(axisOrder = 'row-major')
        self.loResImg.translate(-0.5, -0.5)
        self.hiResImg.translate(-0.5, -0.5)

        self.loResVb.addItem(self.loResImg)
        self.hiResVb.addItem(self.hiResImg)
        self.loResVb.setAspectLocked(True)
        self.hiResVb.setAspectLocked(True)
        #self.loResVb.invertY()

        self.loResScatterPlot = pg.ScatterPlotItem()
        self.hiResScatterPlot = pg.ScatterPlotItem()
        self.transformScatterPlot = pg.ScatterPlotItem()
        self.loResScatterPlot.setData
        self.hiResScatterPlot.setData
        self.transformScatterPlot.setData
        self.loResVb.addItem(self.loResScatterPlot)
        self.hiResVb.addItem(self.hiResScatterPlot)
        self.hiResVb.addItem(self.transformScatterPlot)

        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)
    
    def initControls(self, *args):
        currentRow = 0
        self.grid.addWidget(self.loadLoResButton, currentRow, 0)
        self.grid.addWidget(self.loadHiResButton, currentRow, 1)
        
        currentRow += 1
        self.grid.addWidget(self.loResVbWidget, currentRow, 0)
        self.grid.addWidget(self.hiResVbWidget, currentRow, 1)

        currentRow += 1
        self.grid.addWidget(self.saveCalibButton, currentRow, 0)
        self.grid.addWidget(self.resetCoordsButton, currentRow, 1)

