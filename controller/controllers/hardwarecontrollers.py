# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 10:40:53 2020

@author: _Xavi
"""

import pickle
import time
import threading
import textwrap
import os
import json

import numpy as np

from framework import Signal, Thread, Worker, dirtools
from .basecontrollers import WidgetController
from model.managers.SLMManager import MaskMode, Direction, MaskChoice
from skimage.feature import peak_local_max
import pyqtgraph.ptime as ptime
import scipy.ndimage as ndi


class SLMController(WidgetController):
    """Linked to SLMWidget."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.slmDir = os.path.join(dirtools.UserFileDirs.Root, 'imcontrol_slm')
        if not os.path.exists(self.slmDir):
            os.makedirs(self.slmDir)

        self._widget.initSLMDisplay(self._setupInfo.slm.monitorIdx)
        # self.loadPreset(self._defaultPreset)

        # Connect CommunicationChannel signals
        self._master.slmManager.sigSLMMaskUpdated.connect(self.displayMask)

        # Connect SLMWidget signals
        self._widget.controlPanel.upButton.clicked.connect(
            lambda: self.moveMask(Direction.Up))  # change 'up' to (x,y)=(0,1)
        self._widget.controlPanel.downButton.clicked.connect(
            lambda: self.moveMask(Direction.Down))  # change 'down' to (x,y)=(0,-1)
        self._widget.controlPanel.leftButton.clicked.connect(
            lambda: self.moveMask(Direction.Left))  # change 'left' to (x,y)=(-1,0)
        self._widget.controlPanel.rightButton.clicked.connect(
            lambda: self.moveMask(Direction.Right))  # change 'right' to (x,y)=(1,0)

        self._widget.controlPanel.saveButton.clicked.connect(self.saveParams)
        self._widget.controlPanel.loadButton.clicked.connect(self.loadParams)

        self._widget.controlPanel.donutButton.clicked.connect(lambda: self.setMask(MaskMode.Donut))
        self._widget.controlPanel.tophatButton.clicked.connect(
            lambda: self.setMask(MaskMode.Tophat))

        self._widget.controlPanel.blackButton.clicked.connect(lambda: self.setMask(MaskMode.Black))
        self._widget.controlPanel.gaussianButton.clicked.connect(
            lambda: self.setMask(MaskMode.Gauss))

        self._widget.controlPanel.halfButton.clicked.connect(lambda: self.setMask(MaskMode.Half))
        self._widget.controlPanel.quadrantButton.clicked.connect(
            lambda: self.setMask(MaskMode.Quad))
        self._widget.controlPanel.hexButton.clicked.connect(lambda: self.setMask(MaskMode.Hex))
        self._widget.controlPanel.splitbullButton.clicked.connect(
            lambda: self.setMask(MaskMode.Split))

        self._widget.applyChangesButton.clicked.connect(self.applyParams)
        self._widget.sigSLMDisplayToggled.connect(self.toggleSLMDisplay)
        self._widget.sigSLMMonitorChanged.connect(self.monitorChanged)

        # Initial SLM display
        self.displayMask(self._master.slmManager.maskCombined)

    def toggleSLMDisplay(self, enabled):
        self._widget.setSLMDisplayVisible(enabled)

    def monitorChanged(self, monitor):
        self._widget.setSLMDisplayMonitor(monitor)

    def displayMask(self, maskCombined):
        """ Display the mask in the SLM display. Originates from slmPy:
        https://github.com/wavefrontshaping/slmPy """

        arr = maskCombined.image()

        # Padding: Like they do in the software
        pad = np.zeros((600, 8), dtype=np.uint8)
        arr = np.append(arr, pad, 1)

        # Create final image array
        h, w = arr.shape[0], arr.shape[1]

        if len(arr.shape) == 2:
            # Array is grayscale
            arrGray = arr.copy()
            arrGray.shape = h, w, 1
            img = np.concatenate((arrGray, arrGray, arrGray), axis=2)
        else:
            img = arr

        self._widget.updateSLMDisplay(img)

    # Button pressed functions
    def moveMask(self, direction):
        amount = self._widget.controlPanel.incrementSpinBox.value()
        mask = self._widget.controlPanel.maskComboBox.currentIndex()
        self._master.slmManager.moveMask(mask, direction, amount)
        image = self._master.slmManager.update(maskChange=True, aberChange=True, tiltChange=True)
        self.updateDisplayImage(image)
        # print(f'Move {mask} phase mask {amount} pixels {direction}.')

    def saveParams(self):
        obj = self._widget.controlPanel.objlensComboBox.currentText()
        if obj == 'No objective':
            print('You have to choose an objective from the drop down menu.')
            return
        elif obj == 'Oil':
            filename = 'info_oil.json'
        elif obj == 'Glycerol':
            filename = 'info_glyc.json'
        else:
            raise ValueError(f'Unsupported objective "{obj}"')

        slm_info_dict = self.getInfoDict(self._widget.slmParameterTree.p, self._widget.aberParameterTree.p, self._master.slmManager.getCenters())
        with open(os.path.join(self.slmDir, filename), 'w') as f:
            json.dump(slm_info_dict, f, indent=4)
        print(f'Saved SLM parameters for {obj} objective.')

    def getInfoDict(self, generalParams=None, aberParams=None, centers=None):
        state_general = None
        state_pos = None
        state_aber = None

        if generalParams != None:
            # create dict for general params
            generalparamnames = ["radius", "sigma", "rotationAngle"]
            state_general = {generalparamname: float(generalParams.param("general").param(generalparamname).value()) for generalparamname in generalparamnames}

        if aberParams != None:
            # create dict for aberration params
            masknames = ["left", "right"]
            aberparamnames = ["tilt", "tip", "defocus", "spherical", "verticalComa", "horizontalComa", "verticalAstigmatism", "obliqueAstigmatism"]
            state_aber = dict.fromkeys(masknames)
            for maskname in masknames:
                state_aber[maskname] = {aberparamname: float(aberParams.param(maskname).param(aberparamname).value()) for aberparamname in aberparamnames}

        if centers != None:
            # create dict for position params
            state_pos = dict.fromkeys(masknames)
            for maskname in masknames:
                state_pos[maskname] = {
                                    "xcenter": int(centers[maskname][0]),
                                    "ycenter": int(centers[maskname][1])
                                    }

        info_dict = {
                    "general": state_general,
                    "position": state_pos,
                    "aber": state_aber
                    }
        return info_dict

    def loadParams(self):
        obj = self._widget.controlPanel.objlensComboBox.currentText()
        if obj == 'No objective':
            print('You have to choose an objective from the drop down menu.')
            return
        elif obj == 'Oil':
            filename = 'info_oil.json'
        elif obj == 'Glycerol':
            filename = 'info_glyc.json'
        else:
            raise ValueError(f'Unsupported objective "{obj}"')

        with open(os.path.join(self.slmDir, filename), 'rb') as f:
            slm_info_dict = json.load(f)
            state_general = slm_info_dict["general"]
            state_pos = slm_info_dict["position"]
            state_aber = slm_info_dict["aber"]

        self.setParamTree(state_general=state_general, state_aber=state_aber)
        self._master.slmManager.setGeneral(state_general)
        self._master.slmManager.setCenters(state_pos)
        self._master.slmManager.setAberrations(state_aber)
        self._master.slmManager.saveState(state_general, state_pos, state_aber)
        image = self._master.slmManager.update(maskChange=True, tiltChange=True, aberChange=True)
        self.updateDisplayImage(image)
        # print(f'Loaded SLM parameters for {obj} objective.')

    def setParamTree(self, state_general, state_aber):
        generalParams = self._widget.slmParameterTree.p
        aberParams = self._widget.aberParameterTree.p

        generalparamnames = ["radius", "sigma", "rotationAngle"]
        for generalparamname in generalparamnames:
             generalParams.param("general").param(generalparamname).setValue(float(state_general[generalparamname]))

        masknames = ["left", "right"]
        aberparamnames = ["tilt", "tip", "defocus", "spherical", "verticalComa", "horizontalComa", "verticalAstigmatism", "obliqueAstigmatism"]
        for maskname in masknames:
            for aberparamname in aberparamnames:
                aberParams.param(maskname).param(aberparamname).setValue(float(state_aber[maskname][aberparamname]))

    def setMask(self, maskMode):
        mask = self._widget.controlPanel.maskComboBox.currentIndex()  # 0 = donut (left), 1 = tophat (right)
        self._master.slmManager.setMask(mask, maskMode)
        image = self._master.slmManager.update(maskChange=True)
        self.updateDisplayImage(image)
        # print("Updated image on SLM")

    def applyParams(self):
        slm_info_dict = self.getInfoDict(generalParams=self._widget.slmParameterTree.p, aberParams=self._widget.aberParameterTree.p)
        self.applyGeneral(slm_info_dict["general"])
        self.applyAberrations(slm_info_dict["aber"])
        self._master.slmManager.saveState(state_general=slm_info_dict["general"], state_aber=slm_info_dict["aber"])

    def applyGeneral(self, info_dict):
        self._master.slmManager.setGeneral(info_dict)
        image = self._master.slmManager.update(maskChange=True)
        self.updateDisplayImage(image)
        # print('Apply changes to general slm mask parameters.')

    def applyAberrations(self, info_dict):
        self._master.slmManager.setAberrations(info_dict)
        image = self._master.slmManager.update(aberChange=True)
        self.updateDisplayImage(image)
        # print('Apply changes to aberration correction masks.')

    def updateDisplayImage(self, image):
        image = np.fliplr(image.transpose())
        self._widget.img.setImage(image, autoLevels=True, autoDownsample=False)
        # print("Updated displayed image")

    # def loadPreset(self, preset):
    #    print('Loaded default SLM settings.')


class PositionerController(WidgetController):
    """ Linked to PositionerWidget."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._widget.initControls(self._setupInfo.positioners)

        # Connect CommunicationChannel signals
        self._commChannel.moveZstage.connect(lambda step: self.move('Z', step))

        # Connect PositionerWidget signals
        for positionerName in self._setupInfo.positioners.keys():
            if self._setupInfo.positioners[positionerName].managerProperties['positioner']:
                axes = self._setupInfo.positioners[positionerName].managerProperties['axisCount']
                axislabels = textwrap.wrap(self._setupInfo.positioners[positionerName].managerProperties['axisLabels'],1)
                for i in range(axes):
                    self._widget.pars['UpButton' + positionerName + axislabels[i]].pressed.connect(
                        lambda positionerName=positionerName, axislabel=axislabels[i], i=i: self.move(
                            positionerName,
                            float(self._widget.pars['StepEdit' + positionerName + axislabel].text()),
                            i
                        )
                    )
                    self._widget.pars['DownButton' + positionerName + axislabels[i]].pressed.connect(
                        lambda positionerName=positionerName, axislabel=axislabels[i], i=i: self.move(
                            positionerName,
                            -float(self._widget.pars['StepEdit' + positionerName + axislabel].text()),
                            i
                        )
                    )

    def move(self, positioner, dist, axis):
        """ Moves the piezzos in x y or z (axis) by dist micrometers. """
        newPos = self._master.positionersManager[positioner].move(dist, axis)
        newText = "<strong>{0:.2f} µm</strong>".format(newPos)
        axislabels = textwrap.wrap(self._setupInfo.positioners[positioner].managerProperties['axisLabels'],1)
        self._widget.pars['Position' + positioner + axislabels[axis]].setText(newText)

    def getPos(self):
        return self._master.positionersManager.execOnAll(lambda p: p.position)

    def closeEvent(self):
        self._master.positionersManager.execOnAll(lambda p: p.setPosition(0))


class LaserController(WidgetController):
    """ Linked to LaserWidget."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._widget.initControls(self._master.lasersManager)

        self.aotfLasers = {}
        for laserName, laserManager in self._master.lasersManager:
            if not laserManager.isDigital:
                self.aotfLasers[laserName] = False

        # Connect LaserWidget signals
        for laserModule in self._widget.laserModules.values():
            if not self._master.lasersManager[laserModule.laser].isBinary:
                if self._master.lasersManager[laserModule.laser].isDigital:
                    self.changeEdit(laserModule.laser)

                laserModule.slider.valueChanged[int].connect(lambda _, laser=laserModule.laser: self.changeSlider(laser))
                laserModule.setPointEdit.returnPressed.connect(lambda laser=laserModule.laser: self.changeEdit(laser))

            laserModule.enableButton.toggled.connect(lambda _, laser=laserModule.laser: self.toggleLaser(laser))

        for digModuleLaser in self._widget.digModule.powers.keys():
            self._widget.digModule.powers[digModuleLaser].textChanged.connect(
                lambda _, laser=digModuleLaser: self.updateDigitalPowers([laser])
            )

        self._widget.digModule.DigitalControlButton.clicked.connect(
            lambda: self.GlobalDigitalMod(list(self._widget.digModule.powers.keys()))
        )
        self._widget.digModule.updateDigPowersButton.clicked.connect(
            lambda: self.updateDigitalPowers(list(self._widget.digModule.powers.keys()))
        )

    def closeEvent(self):
        self._master.lasersManager.execOnAll(lambda l: l.setDigitalMod(False, 0))
        self._master.lasersManager.execOnAll(lambda l: l.setValue(0))

    def toggleLaser(self, laserName):
        """ Enable or disable laser (on/off)."""
        self._master.lasersManager[laserName].setEnabled(
            self._widget.laserModules[laserName].enableButton.isChecked()
        )

    def changeSlider(self, laserName):
        """ Change power with slider magnitude. """
        magnitude = self._widget.laserModules[laserName].slider.value()
        if laserName not in self.aotfLasers.keys() or not self.aotfLasers[laserName]:
            self._master.lasersManager[laserName].setValue(magnitude)
            self._widget.laserModules[laserName].setPointEdit.setText(str(magnitude))

    def changeEdit(self, laserName):
        """ Change power with edit magnitude. """
        magnitude = float(self._widget.laserModules[laserName].setPointEdit.text())
        if laserName not in self.aotfLasers.keys() or not self.aotfLasers[laserName]:
            self._master.lasersManager[laserName].setValue(magnitude)
            self._widget.laserModules[laserName].slider.setValue(magnitude)

    def updateDigitalPowers(self, laserNames):
        """ Update the powers if the digital mod is on. """
        if self._widget.digModule.DigitalControlButton.isChecked():
            for laserName in laserNames:
                self._master.lasersManager[laserName].setValue(
                    float(self._widget.digModule.powers[laserName].text())
                )

    def GlobalDigitalMod(self, laserNames):
        """ Start digital modulation. """
        digMod = self._widget.digModule.DigitalControlButton.isChecked()
        for laserName in laserNames:
            laserModule = self._widget.laserModules[laserName]
            laserManager = self._master.lasersManager[laserName]

            if laserManager.isBinary:
                continue

            value = float(self._widget.digModule.powers[laserName].text())
            if laserManager.isDigital:
                laserManager.setDigitalMod(digMod, value)
            else:
                laserManager.setValue(value)
                laserModule.enableButton.setChecked(False)
                laserModule.enableButton.setEnabled(not digMod)
                self.aotfLasers[laserName] = digMod
                laserManager.setEnabled(False)  # TODO: Correct?

            laserModule.setPointEdit.setEnabled(not digMod)
            laserModule.slider.setEnabled(not digMod)

            if not digMod:
                self.changeEdit(laserName)

    def setDigitalButton(self, b):
        self._widget.digModule.DigitalControlButton.setChecked(b)
        self.GlobalDigitalMod(
            [laser.name for laser in self._master.lasersManager if laser.isDigital]
        )


class BeadController(WidgetController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.running = False
        self.addROI()

        # Connect BeadRecWidget signals
        self._widget.roiButton.clicked.connect(self.toggleROI)
        self._widget.runButton.clicked.connect(self.run)

    def toggleROI(self):
        """ Show or hide ROI."""
        if self._widget.roiButton.isChecked() is False:
            self._widget.ROI.hide()
            self.active = False
            self._widget.roiButton.setText('Show ROI')
        else:
            ROIsize = (64, 64)
            ROIcenter = self._commChannel.getCenterROI()

            ROIpos = (ROIcenter[0] - 0.5 * ROIsize[0],
                      ROIcenter[1] - 0.5 * ROIsize[1])

            self._widget.ROI.setPos(ROIpos)
            self._widget.ROI.setSize(ROIsize)
            self._widget.ROI.show()
            self.active = True
            self._widget.roiButton.setText('Hide ROI')

    def addROI(self):
        """ Adds the ROI to ImageWidget viewbox through the CommunicationChannel. """
        self._commChannel.addItemTovb.emit(self._widget.ROI)

    def run(self):
        if not self.running:
            self.dims = np.array(self._commChannel.getDimsScan()).astype(int)
            self.running = True
            self.beadWorker = BeadWorker(self)
            self.beadWorker.newChunk.connect(self.update)
            self.thread = Thread()
            self.beadWorker.moveToThread(self.thread)
            self.thread.started.connect(self.beadWorker.run)
            self._master.detectorsManager.execOnAll(lambda c: c.flushBuffers())
            self.thread.start()
        else:
            self.running = False
            self.thread.quit()
            self.thread.wait()

    def update(self):
        self._widget.img.setImage(np.resize(self.recIm, self.dims + 1), autoLevels=False)


class BeadWorker(Worker):
    newChunk = Signal()
    stop = Signal()

    def __init__(self, controller):
        super().__init__()
        self.__controller = controller

    def run(self):
        dims = np.array(self.__controller.dims)
        N = (dims[0] + 1) * (dims[1] + 1)
        self.__controller.recIm = np.zeros(N)
        i = 0

        while self.__controller.running:
            newImages, _ = self.__controller._master.detectorsManager.execOnCurrent(lambda c: c.getChunk())
            n = len(newImages)
            if n > 0:
                pos = self.__controller._widget.ROI.pos()
                size = self.__controller._widget.ROI.size()

                x0 = int(pos[0])
                y0 = int(pos[1])
                x1 = int(x0 + size[0])
                y1 = int(y0 + size[1])

                for j in range(0, n):
                    img = newImages[j]
                    img = img[x0:x1, y0:y1]
                    mean = np.mean(img)
                    self.__controller.recIm[i] = mean
                    i = i + 1
                    if i == N:
                        i = 0
                self.newChunk.emit()


class MotCorrController(WidgetController):
    """ Linked to MotCorrWidget."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self._manager = self._master.rs232sManager._subManagers['leicadmi']
        #TODO: BELOW IS JUST COPIED FROM LASERCONTROLLER
        self.aotfLasers = {}
        for laserName, laserManager in self._master.lasersManager:
            if not laserManager.isDigital:
                self.aotfLasers[laserName] = False

        # Connect LaserWidget signals
        for laserModule in self._widget.laserModules.values():
            if not self._master.lasersManager[laserModule.laser].isBinary:
                if self._master.lasersManager[laserModule.laser].isDigital:
                    self.changeEdit(laserModule.laser)

                laserModule.slider.valueChanged[int].connect(lambda _, laser=laserModule.laser: self.changeSlider(laser))
                laserModule.setPointEdit.returnPressed.connect(lambda laser=laserModule.laser: self.changeEdit(laser))

            laserModule.enableButton.toggled.connect(lambda _, laser=laserModule.laser: self.toggleLaser(laser))

        for digModuleLaser in self._widget.digModule.powers.keys():
            self._widget.digModule.powers[digModuleLaser].textChanged.connect(
                lambda _, laser=digModuleLaser: self.updateDigitalPowers([laser])
            )

        self._widget.digModule.DigitalControlButton.clicked.connect(
            lambda: self.GlobalDigitalMod(list(self._widget.digModule.powers.keys()))
        )
        self._widget.digModule.updateDigPowersButton.clicked.connect(
            lambda: self.updateDigitalPowers(list(self._widget.digModule.powers.keys()))
        )

    def closeEvent(self):
        self._master.lasersManager.execOnAll(lambda l: l.setDigitalMod(False, 0))
        self._master.lasersManager.execOnAll(lambda l: l.setValue(0))

    def toggleLaser(self, laserName):
        """ Enable or disable laser (on/off)."""
        self._master.lasersManager[laserName].setEnabled(
            self._widget.laserModules[laserName].enableButton.isChecked()
        )

    def changeSlider(self, laserName):
        """ Change power with slider magnitude. """
        magnitude = self._widget.laserModules[laserName].slider.value()
        if laserName not in self.aotfLasers.keys() or not self.aotfLasers[laserName]:
            self._master.lasersManager[laserName].setValue(magnitude)
            self._widget.laserModules[laserName].setPointEdit.setText(str(magnitude))

    def changeEdit(self, laserName):
        """ Change power with edit magnitude. """
        magnitude = float(self._widget.laserModules[laserName].setPointEdit.text())
        if laserName not in self.aotfLasers.keys() or not self.aotfLasers[laserName]:
            self._master.lasersManager[laserName].setValue(magnitude)
            self._widget.laserModules[laserName].slider.setValue(magnitude)

    def updateDigitalPowers(self, laserNames):
        """ Update the powers if the digital mod is on. """
        if self._widget.digModule.DigitalControlButton.isChecked():
            for laserName in laserNames:
                self._master.lasersManager[laserName].setValue(
                    float(self._widget.digModule.powers[laserName].text())
                )

    def GlobalDigitalMod(self, laserNames):
        """ Start digital modulation. """
        digMod = self._widget.digModule.DigitalControlButton.isChecked()
        for laserName in laserNames:
            laserModule = self._widget.laserModules[laserName]
            laserManager = self._master.lasersManager[laserName]

            if laserManager.isBinary:
                continue

            value = float(self._widget.digModule.powers[laserName].text())
            if laserManager.isDigital:
                laserManager.setDigitalMod(digMod, value)
            else:
                laserManager.setValue(value)
                laserModule.enableButton.setChecked(False)
                laserModule.enableButton.setEnabled(not digMod)
                self.aotfLasers[laserName] = digMod
                laserManager.setEnabled(False)  # TODO: Correct?

            laserModule.setPointEdit.setEnabled(not digMod)
            laserModule.slider.setEnabled(not digMod)

            if not digMod:
                self.changeEdit(laserName)

    def setDigitalButton(self, b):
        self._widget.digModule.DigitalControlButton.setChecked(b)
        self.GlobalDigitalMod(
            [laser.name for laser in self._master.lasersManager if laser.isDigital]
        )

