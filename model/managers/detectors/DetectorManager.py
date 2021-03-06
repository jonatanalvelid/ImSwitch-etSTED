from abc import abstractmethod
from dataclasses import dataclass
from typing import List, Any

import numpy as np
from framework import Signal, SignalInterface
import view.guitools as guitools


@dataclass
class DetectorParameter:
    group: str
    value: Any
    editable: bool


@dataclass
class DetectorNumberParameter(DetectorParameter):
    value: float
    valueUnits: str


@dataclass
class DetectorListParameter(DetectorParameter):
    value: str
    options: List[str]


class DetectorManager(SignalInterface):
    imageUpdated = Signal(np.ndarray, bool)

    @abstractmethod
    def __init__(self, name, fullShape, supportedBinnings, model, parameters):
        super().__init__()

        self.__name = name
        self.__model = model
        self._parameters = parameters

        self.__frameStart = (0, 0)
        self.__shape = fullShape
        self.__fullShape = fullShape
        self.__supportedBinnings = supportedBinnings
        self.__image = np.array([])

        self.setBinning(supportedBinnings[0])

    def updateLatestFrame(self, init):
        self.__image = self.getLatestFrame()
        self.imageUpdated.emit(self.__image, init)

    def setParameter(self, name, value):
        """Sets a parameter value and returns the updated list of parameters.
        If the parameter doesn't exist, i.e. the parameters field doesn't
        contain a key with the specified parameter name, an error will be
        raised."""

        if name not in self._parameters:
            raise AttributeError(f'Non-existent parameter "{name}" specified')

        self._parameters[name].value = value
        return self.parameters

    @property
    def name(self):
        return self.__name

    @property
    def model(self):
        return self.__model

    @property
    def binning(self):
        return self._binning

    @property
    def supportedBinnings(self):
        return self.__supportedBinnings

    @property
    def frameStart(self):
        return self.__frameStart

    @property
    def shape(self):
        return self.__shape

    @shape.setter
    def shape(self, crop_shape):
        self.__shape = crop_shape

    @property
    def fullShape(self):
        return self.__fullShape

    @property
    def image(self):
        return self.__image

    @property
    def parameters(self):
        return self._parameters

    @property
    @abstractmethod
    def pixelSize(self):
        return [1,1,1]

    @abstractmethod
    def setBinning(self, binning):
        if binning not in self.__supportedBinnings:
            raise ValueError(f'Specified binning value "{binning}" not supported by the detector')

        self._binning = binning

    @abstractmethod
    def crop(self, hpos, vpos, hsize, vsize):
        """Method to crop the frame read out by the detector."""
        pass

    @abstractmethod
    def getLatestFrame(self):
        """Returns the frame that represents what the detector currently is
        capturing."""
        pass

    @abstractmethod
    def getChunk(self):
        """Returns the frames captured by the detector since getChunk was last
        called, or since the buffers were last flushed (whichever happened
        last)."""
        pass

    @abstractmethod
    def flushBuffers(self):
        """Flushes the detector buffers so that getChunk starts at the last
        frame captured at the time that this function was called."""
        pass

    @abstractmethod
    def startAcquisition(self):
        pass

    @abstractmethod
    def stopAcquisition(self):
        pass
