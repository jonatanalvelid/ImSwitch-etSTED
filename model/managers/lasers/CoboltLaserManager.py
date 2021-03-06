# -*- coding: utf-8 -*-
"""
Created on Tue Mar 24 16:41:57 2020

@author: _Xavi
"""
from lantz import Q_

from model import FullDigitalLaser
from .LaserManager import LaserManager


class CoboltLaserManager(LaserManager):
    def __init__(self, laserInfo, name, **kwargs):
        # Init laser
        self._laser = FullDigitalLaser(laserInfo.managerProperties['digitalDriver'],
                                       laserInfo.managerProperties['digitalPorts'])
        self._nidaqManager = kwargs['nidaqManager']
        print(self._laser.idn)
        self._laser.digital_mod = False
        self._laser.enabled = False
        self._laser.autostart = False

        super().__init__(
            name, isBinary=False, isDigital=True, wavelength=laserInfo.wavelength,
            valueRangeMin=laserInfo.valueRangeMin, valueRangeMax=laserInfo.valueRangeMax,
            valueUnits='mW', valueRangeStep=laserInfo.valueRangeStep
        )
        print(self._laser.digital_mod)

    def setEnabled(self, enabled):
        if self._laser.digital_mod:
            self._nidaqManager.setDigital(self.name, enabled)
        else:
            self._laser.enabled = enabled

    def setValue(self, power):
        power = int(power)
        if self._laser.digital_mod:
            self._laser.power_mod = power * Q_(1, 'mW')
        else:
            self._laser.power_sp = power * Q_(1, 'mW')

    def setDigitalMod(self, digital, initialPower):
        if digital:
            self._laser.enter_mod_mode()
            self._laser.digital_mod = True
            self._laser.power_mod = initialPower * Q_(1, 'mW')
            print('Entered digital modulation mode with power :', initialPower)
            print('Modulation mode is: ', self._laser.mod_mode)
        else:
            self._laser.digital_mod = False
            self._laser.query('cp')
            print('Exited digital modulation mode')
