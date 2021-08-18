# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 09:58:00 2021

@author: jonatanalvelid
"""

from .PositionerManager import PositionerManager


class MHXYStageManager(PositionerManager):
    def __init__(self, positionerInfo, name, *args, **kwargs):
        super().__init__(name=name, initialPosition=[0,0])
        self._rs232Manager = kwargs['rs232sManager']._subManagers[positionerInfo.managerProperties['rs232device']]
        print(str(self._rs232Manager.send('?readsn')))  # print serial no of stage

    def move(self, value, axis):
        if axis == 0:
            cmd = 'mor x ' + str(float(value))
        elif axis == 1:
            cmd = 'mor y ' + str(float(value))
        else:
            print('Wrong axis, has to be 0 or 1.')
            return
        self._rs232Manager.send(cmd)
        self._position[axis] = self._position[axis] + value
        return self._position[axis]

    def setPosition(self, value, axis=0):
        if axis == 0:
            cmd = 'moa x ' + str(float(value))
        elif axis == 1:
            cmd = 'moa y ' + str(float(value))
        else:
            print('Wrong axis, has to be 0 or 1.')
            return
        self._rs232Manager.send(cmd)
        self._position[axis] = value
        return self._position[axis]

    def position_old(self, axis):
        if axis == 0 or axis == 1:
            return self._position[axis]
        else:
            print('Wrong axis, has to be 0 or 1.')

    def position(self, axis):
        if axis == 0:
            cmd = 'pos x'
        elif axis == 1:
            cmd = 'pos y'
        else:
            print('Wrong axis, has to be 0 or 1.')
            return
        pos = float(self._rs232Manager.send(cmd))
        self._position[axis] = pos
        return self._position[axis]

