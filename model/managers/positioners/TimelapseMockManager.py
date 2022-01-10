from .PositionerManager import PositionerManager


class TimelapseMockManager(PositionerManager):
    def __init__(self, positionerInfo, name, **kwargs):
        super().__init__(name, initialPosition=0)

    def move(self, dist, *args):
        return self.setPosition(self._position + dist)

    def setPosition(self, position, *args):
        self._position = position
        return position
