import os
import sys
import logging

from pyqtgraph.Qt import QtGui
import qdarkstyle

import configfileutils
from controller.MainController import MainController
from view.MainView import MainView
from view.guitools import ViewSetupInfo

logging.getLogger("pyvisa").setLevel(logging.WARNING)
logging.getLogger("lantz").setLevel(logging.WARNING)
#logging.getLogger("matplotlib").setLevel(logging.WARNING)

setupInfo = configfileutils.loadSetupInfo(ViewSetupInfo)
app = QtGui.QApplication([])
app.setStyleSheet(qdarkstyle.load_stylesheet(qt_api=os.environ.get('PYQTGRAPH_QT_LIB')))
view = MainView(setupInfo)
controller = MainController(setupInfo, view)
view.show()
sys.exit(app.exec_())
