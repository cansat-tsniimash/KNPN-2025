# This Python file uses the following encoding: utf-8
import sys
import os



from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import QFile
from PySide2 import QtUiTools, QtGui

#from PyQt5 import QtWidgets, uic
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

import numpy as NumPy

class UiLoader(QtUiTools.QUiLoader):
    def createWidget(self, className, parent=None, name=""):
        if className == "PlotWidget":
            return pg.PlotWidget(parent=parent)
        return super().createWidget(className, parent, name)

class gcs(QMainWindow):
    def __init__(self):
        super(gcs, self).__init__()
        self.load_ui()

        self.ui.show()

        grapLayout = pg.GraphicsLayoutWidget()
        #self.ui.gridLayout_2.addWidget(grapLayout, 0, 0, 1, 1)
        #def setup_axis(self, name_x, name_y):
        axis_x = pg.AxisItem(orientation='bottom')
        axis_x.setLabel("Time")
        axis_y = pg.AxisItem(orientation='left')
        axis_y.setLabel("Data")
        self.ui.plot1.setAxisItems({'bottom': axis_x, 'left': axis_y})
        #def setup_plot_legend(self):
        self.legend1 = self.ui.plot1.addLegend()
        self.plot1_list = [self.ui.plot1.plot(NumPy.array([[0, 0], [1, 1], [2, 0]]), name="x")]

        self.pen = QtGui.QPen()
        self.pen.setColor(QtGui.QColor(255, 0, 0))

        self.pen.setWidth(0.1)

        self.plot1_list[0].setPen(self.pen)




    def load_ui(self):
        loader = UiLoader()
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()



if __name__ == "__main__":
    app = QApplication([])
    widget = gcs()
    #widget.show()
    sys.exit(app.exec_())
