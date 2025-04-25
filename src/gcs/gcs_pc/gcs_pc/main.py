# This Python file uses the following encoding: utf-8
import sys
import os

from socket import *

import struct

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2 import QtUiTools, QtGui, QtCore

#from PyQt5 import QtWidgets, uic, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

import numpy as NumPy

import time

class UiLoader(QtUiTools.QUiLoader):
    def createWidget(self, className, parent=None, name=""):
        if className == "PlotWidget":
            return pg.PlotWidget(parent=parent)
        return super().createWidget(className, parent, name)

class DataManager(QtCore.QObject):
    new_data = QtCore.Signal(list)
    #@Slot(int)
    def add_new_data (data):
        if len(data) == 50:
            unpack_data = struct.unpack("<BHI10hIh3fBhH", data)
            self.new_data.emit(unpack_data)
            add_data(self.ui.Accelerometer, unpack_data[2], unpack_data[3]*488/1000/1000)
            add_data(self.ui.Accelerometer, unpack_data[2], unpack_data[4]*488/1000/1000)
            add_data(self.ui.Accelerometer, unpack_data[2], unpack_data[5]*488/1000/1000)
            add_data(self.ui.Giroscope, unpack_data[2], unpack_data[6]*70/1000)
            add_data(self.ui.Giroscope, unpack_data[2], unpack_data[7]*70/1000)
            add_data(self.ui.Giroscope, unpack_data[2], unpack_data[8]*70/1000)
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[9]/1711)
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[10]/1711)
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[11]/1711)
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[12])
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[12])
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[12])
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[12])
            add_data(self.ui.Magnetometer, unpack_data[2], unpack_data[12])
            print ("Time_ms", unpack_data[2])
            print ("Accelerometer x", unpack_data[3]*488/1000/1000)
            print ("Accelerometer y", unpack_data[4]*488/1000/1000)
            print ("Accelerometer z", unpack_data[5]*488/1000/1000)
            print ("Gyroscope x", unpack_data[6]*70/1000)
            print ("Gyroscope y", unpack_data[7]*70/1000)
            print ("Gyroscope z", unpack_data[8]*70/1000)
            print ("Magnetometer x", unpack_data[9]/1711)
            print ("Magnetometer y", unpack_data[10]/1711)
            print ("Magnetometer z", unpack_data[11]/1711)
            print ("Bme_temp", unpack_data[12])
            print ("Bme_press", unpack_data[13])
            print ("Bme_humidity", unpack_data[14])
            print ("Bme_height", unpack_data[15])
            print ("Lux_board", unpack_data[16])
            print ("Lux_sp", unpack_data[17])
            print ("State", unpack_data[18])
            print ("Lidar", unpack_data[19])
            print ("crc", unpack_data[20])
            print ("\n")
        elif len(data) == 53:
            unpack_data = struct.unpack("<BHIh3fIf7HhIhH", data)
            print ("Number", unpack_data[1])
            print ("Time_ms", unpack_data[2])
            print ("Fix", unpack_data[3])
            print ("Lat", unpack_data[4])
            print ("Lon", unpack_data[5])
            print ("Alt", unpack_data[6])
            print ("GPS_time_s", unpack_data[7])
            print ("Current", unpack_data[8])
            print ("Bus_voltage", unpack_data[9])
            print ("MICS_5524", unpack_data[10])
            print ("MICS_CO", unpack_data[11])
            print ("MICS_NO2", unpack_data[12])
            print ("MICS_NH3", unpack_data[13])
            print ("CCS_CO2", unpack_data[14])
            print ("CCS_TVOC", unpack_data[15])
            print ("Bme_temp_g", unpack_data[16])
            print ("Bme_press_g", unpack_data[17])
            print ("Bme_humidity_g", unpack_data[18])
            print ("crc", unpack_data[19])
            print ('\n')
    def start(self):
        host = '192.168.128.217'
        port = 22000
        addr = (host,port)

        udp_socket = socket(AF_INET, SOCK_DGRAM)
        udp_socket.setblocking(False)
        udp_socket.settimeout(0.1)
        udp_socket.sendto(b"h", addr)
        data = []

        while True:
            try:
                data, addr = udp_socket.recvfrom(53)
            except Exception as e:
                pass

            try:
                print(data)
            except Exception as e:
                pass

            try:
                self.pars_packet(data)
            except Exception as e:
                print(e)
                pass

    def pars_packet(self, data):
        if len(data) == 50:
            unpack_data = struct.unpack("<BHI10hIh3fBhH", data)
            self.new_data.emit(unpack_data)
            print ("Time_ms", unpack_data[2])
            print ("Accelerometer x", unpack_data[3]*488/1000/1000)
            print ("Accelerometer y", unpack_data[4]*488/1000/1000)
            print ("Accelerometer z", unpack_data[5]*488/1000/1000)
            print ("Gyroscope x", unpack_data[6]*70/1000)
            print ("Gyroscope y", unpack_data[7]*70/1000)
            print ("Gyroscope z", unpack_data[8]*70/1000)
            print ("Magnetometer x", unpack_data[9]/1711)
            print ("Magnetometer y", unpack_data[10]/1711)
            print ("Magnetometer z", unpack_data[11]/1711)
            print ("Bme_temp", unpack_data[12])
            print ("Bme_press", unpack_data[13])
            print ("Bme_humidity", unpack_data[14])
            print ("Bme_height", unpack_data[15])
            print ("Lux_board", unpack_data[16])
            print ("Lux_sp", unpack_data[17])
            print ("State", unpack_data[18])
            print ("Lidar", unpack_data[19])
            print ("crc", unpack_data[20])
            print ("\n")
        elif len(data) == 53:
            unpack_data = struct.unpack("<BHIh3fIf7HhIhH", data)
            print ("Number", unpack_data[1])
            print ("Time_ms", unpack_data[2])
            print ("Fix", unpack_data[3])
            print ("Lat", unpack_data[4])
            print ("Lon", unpack_data[5])
            print ("Alt", unpack_data[6])
            print ("GPS_time_s", unpack_data[7])
            print ("Current", unpack_data[8])
            print ("Bus_voltage", unpack_data[9])
            print ("MICS_5524", unpack_data[10])
            print ("MICS_CO", unpack_data[11])
            print ("MICS_NO2", unpack_data[12])
            print ("MICS_NH3", unpack_data[13])
            print ("CCS_CO2", unpack_data[14])
            print ("CCS_TVOC", unpack_data[15])
            print ("Bme_temp_g", unpack_data[16])
            print ("Bme_press_g", unpack_data[17])
            print ("Bme_humidity_g", unpack_data[18])
            print ("crc", unpack_data[19])
            print ('\n')
        else:
            print("You are IDIOT!!!!!!!!!!!")


class gcs(QMainWindow):
    def __init__(self):
        super(gcs, self).__init__()
        self.load_ui()

        self.ui.show()

        self.setup_ui()

        #axis_x = pg.AxisItem(orientation='bottom')
        #axis_x.setLabel("Time")
        #axis_y = pg.AxisItem(orientation='left')
        #axis_y.setLabel("Data, ")
        #self.ui.Accelerometer.setAxisItems({'bottom': axis_x, 'left': axis_y})

        #self.legend1 = self.ui.plot1.addLegend()
        #self.plot1_list = [self.ui.plot1.plot(NumPy.array([[0, 0], [1, 1], [2, 0]]), name="x")]

        #self.pen = QtGui.QPen()
        #self.pen.setColor(QtGui.QColor(255, 0, 0))

        #self.pen.setWidth(0.1)

        #self.plot1_list[0].setPen(self.pen)

        self.data_manager = DataManager()

        self.data_thread = QtCore.QThread(self)
        self.data_manager.moveToThread(self.data_thread)
        self.data_thread.started.connect(self.data_manager.start)
        self.data_thread.start()



    def load_ui(self):
        loader = UiLoader()
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        ui_file = QtCore.QFile(path)
        ui_file.open(QtCore.QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

    def set_axis(self, plot, name):
        axis_x = pg.AxisItem(orientation='bottom')
        axis_x.setLabel("Time")
        axis_y = pg.AxisItem(orientation='left')
        axis_y.setLabel(name)
        plot.setAxisItems({'bottom': axis_x, 'left': axis_y})

    def set_line(self, plot, coordinate_x, coordinate_y, color, name):

        line = plot.plot(x=coordinate_x, y=coordinate_y, name=name)

        pen = QtGui.QPen()
        pen.setColor(QtGui.QColor(color))
        pen.setWidth(0.1)
        line.setPen(pen)
        return line

    def add_data(self, plot, time, data):
        oldx, oldy = plot.getData()
        newx = NumPy.hstack((oldx, time))
        newy = NumPy.hstack((oldy, data))
        newdot = plot.setData(x=newx, y=newy)



    def setup_ui(self):
        self.set_axis(self.ui.Accelerometer, "Acceleration, g")
        self.set_axis(self.ui.Giroscope, "Angular rate, m/s")
        self.set_axis(self.ui.Magnetometer, "Magnetic induction, gauss")
        self.set_axis(self.ui.Pressure, "Pressure, J")
        self.set_axis(self.ui.Height, "Height, m")
        self.set_axis(self.ui.Temperature, "Temperature, C")
        self.set_axis(self.ui.Gases_consentration, "Gases consentration, ppm")
        self.set_axis(self.ui.SP_parameters, "SP parameters")
        self.set_axis(self.ui.Illumination, "Illumination, lx")

        self.legend1 = self.ui.Accelerometer.addLegend()
        self.legend2 = self.ui.Giroscope.addLegend()
        self.legend3 = self.ui.Magnetometer.addLegend()
        self.legend4 = self.ui.Pressure.addLegend()
        self.legend5 = self.ui.Height.addLegend()
        self.legend6 = self.ui.Temperature.addLegend()
        self.legend7 = self.ui.SP_parameters.addLegend()
        self.legend8 = self.ui.Gases_consentration.addLegend()
        self.legend9 = self.ui.Illumination.addLegend()

        self.Accelerometer =       [self.set_line(self.ui.Accelerometer,       [0,1], [1,0],    "#0000FF", "x"),
                                    self.set_line(self.ui.Accelerometer,       [0,1], [0.5,0],  "#00FF00", "y"),
                                    self.set_line(self.ui.Accelerometer,       [0,1], [0.25,0], "#FF0000", "z")]
        self.Giroscope =           [self.set_line(self.ui.Giroscope,           [0,1], [1,0],    "#0000FF", "x"),
                                    self.set_line(self.ui.Giroscope,           [0,1], [0.5,0],  "#00FF00", "y"),
                                    self.set_line(self.ui.Giroscope,           [0,1], [0.25,0], "#FF0000", "z")]
        self.Magnetometer =        [self.set_line(self.ui.Magnetometer,        [0,1], [1,0],    "#0000FF", "x"),
                                    self.set_line(self.ui.Magnetometer,        [0,1], [0.5,0],  "#00FF00", "y"),
                                    self.set_line(self.ui.Magnetometer,        [0,1], [0.25,0], "#FF0000", "z")]
        self.Pressure =            [self.set_line(self.ui.Pressure,            [0,1], [1,0],    "#0000FF", "Outside"),
                                    self.set_line(self.ui.Pressure,            [0,1], [0.5,0],  "#00FF00", "Germo")]
        self.Height =              [self.set_line(self.ui.Height,              [0,1], [1,0],    "#0000FF", "BME-280 out"),
                                    self.set_line(self.ui.Height,              [0,1], [0.5,0],  "#00FF00", "GPS")]
        self.Temperature =         [self.set_line(self.ui.Temperature,         [0,1], [1,0],    "#0000FF", "Outside"),
                                    self.set_line(self.ui.Temperature,         [0,1], [0.5,0],  "#00FF00", "Germo")]
        self.Gases_consentration = [self.set_line(self.ui.Gases_consentration, [0,1], [1.5,0],  "#0000FF", "MICS-6814 CO"),
                                    self.set_line(self.ui.Gases_consentration, [0,1], [1.25,0], "#00FF00", "MICS-6814 NO2"),
                                    self.set_line(self.ui.Gases_consentration, [0,1], [1,0],    "#FF0000", "MICS-6814 NH3"),
                                    self.set_line(self.ui.Gases_consentration, [0,1], [0.5,0],  "#F0F000", "MICS-5524"),
                                    self.set_line(self.ui.Gases_consentration, [0,1], [0.25,0], "#1FF0D5", "CCS811 CO2"),
                                    self.set_line(self.ui.Gases_consentration, [0,1], [0.15,0], "#FF00F0", "CCS811 TVOC")]
        self.SP_parameters =       [self.set_line(self.ui.SP_parameters,       [0,1], [1,0],    "#0000FF", "Current, A"),
                                    self.set_line(self.ui.SP_parameters,       [0,1], [0.5,0],  "#00FF00", "Voultage, V")]
        self.Illumination =        [self.set_line(self.ui.Illumination,        [0,1], [1,0],    "#0000FF", "SP"),
                                    self.set_line(self.ui.Illumination,        [0,1], [0.5,0],  "#00FF00", "Board")]
        self.add_data(self.Gases_consentration[0], NumPy.array([5]), NumPy.array([5]))



if __name__ == "__main__":
    app = QApplication([])
    widget = gcs()
    #widget.show()
    sys.exit(app.exec_())
