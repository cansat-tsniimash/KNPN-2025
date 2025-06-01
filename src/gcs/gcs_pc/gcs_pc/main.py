# This Python file uses the following encoding: utf-8
import sys
import os

from socket import *

import struct

from PySide2.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PySide2 import QtUiTools, QtGui, QtCore
from PySide2.QtGui import QFont
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
    new_pack_1 = QtCore.Signal(list)
    new_pack_2 = QtCore.Signal(list)
    mutex = QtCore.QMutex()

    def reconnect(self):
        self.udp_socket.sendto(b"h", self.addr)

    def start(self):
        host = '192.168.0.203'
        port = 22000
        self.addr = (host,port)

        self.udp_socket = socket(AF_INET, SOCK_DGRAM)
        self.udp_socket.setblocking(False)
        self.udp_socket.settimeout(0.1)
        self.udp_socket.bind(("0.0.0.0", 22000))
        data = []
        print("connected")

        self.mutex.lock()
        self.close = True
        close = self.close
        self.mutex.unlock()


        while close:
            try:
                data, addr = self.udp_socket.recvfrom(53)
            except Exception as e:
                print(e)
                pass

            try:
                print(data)
            except Exception as e:
                print(e)
                pass

            try:
                self.pars_packet(data)
            except Exception as e:
                print(e)
                pass
            self.mutex.lock()
            close = self.close
            self.mutex.unlock()

    def pars_packet(self, data):
        if len(data) == 50:
            unpack_data = struct.unpack("<BHI10hIh3fBhH", data)
            self.new_pack_1.emit(unpack_data)
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
            self.new_pack_2.emit(unpack_data)
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

    def stop(self):
        self.mutex.lock()
        self.close = False
        self.mutex.unlock()
        self.udp_socket.close()


class gcs(QMainWindow):
    def __init__(self):
        super(gcs, self).__init__()
        self.show_data = False
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

        self.ui.Connect.clicked.connect(self.connect_action)
        self.ui.Stop.clicked.connect(self.stop_action)
        self.ui.Start.clicked.connect(self.start_action)
        self.ui.Reset.clicked.connect(self.reset_action)

        self.data_manager.new_pack_1.connect(self.add_new_pack_1)
        self.data_manager.new_pack_2.connect(self.add_new_pack_2)


    def load_ui(self):
        loader = UiLoader()
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        ui_file = QtCore.QFile(path)
        ui_file.open(QtCore.QFile.ReadOnly)
        self.ui = loader.load(ui_file, self.centralWidget())
        ui_file.close()

    def set_axis(self, plot, name):
        axis_x = pg.AxisItem(orientation='bottom')
        axis_x.setLabel("Time")
        my_font = QFont("Times", 14, QFont.Bold)
        axis_x.label.setFont(my_font)
        axis_x.setStyle(tickFont=my_font)
        axis_y = pg.AxisItem(orientation='left')
        axis_y.setLabel(name)
        axis_y.label.setFont(my_font)
        axis_y.setStyle(tickFont=my_font)
        plot.setAxisItems({'bottom': axis_x, 'left': axis_y})

    def set_line(self, plot, color, name):

        line = plot.plot(x=[0], y=[0], name=name)

        pen = QtGui.QPen()
        pen.setColor(QtGui.QColor(color))
        pen.setWidth(0.1)
        line.setPen(pen)
        return line

    def add_data(self, plot, time, data):
        oldx, oldy = plot.getData()
        if oldx[0] < 1:
            newx = NumPy.array(time)
            newy = NumPy.array(data)
        else:
            newx = NumPy.hstack((oldx, time))
            newy = NumPy.hstack((oldy, data))
        if NumPy.size(newx) > 100:
            newx = newx[-100:]
            newy = newy[-100:]

        current_len_x = NumPy.size(newx)
        current_len_y = NumPy.size(newy)
        if current_len_x != current_len_y:
            final_len = min(current_len_x, current_len_y)
            newx = newx[:final_len]
            newy = newy[:final_len]
        newdot = plot.setData(x=newx, y=newy)

    def setup_ui(self):
        self.set_axis(self.ui.Accelerometer, "Acceleration, g")
        self.set_axis(self.ui.Giroscope, "Angular rate, m/s")
        self.set_axis(self.ui.Magnetometer, "Magnetic induction, gauss")
        self.set_axis(self.ui.Pressure, "Pressure, Pa")
        self.set_axis(self.ui.Height, "Height, m")
        self.set_axis(self.ui.Temperature, "Temperature, C")
        self.set_axis(self.ui.Gases_consentration, "Gases consentration, mV")
        self.set_axis(self.ui.SP_parameters, "SP parameters")
        self.set_axis(self.ui.Illumination, "Illumination, c.u.")

        self.legend1 = self.ui.Accelerometer.addLegend()
        self.legend2 = self.ui.Giroscope.addLegend()
        self.legend3 = self.ui.Magnetometer.addLegend()
        self.legend4 = self.ui.Pressure.addLegend()
        self.legend5 = self.ui.Height.addLegend()
        self.legend6 = self.ui.Temperature.addLegend()
        self.legend7 = self.ui.SP_parameters.addLegend()
        self.legend8 = self.ui.Gases_consentration.addLegend()
        self.legend9 = self.ui.Illumination.addLegend()

        self.Accelerometer =       [self.set_line(self.ui.Accelerometer,       "#0000FF", "x"),
                                    self.set_line(self.ui.Accelerometer,       "#00FF00", "y"),
                                    self.set_line(self.ui.Accelerometer,       "#FF0000", "z")]
        self.Giroscope =           [self.set_line(self.ui.Giroscope,           "#0000FF", "x"),
                                    self.set_line(self.ui.Giroscope,           "#00FF00", "y"),
                                    self.set_line(self.ui.Giroscope,           "#FF0000", "z")]
        self.Magnetometer =        [self.set_line(self.ui.Magnetometer,        "#0000FF", "x"),
                                    self.set_line(self.ui.Magnetometer,        "#00FF00", "y"),
                                    self.set_line(self.ui.Magnetometer,        "#FF0000", "z")]
        self.Pressure =            [self.set_line(self.ui.Pressure,            "#0000FF", "Outside"),
                                    self.set_line(self.ui.Pressure,            "#00FF00", "Germo")]
        self.Height =              [self.set_line(self.ui.Height,              "#0000FF", "BME-280 out"),
                                    self.set_line(self.ui.Height,              "#00FF00", "GPS")]
        self.Temperature =         [self.set_line(self.ui.Temperature,         "#0000FF", "Outside"),
                                    self.set_line(self.ui.Temperature,         "#00FF00", "Germo")]
        self.Gases_consentration = [self.set_line(self.ui.Gases_consentration, "#0000FF", "MICS-5524"),
                                    self.set_line(self.ui.Gases_consentration, "#00FF00", "MICS-6814 CO"),
                                    self.set_line(self.ui.Gases_consentration, "#FF0000", "MICS-6814 NO2"),
                                    self.set_line(self.ui.Gases_consentration, "#F0F000", "MICS-6814 NH3"),
                                    self.set_line(self.ui.Gases_consentration, "#1FF0D5", "CCS811 CO2"),
                                    self.set_line(self.ui.Gases_consentration, "#FF00F0", "CCS811 TVOC")]
        self.SP_parameters =       [self.set_line(self.ui.SP_parameters,       "#0000FF", "Current, A"),
                                    self.set_line(self.ui.SP_parameters,       "#00FF00", "Voultage, V")]
        self.Illumination =        [self.set_line(self.ui.Illumination,        "#0000FF", "Board"),
                                    self.set_line(self.ui.Illumination,        "#00FF00", "SP")]
        #self.add_data(self.Gases_consentration[0], NumPy.array([5]), NumPy.array([5]))

    #@Slot(list)
    def add_new_pack_1(self, unpack_data):
        if self.show_data:
            self.add_data(self.Accelerometer[0], [unpack_data[2]], [unpack_data[3]*488/1000/1000])
            self.add_data(self.Accelerometer[1], [unpack_data[2]], [unpack_data[4]*488/1000/1000])
            self.add_data(self.Accelerometer[2], [unpack_data[2]], [unpack_data[5]*488/1000/1000])
            self.add_data(self.Giroscope[0], [unpack_data[2]], [unpack_data[6]*70/1000])
            self.add_data(self.Giroscope[1], [unpack_data[2]], [unpack_data[7]*70/1000])
            self.add_data(self.Giroscope[2], [unpack_data[2]], [unpack_data[8]*70/1000])
            self.add_data(self.Magnetometer[0], [unpack_data[2]], [unpack_data[9]/1711])
            self.add_data(self.Magnetometer[1], [unpack_data[2]], [unpack_data[10]/1711])
            self.add_data(self.Magnetometer[2], [unpack_data[2]], [unpack_data[11]/1711])
            self.add_data(self.Temperature[0], [unpack_data[2]], [unpack_data[12]])
            self.add_data(self.Pressure[0], [unpack_data[2]], [unpack_data[13]])
            self.add_data(self.Height[0], [unpack_data[2]], [unpack_data[15]])
            self.add_data(self.Illumination[0], [unpack_data[2]], [unpack_data[16]])
            self.add_data(self.Illumination[1], [unpack_data[2]], [unpack_data[17]])

            self.ui.Packet1.setItem(0, 1, QTableWidgetItem(str(unpack_data[1])))
            self.ui.Packet1.setItem(1, 1, QTableWidgetItem(str(unpack_data[2])))
            self.ui.Packet1.setItem(2, 1, QTableWidgetItem(str(unpack_data[3]*488/1000/1000)))
            self.ui.Packet1.setItem(3, 1, QTableWidgetItem(str(unpack_data[4]*488/1000/1000)))
            self.ui.Packet1.setItem(4, 1, QTableWidgetItem(str(unpack_data[5]*488/1000/1000)))
            self.ui.Packet1.setItem(5, 1, QTableWidgetItem(str(unpack_data[6]*70/1000)))
            self.ui.Packet1.setItem(6, 1, QTableWidgetItem(str(unpack_data[7]*70/1000)))
            self.ui.Packet1.setItem(7, 1, QTableWidgetItem(str(unpack_data[8]*70/1000)))
            self.ui.Packet1.setItem(8, 1, QTableWidgetItem(str(unpack_data[9]/1711)))
            self.ui.Packet1.setItem(9, 1, QTableWidgetItem(str(unpack_data[10]/1711)))
            self.ui.Packet1.setItem(10, 1, QTableWidgetItem(str(unpack_data[11]/1711)))
            self.ui.Packet1.setItem(11, 1, QTableWidgetItem(str(unpack_data[12])))
            self.ui.Packet1.setItem(12, 1, QTableWidgetItem(str(unpack_data[13])))
            self.ui.Packet1.setItem(13, 1, QTableWidgetItem(str(unpack_data[14])))
            self.ui.Packet1.setItem(14, 1, QTableWidgetItem(str(unpack_data[15])))
            self.ui.Packet1.setItem(15, 1, QTableWidgetItem(str(unpack_data[16])))
            self.ui.Packet1.setItem(16, 1, QTableWidgetItem(str(unpack_data[17])))
            self.ui.Packet1.setItem(17, 1, QTableWidgetItem(str(unpack_data[18])))
            self.ui.Packet1.setItem(18, 1, QTableWidgetItem(str(unpack_data[19])))



    #@Slot(list)
    def add_new_pack_2(self, unpack_data):
        if self.show_data:
            self.add_data(self.Height[1], [unpack_data[2]], [unpack_data[6]])
            self.add_data(self.SP_parameters[0], [unpack_data[2]], [unpack_data[8]])
            self.add_data(self.SP_parameters[1], [unpack_data[2]], [unpack_data[9]])
            self.add_data(self.Gases_consentration[0], [unpack_data[2]], [unpack_data[10]])
            self.add_data(self.Gases_consentration[1], [unpack_data[2]], [unpack_data[11]])
            self.add_data(self.Gases_consentration[2], [unpack_data[2]], [unpack_data[12]])
            self.add_data(self.Gases_consentration[3], [unpack_data[2]], [unpack_data[13]])
            self.add_data(self.Gases_consentration[4], [unpack_data[2]], [unpack_data[14]])
            self.add_data(self.Gases_consentration[5], [unpack_data[2]], [unpack_data[15]])
            self.add_data(self.Temperature[1], [unpack_data[2]], [unpack_data[16]])
            self.add_data(self.Pressure[1], [unpack_data[2]], [unpack_data[17]])

            self.ui.Packet2.setItem(0, 1, QTableWidgetItem(str(unpack_data[1])))
            self.ui.Packet2.setItem(1, 1, QTableWidgetItem(str(unpack_data[2])))
            self.ui.Packet2.setItem(2, 1, QTableWidgetItem(str(unpack_data[3])))
            self.ui.Packet2.setItem(3, 1, QTableWidgetItem(str(unpack_data[4])))
            self.ui.Packet2.setItem(4, 1, QTableWidgetItem(str(unpack_data[5])))
            self.ui.Packet2.setItem(5, 1, QTableWidgetItem(str(unpack_data[6])))
            self.ui.Packet2.setItem(6, 1, QTableWidgetItem(str(unpack_data[7])))
            self.ui.Packet2.setItem(7, 1, QTableWidgetItem(str(unpack_data[8])))
            self.ui.Packet2.setItem(8, 1, QTableWidgetItem(str(unpack_data[9])))
            self.ui.Packet2.setItem(9, 1, QTableWidgetItem(str(unpack_data[10])))
            self.ui.Packet2.setItem(10, 1, QTableWidgetItem(str(unpack_data[11])))
            self.ui.Packet2.setItem(11, 1, QTableWidgetItem(str(unpack_data[12])))
            self.ui.Packet2.setItem(12, 1, QTableWidgetItem(str(unpack_data[13])))
            self.ui.Packet2.setItem(13, 1, QTableWidgetItem(str(unpack_data[14])))
            self.ui.Packet2.setItem(14, 1, QTableWidgetItem(str(unpack_data[15])))
            self.ui.Packet2.setItem(15, 1, QTableWidgetItem(str(unpack_data[16])))
            self.ui.Packet2.setItem(16, 1, QTableWidgetItem(str(unpack_data[17])))
            self.ui.Packet2.setItem(17, 1, QTableWidgetItem(str(unpack_data[18])))

    def connect_action(self):
        self.data_manager.reconnect()

    def start_action(self):
        self.show_data = True

    def stop_action(self):
        self.show_data = False

    def reset_action(self):
        all_plot_item_lists = [
            self.Accelerometer, self.Giroscope, self.Magnetometer,
            self.Pressure, self.Height, self.Temperature,
            self.Gases_consentration, self.SP_parameters, self.Illumination
        ]
        for plot_item_list in all_plot_item_lists:
            if plot_item_list: # Check if the list itself is not None or empty
                for item in plot_item_list: # item is a PlotDataItem
                    if item: # Check if item is not None
                        # Reset to the initial (0,0) state to match how set_line initializes them
                        item.setData(x=[0], y=[0])


if __name__ == "__main__":
    app = QApplication([])
    widget = gcs()
    #widget.show()
    sys.exit(app.exec_())
