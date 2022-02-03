#!/usr/bin/env python3
# coding:utf-8

#from __future__ import print_function

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys
import time 

import rospy
from feather.msg import Status, LidarData
from sensor_msgs.msg import PointCloud

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import numpy as np
from math import sqrt

from mainwindow_frontend import Ui_MainWindow
from tools import NodeSub
from motor_command_client import send_command_client
from coordinate_client import send_coordinates
from csv_functions import *


class Worker(QThread):
    '''
    Worker thread
    '''
    def __init__(self, Qthread_data, *args, **kwargs):
        super(Worker, self).__init__()

        self.Qthread_data = Qthread_data
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):

        self.Qthread_data(*self.args, **self.kwargs)


class MyMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = Ui_MainWindow() #to create frontend widgets object
        self.ui.setupUi(self)

        rospy.init_node('main_gui_node')
        #self.ros.start_node()
        #self.ros.subscriber()

        #self.ros.subscriber("status", Status, callback1)
        #self.ros.subscriber("lidar_data", LidarData, callback2)
        #self.ros_lidar.start_node()

        #self.showMaximized()    

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.get_lidar_data)
        self.timer.start()

        self.activate_ROSthread()

        '''self.ui.pushButton_2.setCheckable(True)
        self.ui.pushButton_2.toggle()
        self.ui.pushButton_2.clicked.connect(self.btnstate)'''

        self.ui.pushButton_2.clicked.connect(self.down_click)
        self.ui.pushButton_3.clicked.connect(self.left_click)
        self.ui.pushButton_4.clicked.connect(self.right_click)
        self.ui.pushButton.clicked.connect(self.up_click)

        '''Init cartography widget'''

        '''self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)'''

        self.my_plot = pg.PlotWidget()
        self.my_plot.getPlotItem().hideAxis('bottom')
        self.my_plot.getPlotItem().hideAxis('left')
<<<<<<< HEAD
        self.my_plot.setRange(xRange = [0,1000], yRange = [0,1000])
        self.my_plot.hideButtons()
        self.my_plot.setBackground('w')
        self.my_plot.setMinimumSize(800,650)
        self.my_plot.setMaximumSize(800,650)
=======
        self.my_plot.setRange(xRange = [-12,12], yRange = [-12,12], disableAutoRange = True)
>>>>>>> e65696d1d9b1b8534531a7fd81b3497af0f42ead
        self.ui.verticalLayout_4.addWidget(self.my_plot)
        self.plot = self.my_plot.plot(pen=None, symbolSize=5) #create an object "plot"
        self.my_plot.scene().sigMouseClicked.connect(self.mouse_clicked)

    def get_timer_data(self): #receives data from ROS using a timer 
        self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
        self.ui.label.setText(F"{self.msg.battery}")

    def get_lidar_data(self): #receives data from ROS using a timer 
        self.list_x = []
        self.list_y = []
        self.points_lidar = rospy.wait_for_message('/slam_cloud', PointCloud, timeout = 5)
        #self.points_lidar = rospy.wait_for_message('/lidar_points', LidarData, timeout = 5)
        for point in self.points_lidar.points:
            self.list_x.append(point.x)
            self.list_y.append(point.y)
        self.onNewData(self.list_x, self.list_y)

    def get_Qthread_data(self):
        previous_value = -1
        while True:
            self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
            current_value = self.msg.battery
            if previous_value != current_value:
                self.ui.label_4.setText(F"{self.msg.battery}")

            previous_value = current_value

    def activate_ROSthread(self): #receives data from ROS using a Qthread 
        self.worker = Worker(self.get_Qthread_data)
        self.worker.start()

    def down_click(self):
        #self.ui.label.setText("Down")
        print("down")
        send_command_client(1)

    def left_click(self):
        #left
        print('left')
        send_command_client(2)
        self.my_plot.setRange(xRange = [-12,12], yRange = [-12,12], disableAutoRange = True)

    def right_click(self):
        #right
<<<<<<< HEAD
        print('right')
        send_command_client(3)
=======
        print('3')
        #send_command_client(3)
        file_name = F"gui_{generate_file_name()}"
        save_csv_data(file_name, self.points_lidar)
>>>>>>> e65696d1d9b1b8534531a7fd81b3497af0f42ead

    def up_click(self):
        #up
        print('Up')
        send_command_client(4)



    #def setData(self, x, y):
     #   self.plotDataItem.setData(x, y)

    def onNewData(self, x, y):
        #seria muy interesante hacer un cuadrito blanco de 13x13 para decir que ese es el robot para saber exactamente donde estamos y que es lo que estamos viendo
        self.plot.setData(x,y)

    def mouse_clicked(self, mouseClickEvent):
        self.button_pressed = mouseClickEvent.button()
        if self.button_pressed == 1:
            pos = mouseClickEvent.scenePos()
            pos_x = pos.x()
            pos_y = pos.y()
            print(pos_x, pos_y)
            size_ = self.my_plot.size()

            #send_coordinates(pos_x, pos_y)
            val_x = mapping(0, size_.width(), -12, 12, pos_x)
            val_y = mapping(size_.height(), 0, -12, 12, pos_y)
            print(F"approx location (x = {val_x}m, y = {val_y}m)")
            print(F"distance = {sqrt(val_x **2 + val_y **2)}")

def mapping(min_init, max_init, min_fin, max_fin, value):
    m = float((max_fin - min_fin) / (max_init - min_init))
    return m * (value - max_init) + max_fin


if __name__ == '__main__':

    app = QApplication(sys.argv)
    myMainWindow = MyMainWindow()
    myMainWindow.setWindowTitle("ROBOT_CONTROL")
    #myMainWindow.setFixedSize(1000,900)
    myMainWindow.resize(1000,900)
    myMainWindow.show()
    sys.exit(app.exec_())

