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

        rospy.init_node('node')
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

        self.ui.pushButton_2.clicked.connect(self.down_click)
        self.ui.pushButton_3.clicked.connect(self.another_click)
        self.ui.pushButton_4.clicked.connect(self.otro_click)
        self.ui.pushButton.clicked.connect(self.mas_click)

        '''Init cartography widget'''

        '''self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)'''

        self.my_plot = pg.PlotWidget()
        self.my_plot.getPlotItem().hideAxis('bottom')
        self.my_plot.getPlotItem().hideAxis('left')
        self.x_plot = [-12, 12]
        self.y_plot = [-12, 12]
        self.my_plot.setRange(xRange = self.x_plot, yRange = self.y_plot, disableAutoRange = True)
        self.ui.verticalLayout_4.addWidget(self.my_plot)
        self.plot = self.my_plot.plot(pen=None, symbolSize=5) #create an object "plot"
        self.my_plot.scene().sigMouseClicked.connect(self.mouse_clicked)

    def get_timer_data(self): #receives data from ROS using a timer 
        self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
        self.ui.label.setText(F"{self.msg.battery}")

    def get_lidar_data(self): #receives data from ROS using a timer 
        self.list_x = []
        self.list_y = []
        self.step = 0.5
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
                self.ui.label.setText(F"{self.msg.battery}")

            previous_value = current_value

    def activate_ROSthread(self): #receives data from ROS using a Qthread 
        self.worker = Worker(self.get_Qthread_data)
        self.worker.start()

    def adjust_screen(self, direction_list):
        #[xmin, xmax, ymin, ymax]
        self.x_plot[0] += direction_list[0]
        self.x_plot[1] += direction_list[1]
        self.y_plot[0] += direction_list[2]
        self.y_plot[1] += direction_list[3]
        self.my_plot.setRange(xRange = self.x_plot, yRange = self.y_plot, disableAutoRange = True)
        print(F"x = {self.x_plot}, y = {self.y_plot}")

    def down_click(self):
        #self.ui.label.setText("Down")
        print("down")
        #zoom out
        self.adjust_screen([-self.step, self.step, -self.step, self.step])
        #send_command_client(1)

    def another_click(self):
        #left
        print('2')
        send_command_client(2)
        #move left
        self.adjust_screen([-self.step, -self.step, 0, 0])

    def otro_click(self):
        #right
        #move rigth
        self.adjust_screen([self.step, self.step, 0, 0])
        print('3')
        #send_command_client(3)
        #file_name = F"gui_{generate_file_name()}"
        #save_csv_data(file_name, self.points_lidar)

    def mas_click(self):
        #up
        print('4')
        send_command_client(4)
        #zoom in
        self.adjust_screen([self.step, -self.step, self.step, -self.step])

    #def setData(self, x, y):
     #   self.plotDataItem.setData(x, y)

    def onNewData(self, x, y):
        #seria muy interesante hacer un cuadrito blanco para decir que ese es el robot para saber exactamente donde estamos y que es lo que estamos viendo
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
            val_x = mapping(0, size_.width(), self.x_plot[0], self.x_plot[1], pos_x)
            val_y = mapping(size_.height(), 0, self.y_plot[0], self.y_plot[1], pos_y)
            print(F"approx location (x = {val_x}m, y = {val_y}m)")
            print(F"distance = {sqrt(val_x **2 + val_y **2)}")

def mapping(min_init, max_init, min_fin, max_fin, value):
    m = float((max_fin - min_fin) / (max_init - min_init))
    return m * (value - max_init) + max_fin


if __name__ == '__main__':
    #rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    myMainWindow = MyMainWindow()
    myMainWindow.setWindowTitle("ROBOT_CONTROL")
    #myMainWindow.setFixedSize(850,620)
    myMainWindow.resize(850,620)
    myMainWindow.show()
    sys.exit(app.exec_())

