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
from sensor_msgs.msg import PointCloud, Image

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

from cv_bridge import CvBridge
import cv2 #3.2.0


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

        self.display_width = 640
        self.display_height = 480

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.get_lidar_data)
        self.timer.start()

        self.activate_ROSthread()

        '''self.ui.pushButton_2.setCheckable(True)
        self.ui.pushButton_2.toggle()
        self.ui.pushButton_2.clicked.connect(self.btnstate)'''

        '''self.ui.pushButton_2.clicked.connect(self.down_click)
        self.ui.pushButton_3.clicked.connect(self.left_click)
        self.ui.pushButton_4.clicked.connect(self.right_click)
        self.ui.pushButton.clicked.connect(self.up_click)'''
        self.ui.comboBox.currentTextChanged.connect(self.on_combobox_data_changed)
        self.ui.comboBox_2.currentTextChanged.connect(self.on_combobox_mode_changed)
        self.ui.frame_5.setVisible(False)  


        '''self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)'''

        '''Initialize cartography widget'''
        self.my_plot = pg.PlotWidget()
        self.my_plot.getPlotItem().hideAxis('bottom')
        self.my_plot.getPlotItem().hideAxis('left')
        self.my_plot.setRange(xRange = [-12,12], yRange = [-12,12], disableAutoRange = True)
        self.ui.verticalLayout.addWidget(self.my_plot)
        self.plot = self.my_plot.plot(pen=None, symbolSize=5) #create an object "plot"
        self.my_plot.scene().sigMouseClicked.connect(self.mouse_clicked)
    
    def show_lidar_data(self):
        self.ui.label.hide()
        self.ui.verticalLayout.addWidget(self.my_plot)

    def show_camera_data(self):
        self.my_plot.setParent(None)
        self.ui.label.show()

        cv_img = cv2.imread('/home/juanb/Documents/GUI_Robot/src/feather/src/cool_cat.jpg')
        
        # convert the image to Qt format
        qt_img = self.convert_cv_qt(cv_img)
        self.ui.label.setPixmap(qt_img)

        '''grey = QPixmap(self.display_width, self.display_height)
        grey.fill(QColor('darkGray'))
        self.ui.label.setPixmap(grey)'''

    def get_timer_data(self): #receives data from ROS using a timer 
        self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
        self.ui.label.setText(F"{self.msg.battery}")

    def get_lidar_data(self): #receives data from ROS using a timer 
        self.list_x = []
        self.list_y = []
        #self.points_lidar = rospy.wait_for_message('/slam_cloud', PointCloud, timeout = 5)
        self.points_lidar = rospy.wait_for_message('/lidar_points', LidarData, timeout = 5)
        for point in self.points_lidar.points:
            self.list_x.append(point.x)
            self.list_y.append(point.y)
        self.onNewData(self.list_x, self.list_y)
    
    def get_camera_data(self): #receives data from ROS using a timer 
        self.data = rospy.wait_for_message('/camera/rgb/image_color', Image, timeout = 5)
        
        br = CvBridge()                                                 #convert between ROS and OpenCV images
        current_frame = br.imgmsg_to_cv2(self.data)                          #convert ROS image message to OpenCV image
        bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
        #cv2.imshow("Camera Kinect", bgr_frame)
        cv2.waitKey(1)

    def get_Qthread_data(self):
        previous_value = -1
        while True:
            self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
            current_value = self.msg.battery
            if previous_value != current_value:
                self.ui.lineEdit_3.setText(F"{self.msg.acceleration}")
                self.ui.lineEdit_2.setText(F"{self.msg.speed}")
                self.ui.lineEdit.setText(F"{self.msg.angular_speed}")
                self.ui.lineEdit_4.setText(F"{self.msg.battery}")

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
        print('3')
        #send_command_client(3)
        file_name = F"gui_{generate_file_name()}"
        save_csv_data(file_name, self.points_lidar)

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

    def on_combobox_mode_changed(self, text):
        if text == "Semi-Automatic Mode":
            self.ui.frame_4.setVisible(False)  
            self.ui.frame_5.setVisible(True) 
            self.ui.label_7.setText("Choose an arrival point on the map")     
        if text == "Manual Mode":
            self.ui.frame_4.setVisible(True)  
            self.ui.frame_5.setVisible(False) 
    
    def on_combobox_data_changed(self, text):
        if text == "Lidar Data":
            self.show_lidar_data() 
        if text == "Camera":
            self.show_camera_data()
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.display_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
            

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

