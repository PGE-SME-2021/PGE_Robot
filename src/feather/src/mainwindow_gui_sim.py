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
from widget_frontend import Ui_Widget
from tools import NodeSub
from motor_command_client import send_command_client
from coordinate_client import send_coordinates
from params_client import set_params
from csv_functions import *

from cv_bridge import CvBridge
import cv2 #3.2.0


class Worker(QThread):
    '''
    Worker thread
    This is a bref explanation
    '''
    def __init__(self, Qthread_data, *args, **kwargs):
        '''Constructor
        This is the constructor
        '''
        super(Worker, self).__init__()

        self.Qthread_data = Qthread_data
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):

        self.Qthread_data(*self.args, **self.kwargs)


class MyMainWindow(QtWidgets.QMainWindow):
    ##Constructor
    # this is the constructor
    # @param self Object pointer
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
        self.bgr_frame = QPixmap(self.display_width, self.display_height)

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.start_data_acquisition)
        self.timer.start()

        self.activate_ROSthread()

        '''self.ui.pushButton_settings.setCheckable(True)
        self.ui.pushButton_settings.toggle()
        self.ui.pushButton_settings.clicked.connect(self.btnstate)'''

        self.ui.pushButton_settings.clicked.connect(self.settings_menu)#settings
        self.ui.pushButton_droite.clicked.connect(self.right_click)#right
        self.ui.pushButton_stop.clicked.connect(self.stop_click)#stop
        self.ui.pushButton_gauche.clicked.connect(self.left_click)#left
        self.ui.pushButton_arriere.clicked.connect(self.down_click)#down
        self.ui.pushButton_avant.clicked.connect(self.up_click)#up
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
        self.x_plot = [-12, 12]
        self.y_plot = [-12, 12]
        self.my_plot.setRange(xRange = self.x_plot, yRange = self.y_plot, disableAutoRange = True)
        self.my_plot.hideButtons()
        self.ui.verticalLayout.addWidget(self.my_plot)
        self.plot = self.my_plot.plot(pen=None, symbolSize=5, symbolBrush='b') #create an object "plot"
        self.plot_center = self.my_plot.plot(pen=None, symbol='+', symbolSize=12, symbolBrush='w') #create an object "plot"
        self.plot_center.setData([0],[0])
        self.my_plot.scene().sigMouseClicked.connect(self.mouse_clicked)

    def start_data_acquisition(self):
        self.get_lidar_data()
        self.get_camera_data()

    def show_lidar_data(self):
        self.ui.label.hide()
        self.ui.verticalLayout.addWidget(self.my_plot)

    def get_timer_data(self): #receives data from ROS using a timer 
        self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
        self.ui.label.setText(F"{self.msg.battery}")

    def get_lidar_data(self): #receives data from ROS using a timer 
        self.list_x = []
        self.list_y = []
        self.step = 0.5
        try:
            self.points_lidar = rospy.wait_for_message('/lidar_points', LidarData, timeout = 5)
            for point in self.points_lidar.points:
                self.list_x.append(point.x)
                self.list_y.append(point.y)
            #self.onNewData(self.list_x, self.list_y)
            self.plot.setData(self.list_x, self.list_y)
        except:
            pass

    ## Get Camera data Thread
    # This function waits for a message in '/image_sim' node
    # @param self object pointer
    def get_camera_data(self): #receives data from ROS using a timer 
        try:
            self.data = rospy.wait_for_message('/image_sim', Image, timeout = 5)
            br = CvBridge()
            current_frame = br.imgmsg_to_cv2(self.data)
            self.bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
            #cv2.imshow("Camera Kinect", bgr_frame)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

    def show_camera_data(self):
        self.my_plot.setParent(None)
        self.ui.label.show()

        #cv_img = cv2.imread('/home/juanb/Documents/GUI_Robot/src/feather/src/img1.jpg')
        # convert the image to Qt format
        qt_img = self.convert_cv_qt(self.bgr_frame)
        self.ui.label.setPixmap(qt_img)

        '''grey = QPixmap(self.display_width, self.display_height)
        grey.fill(QColor('darkGray'))
        self.ui.label.setPixmap(grey)'''

    def get_Qthread_data(self):
        previous_value = -1
        while True:
            try:
                self.msg = rospy.wait_for_message('/status', Status, timeout = 7)
                current_value = self.msg.battery
                if previous_value != current_value:
                    self.ui.lineEdit_3.setText(F"{self.msg.acceleration}")
                    self.ui.lineEdit_2.setText(F"{self.msg.speed}")
                    self.ui.lineEdit.setText(F"{self.msg.angular_speed}")
                    self.ui.lineEdit_4.setText(F"{self.msg.battery}")
            except:
                pass

            previous_value = current_value

    def activate_ROSthread(self): #receives data from ROS using a Qthread 
        self.worker = Worker(self.get_Qthread_data)
        self.worker.start()

    ## Adjust Screen
    # This function is used to change lidar widget size.
    # @param self object pointer
    # @param direction_list This gets a list of 4 numbers that increase or decrease the lidar widget point of view like this [2, 3, -3, -1]
    def adjust_screen(self, direction_list):
        #[xmin, xmax, ymin, ymax]
        self.x_plot[0] += direction_list[0]
        self.x_plot[1] += direction_list[1]
        self.y_plot[0] += direction_list[2]
        self.y_plot[1] += direction_list[3]
        self.my_plot.setRange(xRange = self.x_plot, yRange = self.y_plot, disableAutoRange = True)
        print(F"x = {self.x_plot}, y = {self.y_plot}")
    
    def settings_menu(self):
        self.settings_gui = QtWidgets.QDialog()
        self.settings_gui.ui = Ui_Widget()
        self.settings_gui.ui.setupUi(self.settings_gui)
        self.settings_gui.setWindowTitle("Robot settings")
        #self.settings_gui.setWindowIcon(QtGui.QIcon("image.ico"))
        self.settings_gui.show() 

    ## Down function
    # When down button is pressed this makes the robot go backward.
    def down_click(self):
        send_command_client(2,230,230)

    ## Stop function
    # When stop button is pressed this makes the motors stop.
    def stop_click(self):
        send_command_client(5,230,230)

    ## Left button
    # When left button is pressed this makes the robot turn left.
    def left_click(self):
        send_command_client(4,230,230)

    ## Right button
    # When right button is pressed this makes the robot turns right.
    def right_click(self):
        send_command_client(3,230,230)

    ## Up button
    # When right button is pressed this makes the robot move forward.
    def up_click(self):
        send_command_client(1,230,230)

    '''def onNewData(self, x, y):
        self.plot.setData(x,y)'''

    ## Send map coordinate
    # When we click on the lidar widget this sends the relative coordinate of the robot.
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
            send_coordinates(val_x, val_y)

    def on_combobox_mode_changed(self, text):
        if text == "Semi-Automatic Mode":
            self.ui.frame_4.setVisible(False)  
            self.ui.frame_5.setVisible(True) 
            self.ui.label_text.setText("Choose an arrival point on the map")     
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
    '''Main function
    '''
    app = QApplication(sys.argv)
    myMainWindow = MyMainWindow()
    myMainWindow.setWindowTitle("OPEN INDUS ROBOT")
    #myMainWindow.setFixedSize(1000,900)
    myMainWindow.resize(1000,900)
    myMainWindow.show()
    sys.exit(app.exec_())

