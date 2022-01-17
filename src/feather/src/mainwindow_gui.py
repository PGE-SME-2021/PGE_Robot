#!/usr/bin/env python3
# coding:utf-8

#from __future__ import print_function

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys
import time 

import rospy
from feather.msg import Status

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication

from mainwindow_frontend import Ui_MainWindow
from status_receiver import NodeSub

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

        #self.ros = MinimalSubscriber() #to create ROS subscriber object 
        self.ros = NodeSub('status', Status, 'status_recv')
        self.ros.start_node()
        #self.ros.subscriber()

        #self.showMaximized()    

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.get_timer_data)
        self.timer.start()

        #self.activate_ROSthread()

        #self.ui.pushButton.clicked.connect(self.activate_ROSthread)
        self.ui.pushButton_2.clicked.connect(self.down_click)

    def get_timer_data(self): #receives data from ROS using a timer 
        self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
        self.ui.label.setText(F"{self.msg.battery}")

    def get_Qthread_data(self):
        previous_value = -1
        while True:
            self.msg = rospy.wait_for_message('/status', Status, timeout = 5)
            current_value = self.msg.battery
            if previous_value != current_value:
                self.ui.label.setText(F"{self.msg.battery}")
            else:
                break

            previous_value = current_value

    def activate_ROSthread(self): #receives data from ROS using a Qthread 
        self.worker = Worker(self.get_Qthread_data)
        self.worker.start()

    def down_click(self):
        self.ui.label.setText("Down")


if __name__ == '__main__':
    #rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    myMainWindow = MyMainWindow()
    myMainWindow.setWindowTitle("ROBOT_CONTROL")
    #myMainWindow.setFixedSize(850,620)
    myMainWindow.show()
    sys.exit(app.exec_())
