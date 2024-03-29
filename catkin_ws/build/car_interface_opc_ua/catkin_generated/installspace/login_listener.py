#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from datetime import datetime, timedelta
import sys
from PyQt5.QtWidgets import QApplication, QMessageBox

def callback(data):
    """
    Callback function, if data is published
    """
    #rospy.loginfo(f"Login data: {data.data}")
    LoginData = data.data.split(";")
    ClientIP, Username, Password, TimeStamp, MaxTimeLogInData = LoginData

    rospy.loginfo(f"ClientIP: {ClientIP}, Username: {Username}, Password: {Password}, TimeStamp: {TimeStamp}, MaxTimeLogInData: {MaxTimeLogInData} min")
    #ValidTill = datetime.strptime(TimeStamp, "%Y-%-m-%d %H:%M:%S") + timedelta(minutes=MaxTimeLogInData)
    #ValidTill = datetime.strptime(TimeStamp, "%a %b %d %H:%M:%S %Y") + timedelta(minutes=MaxTimeLogInData)
    #rospy.loginfo(f"Login is valid until {ValidTill}")

    # Show Pop-Up: Message
    """
    app = QApplication(sys.argv)
    pop_up = QMessageBox()
    pop_up.setText(f"ClientIP: {ClientIP} Username: {Username} Password: {Password} TimeStamp: {TimeStamp} MaxTimeLogInData: {MaxTimeLogInData} min")
    pop_up.setWindowTitle("New connection")

    pop_up.addButton(QMessageBox.Ok)
    pop_up.exec_()
    """

def LoginListener():
    """
    Simple function to listen to the OPC UA login data
    """

    # Init the node:
    rospy.init_node('login_listener', anonymous=True)

    # Subscribe to the topic:
    rospy.Subscriber('OPCUA_LoginData', String, callback)
    
    # Wait until this node is stopped:
    rospy.spin()

if __name__ == '__main__':
    LoginListener()
