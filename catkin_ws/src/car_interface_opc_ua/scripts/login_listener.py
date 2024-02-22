#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    """
    Callback function, if data is published
    """
    #rospy.loginfo(f"Login data: {data.data}")
    LoginData = data.data.split(";")
    ClientIP, Username, Password, TimeStamp, MaxTimeLogInData = LoginData

    #rospy.loginfo(f"ClientIP: {ClientIP}, Username: {Username}, Password: {Password}, TimeStamp: {TimeStamp}, MaxTimeLogInData: {MaxTimeLogInData} min")
    rospy.loginfo("ClientIP: %s",ClientIP)
    rospy.loginfo("Username: %s", Username)
    rospy.loginfo("Password: %s",Password)
    rospy.loginfo("TimeStamp: %s",TimeStamp)
    rospy.loginfo("MaxTimeLogInData: %s mins",MaxTimeLogInData)

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
