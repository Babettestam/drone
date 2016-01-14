#!/usr/bin/env python
import rospy
import roslib;

# messaging
from std_msgs.msg import Empty

# Receiving drone data
from ardrone_autonomy.msg import Navdata

# Get video stream from drone
from DroneVideoDisplay import DroneVideoDisplay

# GUI Library
from PySide import QtCore, QtGui

def callback(navdata):
    print("\n\nbattery=%f \n vx=%f \n vy=%f \n z=%f \n yaw=%f"%(navdata.tags,navdata.vx,navdata.vy,navdata.altd,navdata.rotZ))
    # print("callback")

if __name__ == '__main__':
    import sys

    rospy.init_node('example_node', anonymous=True)
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)

    # publish commands (send to quadrotor)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    
    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    # app.exec_()

    print("ready!")
    rospy.sleep(1.0)
    
    print("takeoff..")
    #pub_takeoff.publish(Empty())
    rospy.sleep(5.0)
    
    print("land..")
    #pub_land.publish(Empty())
    
    print("done!")