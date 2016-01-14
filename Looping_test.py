#!/usr/bin/env python
import rospy
import roslib
import time

# messaging
from std_msgs.msg import Empty                                                                      
                                                                                                                            
# Receiving Data                                                                                        
from ardrone_autonomy.msg import Navdata     
from ardrone_autonomy.srv import FlightAnim     

# Get Video Stream
# from DroneVideoDisplay import DroneVideoDisplay

# GUI Library
from PySide import QtCore, QtGui

# Movement Library
from geometry_msgs.msg import Twist, Vector3


if __name__ == '__main__':
    import sys

    rospy.init_node('example_node', anonymous=True)

    # publish commands
    pub_velocity = rospy.Publisher('/cmd_vel', Twist)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    
    Animation = rospy.ServiceProxy('ardrone/setflightanimation',FlightAnim)
    

    print("Prepare to fly")
    rospy.sleep(1.0)

    pub_takeoff.publish(Empty())
    print("Flying..");
    rospy.sleep(10.0) #Wait 3 seconds to stabelize

    print("Looping")
    Animation(19, 0);
    rospy.sleep(8.0);
    # Animation(18, 0);
    # rospy.sleep(3.0);
    Animation(17, 0);
    rospy.sleep(2.0);

    print("landing..")
    rospy.sleep(3.0)
    pub_land.publish(Empty())
    rospy.sleep(2.0) #Wait 3 seconds to stabelize
    print("landed..");