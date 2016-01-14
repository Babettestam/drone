#!/usr/bin/env python
import rospy
import roslib
import time

# messaging
from std_msgs.msg import Empty                                                                      
                                                                                                                            
# Receiving Data                                                                                        
from ardrone_autonomy.msg import Navdata     

# Get Video Stream
# from DroneVideoDisplay import DroneVideoDisplay

# GUI Library
from PySide import QtCore, QtGui

# Movement Library
from geometry_msgs.msg import Twist, Vector3

#Global variables
SearchForMarkers = False

def getNewMarker(tags, t_width):

    print("Looking for new tag...")
    
    if tags <= 0: 
        print("No tags found!, Search for new markers")
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5))) #DEBUGGING MODES

    elif tags > 0:
        print("Found a NEW tag");
        if t_width < 70:
            print("And the tag is far away")
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.0))) #DEBUGGING MODES
            
            global SearchForMarkers
            SearchForMarkers = False
        
        else:
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5))) #DEBUGGING MODES
            print("But it is the same tag :$");
            

def FlyToTag(tagCount, tWidth, tHeight):
    # print("FUNCTION FLY TO TAG, SearchForMarkers is: ")
    # print(SearchForMarkers);

    if(SearchForMarkers is False):
        if tWidth < 40:
            # print("Flying.. towards tag!")
            pub_velocity.publish(Twist(Vector3(0.3,0,0),Vector3(0,0,0))) #DEBUGGING MODES
        elif tWidth <= 60:
            # print("Braking.. tag is close by.")
            pub_velocity.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,0)))  #DEBUGGING MODES
        elif tWidth <= 80:
            # print("Braking.. tag is close by.")
            pub_velocity.publish(Twist(Vector3(0.05,0,0),Vector3(0,0,0)))  #DEBUGGING MODES
        else:
            print("Stop Flying.. tag reached!")
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0))) #DEBUGGING MODES
            time.sleep(0.5);
            getNewMarker(tagCount, tWidth);

            global SearchForMarkers
            SearchForMarkers = True
    else:
        getNewMarker(tagCount, tWidth);


def rotateDrone(tagCount, width, height, tagX):
    if(tagX < 400):
        # print("rotating drone to the left!");
        #rotate to left
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.5))) #DEBUGGING MODES

    elif(tagX > 600):
        # print("rotating drone to the right!");
        #rotate to right
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5))) #DEBUGGING MODES

    else:
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.0))) #DEBUGGING MODES
        FlyToTag(tagCount, width, height)

def callback(navdata): #Callback Function Including All The Navigation Data
    dStatus = navdata.state
    # print(dStatus)

    if(SearchForMarkers is True):
        # print("Already searching for new tags..");
        tagCount    = navdata.tags_count
        tagId       = 0
        if(tagCount > 0):
            tagWidth    = navdata.tags_width[tagId]
        else:
            tagWidth    = 0

        getNewMarker(tagCount, tagWidth) #Move The Drone
        
    elif navdata.tags_count > 0: # found a tag
        print("found a tag");
        tagId       = 0
        tagCount    = navdata.tags_count 
        tagType     = navdata.tags_type[tagId]
        tagXc       = navdata.tags_xc[tagId]
        tagYc       = navdata.tags_yc[tagId]
        tagWidth    = navdata.tags_width[tagId]
        tagHeight   = navdata.tags_height[tagId]

        # print("\n\n Tagcount= %f \n Tagtype= %f \n TagXc= %f \n tagYc= %f \n tagWidth= %f \n tagHeight= %f"%(tagCount, tagType, tagXc, tagYc, tagWidth, tagHeight))
        mover(tagCount, tagWidth, tagHeight, tagXc, dStatus) #Move The Drone
    else:
        print("No tags found!")
        if(SearchForMarkers is False):
            pub_velocity.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))

def mover(tagCount, w, h, x, status): #Function Which Includes All The In Air Movements.
    if status == 3 or status == 4 or status == 7:
       rotateDrone(tagCount, w, h, x) 
        # FlyToTag(w, h) #only here for debug

def takeMeOff(): #Function Which Includes The Taking Off
    print("Taking Off..")
    pub_takeoff.publish(Empty()) #DEBUGGING MODES
    rospy.sleep(3.0) #Wait 3 seconds to stabelize

    global SearchForMarkers
    SearchForMarkers = True
# 
def landMe(): #Function Which Includes The Landing
    print("Trying to land..")
    pub_land.publish(Empty()) #DEBUGGING MODES
    rospy.sleep(1.0)
    print("Landed..")

def getVideo(): #Function Which Includes The Video Streaming
    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()

    display.show()
    app.exec_()

if __name__ == '__main__':
    import sys

    rospy.init_node('example_node', anonymous=True)
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)

    # publish commands
    pub_velocity = rospy.Publisher('/cmd_vel', Twist)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    # rospy.se

    print("Prepare to fly")
    rospy.sleep(1.0)

    takeMeOff()

    rospy.sleep(60.0) #Fly Time

    landMe()