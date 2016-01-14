#!/usr/bin/env python
import rospy
import roslib
import time

# Needed For Messaging
from std_msgs.msg import Empty                                                                      
                                                                                                                            
# Needed For Receiving Data                                                                                        
from ardrone_autonomy.msg import Navdata     

# Needed For The GUI Library
from PySide import QtCore, QtGui

# Needed For Drone Movement
from geometry_msgs.msg import Twist, Vector3

#Global variables
SearchForMarkers = False

def getNewMarker(tags, t_width): # Function For Searching For Another Marker

    # Make Sure the Drone Is Hovering, Otherwise It Turns Weird
    pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

    # Check If We Have Not Found Any Markers
    if tags <= 0: 
        # Start Rotating To The Right
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5)))
        
        # Let The Drone Sleep.
        rospy.sleep(3.0)

    # Check If We Have Found Markers
    elif tags > 0:
        
        # Check If Marker Is Further Away
        if t_width < 70:
            # Make Sure The Drone Is Hovering
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.0)))
            
            # Set The Global SearchForMarkes To True
            # Global Is Needed, Otherwise It Will Make A Other Private Boolean
            global SearchForMarkers
            SearchForMarkers = False
        
        else:
            # Make The Drone Continue Search For Markers
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5)))
            

def FlyToTag(tagCount, tWidth, tHeight): # Function To Make The Drone Fly To The Tag
    
    # Check If Boolean Is False
    if(SearchForMarkers is False):
        # If False: We Have Found A Marker And We Are Not Nearby Yet.
        
        if tWidth <= 40:
            # Tag Is Far Away! Fly Hard To Get Closer.
            pub_velocity.publish(Twist(Vector3(0.3,0,0),Vector3(0,0,0))) 

        elif tWidth <= 60:
            # Tag Is Coming Closer. Move Forward Little Slower.
            pub_velocity.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,0)))

        elif tWidth <= 80:
            # Tag Is Coming Really Close. Move Forward Slower.
            pub_velocity.publish(Twist(Vector3(0.05,0,0),Vector3(0,0,0)))

        else:
            # Tag Is Reached! Stop Moving Forward!
            pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0)))
            
            # Give The Drone Some Time To Stop Moving.
            rospy.sleep(1.0)

            # Set The Global SearchForMarkes To True
            # Global Is Needed, Otherwise It Will Make A Other Private Boolean
            global SearchForMarkers
            SearchForMarkers = True

            # Call Function To Get New Marker
            getNewMarker(tagCount, tWidth);

    else:
        # We Are At Marker, We Need A New Marker 
        getNewMarker(tagCount, tWidth);

def rotateDrone(tagCount, width, height, tagX): # Function To Rotate Drone Towards Tag.
   
    # Make Sure The Drone Is Not Flying Forwards And Is Hovering.
    pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.0)))
    
    # Check If TagX Is Smaller Than 400.
    # If Smaller: The Tag is On The Left, So We Rotate To The Left.
    if(tagX < 400):
        #Rotate Drone To The Left Towards Tag.
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,0.5)))

    # Check If TagX Is Bigger Than 600.
    # If Bigger: The Tag is On The Right, So We Rotate To The Right.
    elif(tagX > 600):
       
        pub_velocity.publish(Twist(Vector3(0.0,0,0),Vector3(0,0,-0.5)))

    # If TagX Is Betwen 400 and 600 The Drone is Good To Go.
    else:
        # Drone Knows His Direction, Fly To The Tag.
        FlyToTag(tagCount, width, height)

def callback(navdata): #Callback Function, This Function Will be Called When New Data Is In the '/ardrone/navdata' Topic.
    
        # Get The State Of The Drone
        dStatus = navdata.state
        # 1: Inited       5: DEBUGG
        # 2: Landed       6: Taking off
        # 3,7: Flying     8: Landing
        # 4: Hovering     9: Looping
    
    # Check If Boolean Is True
    # Boolean Is True When Drone Is Rotating And Searching For Tags.
    if(SearchForMarkers is True):
        # If True: Drone Was Already Searching For Tags.

        tagCount    = navdata.tags_count
        tagId       = 0

        # Check If Drone Found Tag
        if(tagCount > 0):
            # If True: Set Tag Width
            tagWidth    = navdata.tags_width[tagId]

        else:
            # If False: Set Tag Width = 0 
            tagWidth    = 0

        # Make The Drone Rotate And Fly To The Tag
        getNewMarker(tagCount, tagWidth)
        
    # Check If The Drone Found A Tag    
    elif navdata.tags_count > 0: 
        # We Found A Tag, We Set Data To Variables 
        tagId       = 0
        tagCount    = navdata.tags_count 
        tagType     = navdata.tags_type[tagId]
        tagXc       = navdata.tags_xc[tagId]
        tagYc       = navdata.tags_yc[tagId]
        tagWidth    = navdata.tags_width[tagId]
        tagHeight   = navdata.tags_height[tagId]

        # Below Is A Print With All Tag Information, Usefull For Debugging.
        # print("\n\n Tagcount= %f \n Tagtype= %f \n TagXc= %f \n tagYc= %f \n tagWidth= %f \n tagHeight= %f"%(tagCount, tagType, tagXc, tagYc, tagWidth, tagHeight))
       
        # Make The Drone!
        mover(tagCount, tagWidth, tagHeight, tagXc, dStatus)
    else:
        # There Are No Tags Found!
        print("No tags found!")

        # Check If We Are Still Searching For Tags
        if(SearchForMarkers is False):
            # We Lost The Tag, Make The Drone Go In Hover Mode!
            pub_velocity.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))

def mover(tagCount, w, h, x, status): #Function With Drone In Air Movements
    
    #Check If The Drone Is Flying / Hover Mode
    if status == 3 or status == 4 or status == 7:
        
        # Make The Drone Rotate In Front Of The Tag
        rotateDrone(tagCount, w, h, x) 

def takeMeOff(): #Function For Taking Of The Drone
    
    print("Taking Off..")
    
    # Publisher Sends Empty Message To The '/ardrone/takeoff' topic. Drone Will Takeoff.
    pub_takeoff.publish(Empty()) 
    
    # Sleep And Wait Till Drone Is Taken Off.
    rospy.sleep(3.0) 

    # Global Variable Set To True.
    # Drone Will Start Searching For New Markers
    global SearchForMarkers
    SearchForMarkers = True

def landMe(): #Function For Landing The Drone

    print("Trying to land..")

    # Publisher Sends Empty Message To The '/ardrone/land' topic. Drone Will Land.
    pub_land.publish(Empty())

    # Sleep And Wait Till Drone Is Landed.
    rospy.sleep(1.0)

    print("Landed..")

if __name__ == '__main__':
    import sys

    # Set the init node to anonymous
    rospy.init_node('example_node', anonymous=True)

    # Subscribe To The Navdata Topic.
    # We Will Receive All Updates From The Drone.
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)

    #Public Variables 
    pub_velocity = rospy.Publisher('/cmd_vel', Twist)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    

    # Prepare The Drone To Fly.
    rospy.sleep(1.0)

    # Call Function For Taking Off.
    takeMeOff()

    # Wait 60 Seconds! This Is The Time The Drone Will Be Flying.
    rospy.sleep(60.0)

    # Call Function For Landing.
    landMe()