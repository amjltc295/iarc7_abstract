#! /usr/bin/env python
import sys
import rospy
import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import OdometryArray
from iarc7_safety.SafetyClient import SafetyClient
from geometry_msgs.msg import (PointStamped,
                               PoseStamped,
                               PoseWithCovarianceStamped,
                               TransformStamped,
                               TwistStamped,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from iarc7_msgs.srv import SetBoolOn, SetBoolOnRequest
import tf

count=0
#state 0 look for roomba
#state 1 land on roomba
#state 2 land infront of roomba
state="look"
tap_service=None

#callback giving roomba locations as a OdometryArray
def roomba_callback(msgs):
    global count
    if msgs is not None:
        count+=1
        if (count==100):
            #highest priority->priority[0]
            #lowest priority->priority[3]
            priority=[[],[],[],[]]
            for msg in msgs.data:
		set_priorities(msg.pose.pose.position,msg,priority)
            roomba=determine_action(priority)
            rospy.logwarn(roomba.child_frame_id)
            rospy.logwarn(state)
            if state=="top touch":
                tap_service('{}/top_touch'.format(roomba.child_frame_id.replace("/base_link","")), True)
            elif state=="bumper":
                tap_service('{}/bumper'.format((roomba.child_frame_id).replace("/base_link","")),True)
            count=0

def set_priorities(roomba_position,roomba_data,priority):
    x=roomba_position.x
    y=roomba_position.y
    if((x>5 or x<-5) and y<8 or y<-5):
        priority[0].append(roomba_data)
    elif(y<0):
        priority[1].append(roomba_data)
    elif(y<5):
        priority[2].append(roomba_data)
    else:
        priority[3].append(roomba_data)

def determine_action(priorities):
    global state
    for priority in priorities:
        for roomba in priority:
		quaternion = (
		    roomba.pose.pose.orientation.x,
		    roomba.pose.pose.orientation.y,
		    roomba.pose.pose.orientation.z,
		    roomba.pose.pose.orientation.w)
        	euler=tf.transformations.euler_from_quaternion(quaternion)
                if(euler[1]<-1.22173 and euler[1]>-2.79253):
                    state="top touch"
                    return roomba
                elif(euler[1]>-2.79253 and euler[1]<1.22173):
		    state="bumper"
                    return roomba

def roomba_follow():
    global tap_service
    rospy.logwarn("IN ROOMBA_FOLLOW")
    rospy.Subscriber('roombas',OdometryArray,roomba_callback)
    rospy.wait_for_service('/sim/roomba_bumper_tap')
    tap_service = rospy.ServiceProxy('/sim/roomba_bumper_tap', SetBoolOn)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('roomba_follow_abstract')
        roomba_follow()
        while not rospy.is_shutdown():
            pass

    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Roomba follow abstract shutdown")
