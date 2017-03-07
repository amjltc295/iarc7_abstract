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

def roomba_loc_callback(data):
    rospy.logwarn("IN ROOMBA_CALLBACK")
    if data is not None:
        rospy.logwarn(data[0].pose.pose)

def roomba_follow():
    rospy.logwarn("IN ROOMBA_FOLLOW")
    rospy.Subscriber('roombas',OdometryArray,roomba_loc_callback)
    while(1):
        rospy.sleep(5.0)
        rospy.logwarn("IN ROOMBA_FOLLOW")

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
