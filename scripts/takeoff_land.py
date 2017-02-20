#! /usr/bin/env python
import sys
import rospy

import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

def takeoff_land():
    safety_client = SafetyClient('takeoff_land_abstract')
    # Since this abstract is top level in the control chain there is no need to check
    # for a safety state. We can also get away with not checking for a fatal state since
    # all nodes below will shut down.
    assert(safety_client.form_bond())

    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    rospy.sleep(5.0)

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff", takeoff_height=1.0)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    rospy.sleep(2.0)

    # Test land
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('takeoff_land_abstract')
        takeoff_land()
    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Takeoff and land abstract shutdown")
