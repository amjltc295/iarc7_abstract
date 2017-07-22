#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient
from roomba_controller import RoombaController, RoombaRequests, RoombaControllerStates

def hit_roomba():
    safety_client = SafetyClient('game')
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
    rospy.sleep(2.0)

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))
    rospy.sleep(2.0)

    # change element in array to test diff roombas
    roomba_id = roomba_array.data[5].child_frame_id 

    roomba_request = RoombaRequests(roomba_id, False, 5)

    roomba_controller = RoombaController(roomba_request, client, _receive_roomba_controller_status)

    roomba_controller.run()

    while (controller_state != RoombaControllerStates.completed):
        pass

def _receive_roomba_status(data):
    global roomba_array
    roomba_array = data

def _receive_roomba_controller_status(data):
    global controller_state
    controller_state = data

if __name__ == '__main__':
    try:
        roomba_array = []
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        _roomba_status_sub = rospy.Subscriber('roombas', 
                         OdometryArray, _receive_roomba_status)

        rospy.init_node('game')
        rospy.logwarn("RUNNING")
        hit_roomba()
        while not rospy.is_shutdown():
            pass

    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Game abstract shutdown")
