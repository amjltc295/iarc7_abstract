#! /usr/bin/env python
import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
import threading
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

class RoombaRequests(object):
    def __init__(self, frame_id, mode, time_to_hold_once_done):
        self.frame_id = frame_id
        self.mode = mode
        self.time_to_hold = time_to_hold_once_done

class RoombaControllerStates(object):
    tracking = 1
    block = 2
    hit = 3
    recover_from_success = 4
    hold_position = 5
    completed = 6
    failed_task = 7
    failed_recovery = 8
    invalid_state = 9

class RoombaController(object): 

    def __init__(self, roomba_request, client, status_callback):
        self._status_callback = status_callback
        self._client = client
        roomba_id = roomba_request.frame_id
        self._roomba_id = roomba_id [0:len(roomba_id)-10]
        self._time_to_hold = roomba_request.time_to_hold
        # True means hit roomba, false means block
        self._mode = roomba_request.mode

        self._is_complete = False
        self._canceled = False
        self._end = False

        self._holding = (self._time_to_hold != 0)

        self._state = RoombaControllerStates.tracking

    def _next_goal(self):
        while self._state != RoombaControllerStates.completed and not rospy.is_shutdown() and not self._end:

            # determining goals
            if self._canceled:
                # do something to indicate canceled
                self._end = True

            elif (self._state == RoombaControllerStates.failed_task):
                # recover drone first
                goal = QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()

                if not self._client.get_result():
                     rospy.logerr("Roomba Controller cannot recover drone height")
                     self._state = RoombaControllerStates.failed_recovery
                
                self._end = True

            elif self._state == RoombaControllerStates.recover_from_success:
                # this is where the height recover task will be called
                goal = QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("Recover Height success: {}".format(self._client.get_result()))

            elif self._state == RoombaControllerStates.tracking:
                goal = QuadMoveGoal(movement_type="track_roomba", frame_id=self._roomba_id, mode=self._mode)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("TrackRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaControllerStates.hit:
                goal = QuadMoveGoal(movement_type="hit_roomba", frame_id=self._roomba_id)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("HitRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaControllerStates.block:
                goal = QuadMoveGoal(movement_type="block_roomba", frame_id=self._roomba_id)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("BlockRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaControllerStates.hold_position:
                goal = QuadMoveGoal(movement_type="hold_position", hold_current_position = True)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # waits to cancel hold task 
                rospy.sleep(self._time_to_hold)
                self._client.cancel_goal()
                rospy.logwarn("Hold Position Task canceled")
                self._state = RoombaControllerStates.completed

            elif self._state == RoombaControllerStates.completed:
                rospy.logwarn("RoombaController was successful")

            else: 
                rospy.logerr("Roomba Controller is in an invalid state")
                self._end = True
                self._state = RoombaControllerStates.invalid_state
            
            # state transitioning
            if not self._client.get_result():
                if self._state == RoombaControllerStates.failed_task:
                    self._state == RoombaControllerStates.failed_task_and_recovery
                else:
                    self._state == RoombaControllerStates.failed_task

            else:
                if self._state == RoombaControllerStates.tracking:
                    if self._mode:
                        self._state = RoombaControllerStates.hit
                    else:
                        self._state = RoombaControllerStates.block

                elif self._state == RoombaControllerStates.recover_from_success:
                    if self._holding:
                        self._state = RoombaControllerStates.hold_position
                    else:
                        self._state = RoombaControllerStates.completed

                elif self._state == RoombaControllerStates.hit:
                    self._state = RoombaControllerStates.recover_from_success

                elif self._state == RoombaControllerStates.block:
                    self._state = RoombaControllerStates.recover_from_success

                elif self._state == RoombaControllerStates.hold_position:
                    self._state == RoombaControllerStates.completed

                elif self._state == RoombaControllerStates.completed:
                    rospy.logwarn("RoombaController was successful")

                else: 
                    rospy.logerr("Roomba Controller is in an invalid state")
                    self._end = True
                    self._state = RoombaControllerStates.invalid_state

            self._status_callback(self._state)    

    def run(self):
        next_thread = threading.Thread(target=self._next_goal)
        next_thread.setDaemon(True)
        next_thread.start()

        self._status_callback(self._state)
