#! /usr/bin/env python

import sys
import rospy
from iarc7_msgs.msg import OdometryArray
import actionlib
import tf2_ros
import threading
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_safety.SafetyClient import SafetyClient

class RoombaRequest(object):
    def __init__(self, frame_id, TRACKING_mode, time_to_hold_once_done):
        self.frame_id = frame_id
        self.TRACKING_mode = TRACKING_mode
        self.time_to_hold = time_to_hold_once_done

class RoombaRequestExecuterState(object):
    TRACKING = 1
    BLOCK = 2
    HIT = 3
    RECOVER_FROM_SUCCESS = 4
    HOLD_POSITION = 5
    COMPLETED = 6
    FAILED_TASK = 7
    FAILED_RECOVERY = 8
    INVALID_STATE = 9

class RoombaRequestExecuter(object): 

    def __init__(self, roomba_request, client, status_callback):
        """
        Roomba Request Executer constructor 

        Args:
            roomba_request (RoombaRequest): object with request details from AI
            client (SimpleActionClient): client to send action commands to
            status_callback: callback funtion to provide status updates to 
        """

        self._status_callback = status_callback
        self._client = client
        roomba_id = roomba_request.frame_id

        # removes the "/frame_id" at end of roomba_id
        self._roomba_id = roomba_id[0:len(roomba_id)-10]
        self._time_to_hold = roomba_request.time_to_hold
        
        # True means HIT roomba, false means BLOCK
        self._TRACKING_mode = roomba_request.TRACKING_mode

        self._is_complete = False
        self._canceled = False
        self._end = False

        self._holding = (self._time_to_hold != 0)

        self._state = RoombaRequestExecuterState.TRACKING

    def _run(self):
        """
        Roomba Request Executer run fucntion 

        Description: 
            Fires off tasks until the roomba request is finished, canceled, or fails. 
        """
        while (self._state != RoombaRequestExecuterState.COMPLETED 
            and not rospy.is_shutdown() and not self._end):

            # determining goals
            if self._canceled:
                # do something to indicate canceled
                self._end = True

            elif (self._state == RoombaRequestExecuterState.FAILED_TASK):
                # recover drone first
                goal=QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()

                if not self._client.get_result():
                     rospy.logerr("Roomba Controller cannot recover drone height")
                     self._state = RoombaRequestExecuterState.FAILED_RECOVERY
                
                self._end = True

            elif self._state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                # this is where the height recover task will be called
                goal=QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("Recover Height success: {}".format(self._client.get_result()))

            elif self._state == RoombaRequestExecuterState.TRACKING:
                goal=QuadMoveGoal(movement_type="track_roomba", frame_id=self._roomba_id, 
                                    TRACKING_mode=self._TRACKING_mode)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("TrackRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaRequestExecuterState.HIT:
                goal=QuadMoveGoal(movement_type="HIT_roomba", frame_id=self._roomba_id)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("HITRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaRequestExecuterState.BLOCK:
                goal=QuadMoveGoal(movement_type="BLOCK_roomba", frame_id=self._roomba_id)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                self._client.wait_for_result()
                rospy.logwarn("BLOCKRoomba success: {}".format(self._client.get_result()))

            elif self._state == RoombaRequestExecuterState.HOLD_POSITION:
                goal=QuadMoveGoal(movement_type="HOLD_POSITION", hold_current_position = True)
                # Sends the goal to the action server.
                self._client.send_goal(goal)
                # waits to cancel hold task 
                rospy.sleep(self._time_to_hold)
                self._client.cancel_goal()
                rospy.logwarn("Hold Position Task canceled")
                self._state = RoombaRequestExecuterState.COMPLETED

            elif self._state == RoombaRequestExecuterState.COMPLETED:
                rospy.logwarn("RoombaController was successful")

            else: 
                rospy.logerr("Roomba Controller is in an invalid state")
                self._end = True
                self._state = RoombaRequestExecuterState.INVALID_STATE
            
            # state transitioning
            if not self._client.get_result():
                if self._state == RoombaRequestExecuterState.FAILED_TASK:
                    self._state == RoombaRequestExecuterState.FAILED_TASK_and_recovery
                else:
                    self._state == RoombaRequestExecuterState.FAILED_TASK

            else:
                if self._state == RoombaRequestExecuterState.TRACKING:
                    if self._TRACKING_mode:
                        self._state = RoombaRequestExecuterState.HIT
                    else:
                        self._state = RoombaRequestExecuterState.BLOCK

                elif self._state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                    if self._holding:
                        self._state = RoombaRequestExecuterState.HOLD_POSITION
                    else:
                        self._state = RoombaRequestExecuterState.COMPLETED

                elif self._state == RoombaRequestExecuterState.HIT:
                    self._state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                elif self._state == RoombaRequestExecuterState.BLOCK:
                    self._state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                elif self._state == RoombaRequestExecuterState.HOLD_POSITION:
                    self._state == RoombaRequestExecuterState.COMPLETED

                elif self._state == RoombaRequestExecuterState.COMPLETED:
                    rospy.logwarn("RoombaController was successful")

                else: 
                    rospy.logerr("Roomba Controller is in an invalid state")
                    self._end = True
                    self._state = RoombaRequestExecuterState.INVALID_STATE

            self._status_callback(self._state)    

    def start(self):
        next_thread = threading.Thread(target=self._run)
        next_thread.setDaemon(True)
        next_thread.start()

        self._status_callback(self._state)
