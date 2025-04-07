import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle
from irobot_create_msgs.msg import HazardDetectionVector

from pynput.keyboard import KeyCode
from threading import Lock
from rclpy.executors import MultiThreadedExecutor

import random
import math


# To help with Multithreading
lock = Lock()

class Slash(Node):
    """
    Class to coordinate actions and subscriptions
    """

    def __init__(self, namespace='create3_03EE'):
        super().__init__('slasher')

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        cb_Subscripion = MutuallyExclusiveCallbackGroup()
        cb_Action =MutuallyExclusiveCallbackGroup()

        # Subscription to Hazards, the callback function attached only looks for bumper hits
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data,callback_group=cb_Subscripion)

        # Action clients for movements
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',callback_group=cb_Action)
        self._dock_ac = ActionClient(self, Dock, f'/{namespace}/dock',callback_group=cb_Action)
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance',callback_group=cb_Action)
        self._rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle',callback_group=cb_Action)

        # Variables
        self._goal_uuid = None



    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and if its
        a 'bump' message, cancel the current action. 

        For this to work, make sure you have:
        ros__parameters:
            reflexes_enabled: false
        in your Application Configuration Parameters File!!!
        '''

        # If it wasn't doing anything, there's nothing to cancel
        if self._goal_uuid is None:
            return

        # msg.detections is an array of HazardDetection from HazardDetectionVectors.
        # Other types can be gotten from HazardDetection.msg
        for detection in msg.detections:
            if detection.type == 1:   #If it is a bump
                self.get_logger().warning('HAZARD DETECTED')
                with lock: # Make this the only thing happening
                    self.get_logger().warning('CANCELING GOAL')           
                    self._goal_uuid.cancel_goal_async()
                    # Loop until the goal status returns canceled
                    while self._goal_uuid and self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        pass    
                    print('Goal canceled.')


#--------------------Async send goal calls-----------------------------
    def sendDriveGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        
        with lock:
            drive_handle = self._drive_ac.send_goal_async(goal)
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal
            
            # Hold ID in case we need to cancel it
            self._goal_uuid = drive_handle.result() 

        while self._goal_uuid and self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid and self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            if self._goal_uuid is None or self._goal_uuid.status is GoalStatus.STATUS_CANCELED :
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass

        # Reset the goal ID, nothing should be running
        self._goal_uuid = None 

    def sendRotateGoal(self,goal):
        """
        Sends a rotate goal asynchronously and 'blocks' until the goal is complete
        """
        self.get_logger().warning('DBG::sendRotateGoal::Start of Method')
        with lock:
            rotate_handle = self._rotate_ac.send_goal_async(goal)
            while not rotate_handle.done():
                self.get_logger().warning('DBG::sendRotateGoal::Rotate_handle not done')
                pass # Wait for Action Server to accept goal
            
            self.get_logger().warning('DBG::sendRotateGoal::Rotate_handle is DONE')
            # Hold ID in case we need to cancel it
            self._goal_uuid = rotate_handle.result() 

        while self._goal_uuid and self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            self.get_logger().warning('DBG::sendRotateGoal::GoalStatus is UNKNOWN')
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid and self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning('DBG::sendRotateGoal::GoalStatus is NOT SUCCEEDED')
            if self._goal_uuid is None or self._goal_uuid.status is GoalStatus.STATUS_CANCELED :
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass

        # Reset the goal ID, nothing should be running
        self._goal_uuid = None 
#----------------------------------------------------------------------

    def undock_self(self):
        """
        Undocks robot
        """
        self.get_logger().warning('WAITING FOR SERVER')
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

        undock_goal = Undock.Goal()

        self._undock_ac.send_goal(undock_goal)
        self.get_logger().warning('UNDOCKED')

        return

    def drive_away(self, dist=1.0):
        """
        Drives robot forward a set distance

        @param dist: distance in meters for robot to move
        """
        # wait for DriveDistance action server (blocking)
        self.get_logger().warning('WAITING FOR SERVER')
        self._drive_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning(f'CREATING DRIVE GOAL')

        # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = dist 
        self.get_logger().warning(f'DRIVE GOAL MADE')

        self.get_logger().warning(f'DRIVING {dist:.3f} METERS')
        # send goal to async function
        self.sendDriveGoal(drive_goal)

    def dock_self(self):
        """
        Docks an undocked robot
        """
        # Freshly started
        self.get_logger().warning('WAITING FOR SERVER')
    # wait until the robot server is found and ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('CREATING DOCK GOAL')

    # create new Dock goal object to send to server
        dock_goal = Dock.Goal()

        self._dock_ac.send_goal(dock_goal)
        self.get_logger().warning('DOCKING')


    def turn_distance(self, radians=1.5708, degree=None):
        """
        Turns robot a random number of radians

        @param radians: radians to turn, defaults to 90 degree turn
        @param degree: degree to turn, optional
        """
        # Freshly started, undock
        self.get_logger().warning('WAITING FOR SERVER')
    # wait until the robot server is found and ready to receive a new goal
        self._rotate_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('CREATING TURN GOAL')

    # create new Undock goal object to send to server
        rotate_goal = RotateAngle.Goal()
        self.get_logger().warning('TURN GOAL CREATED')
        # rotate_goal.angle = 0.5 * math.pi
        rotate_goal.angle = radians

        # use degrees if passed in
        if degree != None:
            rotate_goal.angle = math.radians(degree)

        self.get_logger().warning('ROTATING')

        self.sendRotateGoal(rotate_goal)

    def test_path(self):
        """
        Test path for the manufacturing line

        TODO: Remove to its own python package, but this will work for now
        """

        # undock
        self.undock_self()

        # Drive 2 meters out
        self.drive_away(dist=2.0)
        
        # turn 90 degrees
        self.turn_distance()

        # # move .75 meters 
        # self.drive_away(dist=0.75)

        # # In position to send action client to Robot2 to pick up the dice.
        # # This would end this initial action, and now it should be waiting for Robot 2 to send it an action
        # # to move to Robot 5s position.

        # # the rest is temporary code to just go and dock itself
        # self.turn_distance(degree=180.0)

        # self.drive_away(0.75)

        # self.turn_distance(degree=-90.0)

        # self.drive_away(dist=1.5)

        # self.dock_self()
        return True # TODO : Determine if this really is the best method of returning. How will false ever be sent?


def main():
    rclpy.init()

    namespace = 'create3_03EE'
    s = Slash(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(s)

    # keycom = KeyCommander([
    #     (KeyCode(char='r'), s.drive_away),
    #     (KeyCode(char='d'), s.dock_self),
    #     (KeyCode(char='t'), s.turn_distance),
    #     (KeyCode(char='u'), s.undock_self),
    #     (KeyCode(char='p'), s.test_path),
    #     ])

    # print("u: Undock robot")
    # print("r: Start drive_away")
    # print("d: Dock robot")
    # print("t: Turn robot 90 degrees")
    # print("p: Run Test Path")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        s.get_logger().info('KeyboardInterrupt, shutting down.\n')
    rclpy.shutdown()

if __name__ == '__main__':
    main()