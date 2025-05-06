import rclpy
from create3_action_client import ChangeStateActionClient
from create3_action_server import ChangeStateActionServer
from movement import Slash
from fanuc2_ml import BeakerActionClient, BunsenActionClient
from stdbot_trigger_client import TriggerClient

import time

def main():
    rclpy.init()
    create3_ac = ChangeStateActionClient()
    beaker_ac = BeakerActionClient()
    bunsen_ac = BunsenActionClient()

    print("Sending iCreate3 the goal")
    create3_ac.send_goal_and_wait(1)
    print("Goal is finished")

    print("Sending Paul the goal")
    dave_tc = TriggerClient()
    dave_tc.send_request()
    print("paul is done")

    # send goal to carson
    # beaker_ac.send_goal_and_wait(2) # send carson to pick up from paul

    # move to grant
    create3_ac.send_goal_and_wait(2)

    # send grant the status code to move the gripper back
    # bunsen_ac.send_goal_and_wait(4)

    # create3_ac.send_goal_and_wait(3)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()