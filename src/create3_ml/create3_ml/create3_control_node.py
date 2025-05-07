import rclpy
from .create3_action_client import ChangeStateActionClient
from .create3_action_server import ChangeStateActionServer
from .movement import Slash
from fanuc2_ml import BeakerActionClient, BunsenActionClient
from .stdbot_trigger_client import TriggerClient

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

    # move to grant
    create3_ac.send_goal_and_wait(2)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()