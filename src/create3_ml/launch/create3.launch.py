from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    start_modbus = ExecuteProcess(
        cmd = ['python3', '../create3_ml/pymodbus_start_button.py'],
        name = 'start_modbus',
        shell=False,
        output = 'screen'
    )

    status_modbus = ExecuteProcess(
        cmd = ['python3', '../create3_ml/pymodbus_server_test_lance.py'],
        name = 'status_modbus',
        shell=False,
        output = 'screen'
    )

    create3_server = ExecuteProcess(
        cmd = ['ros2', 'run', 'create3_ml', 'server'],
        name = 'create3_server',
        shell=False,
        output = 'screen'
    )

    create3_control = ExecuteProcess(
        cmd = ['ros2', 'run', 'create3_ml', 'control'],
        name = 'create3_control',
        shell=False,
        output = 'screen'
    )

    return LaunchDescription([
        start_modbus,
        status_modbus,
        create3_server,
        create3_control
    ])