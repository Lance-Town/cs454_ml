import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

from pymodbus.datastore import (
    ModbusSlaveContext,
    ModbusServerContext,
    ModbusSparseDataBlock,
)
from pymodbus.server import StartTcpServer

from create3_status_node import RobotStatusNode
from numpy import uint16
import rclpy

import threading

lock = threading.Lock()

def updating_task(context):
    register_type = 3 #input register
    device_id = 0x01
    address = 0x01
    count = 1

    rclpy.init()
    create3_sn = RobotStatusNode()

    while True:
        with lock:
            if (rclpy.ok()):
                rclpy.spin_once(create3_sn, timeout_sec=1.0)
            print(context[device_id].getValues(register_type,address,count=count))

            battery = uint16(create3_sn.get_battery())
            position = create3_sn.get_position()

            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x01, count=1, payload=[uint16(battery)])
            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x02, count=1, payload=[uint16(position[0])])
            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x03, count=1, payload=[uint16(position[1])])

def setup_server():
    def get_datablock():
        return ModbusSparseDataBlock([0]*100)
    
    slave_context = ModbusSlaveContext(di=get_datablock(),co=get_datablock(),hr=get_datablock(),ir=get_datablock())
    context = ModbusServerContext(slaves = slave_context, single=True)
    return context

def publish_to_address(context, register_type, device_id, address, count, payload):
    context[device_id].setValues(register_type,address, payload)

def run_server(context):
    ip = "172.29.208.15"
    port = 5020
    log.info(f"Starting server, listing on {port} {ip}")

    update_thread = threading.Thread(target=updating_task, args=(context,))
    update_thread.daemon = True
    update_thread.start()

    StartTcpServer(
        context = context,
        address = (ip,port),
    )

def main():
    run_args = setup_server()
    run_server(run_args)

if __name__ == "__main__":
    main()
