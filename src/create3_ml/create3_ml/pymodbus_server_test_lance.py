# # this module imports source code needed to interface with robot API

# import numpy as np 
# import random
# import time

# import asyncio
# import sys
# import logging

# import logging
# logging.basicConfig()
# log = logging.getLogger()
# log.setLevel(logging.DEBUG)


# from pymodbus.datastore import (
#     ModbusSlaveContext,
#     ModbusSequentialDataBlock,
#     ModbusServerContext,
#     ModbusSparseDataBlock,
# )
# from pymodbus.client import ModbusTcpClient
# client = ModbusTcpClient(host='localhost',port=5020)
# from pymodbus.server import StartAsyncTcpServer
# from pymodbus.constants import Endian
# from pymodbus.payload import BinaryPayloadDecoder
# from pymodbus.payload import BinaryPayloadBuilder

# from create3_status_node import RobotStatusNode
# from numpy import uint16
# import rclpy

# from threading import Lock

# lock = Lock()

# def updating_task(context):
#     register_type = 4 #input register
#     device_id = 0x01
#     address = 0x00
#     count = 1

#     print("\n\n\ninitializing create3")
#     rclpy.init()
#     create3_sn = RobotStatusNode()
#     print("done")

#     while True:
#         with lock:
#             if (rclpy.ok()):
#                 # print("spinning")
#                 rclpy.spin_once(create3_sn, timeout_sec=1.0)
#                 # print("done spinning")
#             # else:
#                 # print("its fucked")
#             # await asyncio.sleep(2)
#             print(context[device_id].getValues(register_type,address,count=count))

#             battery = create3_sn.get_battery()
#             # print(f"\n\n\n{battery}\n\n\n")

#             payload = [battery] #HAS TO BE A LIST
#             print(payload)
#             context[device_id].setValues(register_type,address,payload)
#             print("test")

#             txt= f"updating_task: incremented values: {payload!s} at address {address!s}"
#             log.info(txt)

# def setup_server():
#     def get_datablock():
#         return ModbusSparseDataBlock([0]*100)
    
#     slave_context = ModbusSlaveContext(di=get_datablock(),co=get_datablock(),hr=get_datablock(),ir=get_datablock())
#     context = ModbusServerContext(slaves = slave_context, single=True)
#     return context


# async def run_async_server(context):
#     log.info("Starting ASYNC server, listing on 5020 localhost")
#     await StartAsyncTcpServer(
#         context = context,
#         address = ("172.29.209.74",5020),
#     )
#     task = asyncio.create_task(updating_task(context))

# async def main():
#     run_args = setup_server()
#     await run_async_server(run_args)

# if __name__ == "__main__":
#     asyncio.run(main(), debug=True)


# this module imports source code needed to interface with robot API

import numpy as np 
import random
import time

import asyncio
import sys
import logging

import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)


from pymodbus.datastore import (
    ModbusSlaveContext,
    ModbusSequentialDataBlock,
    ModbusServerContext,
    ModbusSparseDataBlock,
)
from pymodbus.client import ModbusTcpClient
client = ModbusTcpClient(host='localhost',port=5020)
from pymodbus.server import StartAsyncTcpServer, StartTcpServer
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder

from create3_status_node import RobotStatusNode
from numpy import uint16, int16
import rclpy

from threading import Lock
import threading

lock = Lock()

def updating_task(context):
    register_type = 3 #input register
    device_id = 0x01
    address = 0x01
    address2 = 0x02
    count = 1

    print("\n\n\ninitializing create3")
    rclpy.init()
    create3_sn = RobotStatusNode()
    print("done")

    while True:
        with lock:
            if (rclpy.ok()):
                # print("spinning")
                rclpy.spin_once(create3_sn, timeout_sec=1.0)
                # print("done spinning")
            # else:
                # print("its fucked")
            # await asyncio.sleep(2)
            print(context[device_id].getValues(register_type,address,count=count))

            battery = uint16(create3_sn.get_battery())
            position = create3_sn.get_position()

            # print("\n\n\nPosition: ", position)
            # print(f"\n\n\n{battery}\n\n\n")

            # payload = [int(battery)] #HAS TO BE A LIST
            # print(payload)
            # context[device_id].setValues(register_type,address,payload)
            # print("test")

            # payload = position #HAS TO BE A LIST
            # print(int16(position[0]))
            # context[device_id].setValues(register_type,address2,payload)
            # print("test")

            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x01, count=1, payload=[uint16(battery)])
            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x02, count=1, payload=[uint16(position[0])])
            publish_to_address(context=context, register_type=0x04, device_id=0x01, address=0x03, count=1, payload=[uint16(position[1])])

            # txt= f"updating_task: incremented values: {payload!s} at address {address!s}"
            # log.info(txt)

def setup_server():
    def get_datablock():
        return ModbusSparseDataBlock([0]*100)
    
    slave_context = ModbusSlaveContext(di=get_datablock(),co=get_datablock(),hr=get_datablock(),ir=get_datablock())
    context = ModbusServerContext(slaves = slave_context, single=True)
    return context

def publish_to_address(context, register_type, device_id, address, count, payload):
    context[device_id].setValues(register_type,address, payload)

def run_async_server(context):
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
    # updating_task(context)

def main():
    run_args = setup_server()
    run_async_server(run_args)

if __name__ == "__main__":
    main()
