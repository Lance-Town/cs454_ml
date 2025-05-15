import logging
logging.basicConfig()
log = logging.getLogger()
# log.setLevel(logging.DEBUG)

from pymodbus.datastore import (
    ModbusSlaveContext,
    ModbusServerContext,
    ModbusSparseDataBlock,
)
from pymodbus.server import StartTcpServer
import threading
from . import create3_control_node

lock = threading.Lock()

def updating_task(context):
    register_type = 3 #holding register
    device_id = 0x01
    address = 0x0
    count = 1

    with lock:
        while True:
            val = context[device_id].getValues(register_type,address,count=count)
            if val[0] == 1:
                break
        
        # server needs to be killed now
        create3_control_node.main()
        


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
    port = 5021
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
