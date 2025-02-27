import time
from pymavlink.mavutil import mavlink, mavtcp

CONST = mavlink

def verify_command_received(conn: mavtcp, command: any, timeout: float = 3) -> bool: 
    """
    Args:
        conn (mavtcp): Established tcp connection
        command (any): Command to verify
        timeout (float, optional): Time to wait for in seconds. Defaults to 3.

    Returns:
        bool: **True** if the command was received successfully, otherwise **False**
    """
    
    ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack_msg and ack_msg.command == command:
        result = ack_msg.result
        if result == CONST.MAV_RESULT_ACCEPTED:
            print(f"COMMAND_ACK received: {command} - ACCEPTED")
            return True
        elif result == CONST.MAV_RESULT_DENIED:
            print("COMMAND_ACK: DENIED")
        elif result == CONST.MAV_RESULT_FAILED:
            print("COMMAND_ACK: FAILED")
        elif result == CONST.MAV_RESULT_TEMPORARILY_REJECTED:
            print("COMMAND_ACK: TEMPORARILY_REJECTED")
        elif result == CONST.MAV_RESULT_UNSUPPORTED:
            print("COMMAND_ACK: UNSUPPORTED")
        else:
            print(f"COMMAND_ACK: unknown result code: {result}")
    else:
        print("No valid COMMAND_ACK received (timeout or different command).")
    return False
    
def set_parameter(
    conn: mavtcp, 
    param_name: str, 
    param_value: float, 
    param_type: any = CONST.MAV_PARAM_TYPE_REAL32, 
    timeout: float = 2) -> bool:
    """
    Sends PARAM_SET to the vehicle for 'param_name' with 'param_value'.
    param_type typically is MAV_PARAM_TYPE_INT32 (for integer) or MAV_PARAM_TYPE_REAL32 (float).
    """
    if isinstance(param_name, str):
        param_name = param_name.encode('utf-8')

    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        param_name,
        param_value, 
        param_type
    )

    start_time = time.time()
    while True:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if not msg:
            if (time.time() - start_time) > timeout:
                print(f"Timeout: no PARAM_VALUE for {param_name.decode('utf-8')}")
                return False
            continue
        if msg.param_id.encode('utf-8') == param_name:
            print(f"PARAM_VALUE received: {msg.param_id} = {msg.param_value}")
            break
    return True
