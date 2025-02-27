from pymavlink import mavutil
from control_system import util
from typing import Callable

CONST = mavutil.mavlink

class ControlSystem:
    """
    Args:
        vertical_thrusters (list[int]): channels for vertical thrusters from top to bottom row by row
        horizontal_thrusters (list[int]): channels for horizontal thrusters from top to bottom row by row
        ip (str, optional): Defaults to "127.0.0.1".
        port (int, optional): Defaults to 5777.
        connection_timeout (float, optional): Defaults to 10.0.

    Raises:
        Exception: throws exception if connection is unsuccessful
    """
    
    def __init__(self, 
                vertical_thrusters: list[int],
                horizontal_thrusters: list[int],
                ip: str = "127.0.0.1", 
                port: int = 5777, 
                connection_timeout: float = 10.0):
        print(f"Connecting to a tcp connection at {ip}:{port}...")
        
        self.conn = mavutil.mavlink_connection(f"tcp:{ip}:{port}")
        msg = self.conn.wait_heartbeat(timeout=connection_timeout)
        if not msg:
            raise Exception("Failed to make connection!")
        print("Connection successful!")

        self.vertical_thrusters = vertical_thrusters
        self.vertical_pwms = [1500.0] * len(vertical_thrusters)
        
        self.horizontal_thrusters = horizontal_thrusters
        self.horizontal_pwms = [1500.0] * len(horizontal_thrusters)
        
    def enable_rc_control(self):
        """
        Sets SERVOx_FUNCTION value to RCINx. e.g. RCIN1 = 50 + 1 = 51, where x is the channel number.  
        """
        
        for vt in self.vertical_thrusters:
            if not util.set_parameter(self.conn, f"SERVO{vt}_FUNCTION", 50 + vt):
                print(f"Failed to set RCIN{vt}!")
                
        for ht in self.horizontal_thrusters:
            if not util.set_parameter(self.conn, f"SERVO{ht}_FUNCTION", 50 + ht):
                print(f"Failed to set RCIN{ht}!")
            else:
                print(f"SERVO{ht}_FUNCTION is set to {50 + ht}")

    def move(self):
        for ch, pwm in zip(self.horizontal_thrusters, self.horizontal_pwms):
            self.conn.set_servo(ch, pwm)
            print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)
            
        for ch, pwm in zip(self.vertical_thrusters, self.vertical_pwms):
            self.conn.set_servo(ch, pwm)
            print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)

    def move_forward(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()      
    
    def move_backward(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_right(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_left(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_forward_right(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_forward_left(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_backward_right(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def move_backward_left(self, hd: float, vd: float, set_pwms: Callable[[list[float], list[float]], None] = None):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
            vd (float): pwm deviation from 1500 for vertical thrusters
            set_pwms (Callable[[list[float], list[float]], None], optional): A callback function that receives list of pwm values for horizontal and vertical thrusters respectively. Defaults to None.
        """
        
        if set_pwms:
            set_pwms(self.horizontal_pwms, self.vertical_pwms)
        else:
            if self.horizontal_thrusters:
                self.horizontal_pwms[0] = 1500 - hd
                self.horizontal_pwms[1] = 1500 + hd
                self.horizontal_pwms[2] = 1500 + hd
                self.horizontal_pwms[3] = 1500 - hd
            else:
                print("No horizontal thruster channel provided!")

            # TODO: vertical thrusters

        self.move()
    
    def stop(self):
        for i in range(len(self.horizontal_pwms)):
            self.horizontal_pwms[i] = 1500

        for i in range(len(self.vertical_pwms)):
            self.vertical_pwms[i] = 1500
            
        self.move()