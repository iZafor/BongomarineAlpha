import math
from pymavlink import mavutil
from control_system import util

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
            
    def rotate(self, target_angle: float, tolerance: float = 5.0):
        """Rotate the vehicle to a target angle using the shortest path
        
        Args:
            target_angle (float): Target angle in degrees (-179 to 179)
            tolerance (float, optional): Angle difference tolerance in degrees. Defaults to 5.0.
        """
        while True:
            attitude_data = util.read_message(self.conn, "ATTITUDE")
            if attitude_data:
                # Convert current yaw to degrees (-179 to 179)
                current_yaw = math.degrees(attitude_data.yaw)
                
                # Calculate the angle difference considering the wraparound
                diff = target_angle - current_yaw
                if diff > 180:
                    diff -= 360
                elif diff < -180:
                    diff += 360
                    
                # Check if we've reached the target angle within tolerance
                if abs(diff) <= tolerance:
                    self.stop()  # Stop rotating when target reached
                    break
                    
                # Calculate PWM based on absolute difference (with scaling factor)
                pwm = min(abs(diff) * 2, 400)  # Limit maximum PWM deviation
                pwms = [1500] * len(self.horizontal_thrusters)
                
                # Rotate clockwise if diff is positive, counterclockwise if negative
                if diff > 0:  # Clockwise rotation
                    # Row 1: Thruster 1 (CCW) forward, Thruster 2 (CW) backward
                    pwms[0] = 1500 + pwm  # Thruster 1 forward
                    pwms[1] = 1500 - pwm  # Thruster 2 backward
                    # Row 2: Thruster 3 (CW) forward, Thruster 4 (CCW) backward
                    pwms[2] = 1500 + pwm  # Thruster 3 forward
                    pwms[3] = 1500 - pwm  # Thruster 4 backward
                else:  # Counter-clockwise rotation
                    # Row 1: Thruster 1 (CCW) backward, Thruster 2 (CW) forward
                    pwms[0] = 1500 - pwm  # Thruster 1 backward
                    pwms[1] = 1500 + pwm  # Thruster 2 forward
                    # Row 2: Thruster 3 (CW) backward, Thruster 4 (CCW) forward
                    pwms[2] = 1500 - pwm  # Thruster 3 backward
                    pwms[3] = 1500 + pwm  # Thruster 4 forward
                    
                # Apply the PWM values to thrusters
                for ch, pwm in zip(self.horizontal_thrusters, pwms):    
                    self.conn.set_servo(ch, pwm)
                    print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
                    util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)
            else:
                break
            
    def move(self):
        for ch, pwm in zip(self.horizontal_thrusters, self.horizontal_pwms):
            self.conn.set_servo(ch, pwm)
            print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)
            
        for ch, pwm in zip(self.vertical_thrusters, self.vertical_pwms):
            self.conn.set_servo(ch, pwm)
            print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)

    def move_upward(self, vd: float):
        """
        Args:
            vd (float): pwm deviation from 1500 for vertical thrusters
        """
        
        if self.vertical_pwms:
            for ch, clock_dir in zip(self.vertical_thrusters, self.vertical_clock_dir):
                if clock_dir:
                    self.vertical_pwms[ch] = 1500 + vd
                else:
                    self.vertical_pwms[ch] = 1500 - vd
        else:
            print("No vertical thruster channel provided!")

        self.move()
        
    def move_downward(self, vd: float):
        """
        Args:
            vd (float): pwm deviation from 1500 for vertical thrusters
        """
        
        if self.vertical_pwms:
            for ch, clock_dir in zip(self.vertical_thrusters, self.vertical_clock_dir):
                if clock_dir:
                    self.vertical_pwms[ch] = 1500 - vd
                else:
                    self.vertical_pwms[ch] = 1500 + vd
        else:
            print("No vertical thruster channel provided!")

        self.move()      

    def move_forward(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 - hd # forward
            self.horizontal_pwms[1] = 1500 + hd # forward
            self.horizontal_pwms[2] = 1500 - hd # backward
            self.horizontal_pwms[3] = 1500 + hd # backward
        else:
            print("No horizontal thruster channel provided!")

        self.move()      
    
    def move_backward(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 + hd # backward
            self.horizontal_pwms[1] = 1500 - hd # backward
            self.horizontal_pwms[2] = 1500 + hd # forward
            self.horizontal_pwms[3] = 1500 - hd # forward
        else:
            print("No horizontal thruster channel provided!")

        self.move()
    
    def move_right(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        # TODO: implement move_right
        pass
    
    def move_left(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        # TODO: implement move_left
        pass
    
    def stop(self):
        for i in range(len(self.horizontal_pwms)):
            self.horizontal_pwms[i] = 1500

        for i in range(len(self.vertical_pwms)):
            self.vertical_pwms[i] = 1500
            
        self.move()
        
    def cli_control(self):
        hd, vd = 100, 100  

        try:
            self.enable_rc_control()
            while True:
                command = input("""
Enter command:
f -> Move forward
b -> Move backward
l -> Move left
r -> Move right
u -> Move up
d -> Move down
s -> Stop
q -> Quit
hd <value> -> Set horizontal thruster value (0-500)
vd <value> -> Set vertical thruster value (0-500)
deg <value> -> Set heading angle
: """)
                match command:
                    case "f":
                        self.move_forward(hd)
                        print("Moving forward")
                    case "b":
                        self.move_backward(hd)
                        print("Moving backward")
                    case "l":
                        self.move_left(hd)
                        print("Moving left")
                    case "r":
                        self.move_right(hd)
                        print("Moving right")
                    case "u":
                        self.move_upward(vd)
                        print("Moving up")
                    case "d":
                        self.move_downward(vd)
                        print("Moving down")
                    case "s":
                        self.stop()
                        print("Stopping")
                    case "q":
                        self.stop()
                        print("Quitting")
                        break
                    case x if x.startswith("hd "):
                        try:
                            hd = ControlSystem.validate_pwm(float(command.split()[1]))
                            print(f"Horizontal deviation set to {hd}")
                        except ValueError as e:
                            print(f"Invalid value: {str(e)}")
                    case x if x.startswith("vd "):
                        try:
                            vd = ControlSystem.validate_pwm(float(command.split()[1]))
                            print(f"Vertical deviation set to {vd}")
                        except ValueError as e:
                            print(f"Invalid value: {str(e)}")
                    case x if x.startswith("deg "):
                        try:
                            deg = ControlSystem.validate_rotation_deg(float(command.split()[1]))
                            print(f"Rotating to {deg} degrees")
                            self.rotate(deg)
                        except ValueError as e:
                            print(f"Invalid value: {str(e)}")
                    case _:
                        print("Invalid command")
        except KeyboardInterrupt:
            print("\nKeyboard interrupt - Stopping motors and exiting...")
            self.stop()

    @staticmethod
    def validate_pwm(value: float) -> float:
        """Validate PWM deviation value between 0 and 500
        
        Args:
            value (float): PWM deviation value to validate
            
        Returns:
            float: Validated PWM deviation value
            
        Raises:
            ValueError: If value is outside the valid range (0-500)
        """
        if not (0 <= value <= 500):
            raise ValueError("PWM deviation must be between 0 and 500")
        return value
    
    @staticmethod
    def validate_rotation_deg(value: float) -> float:
        """Validate rotation angle value between -179 and 179
        
        Args:
            value (float): Rotation angle value to validate
            
        Returns:
            float: Validated rotation angle value
            
        Raises:
            ValueError: If value is outside the valid range (-179 to 179)
        """
        if not (-179 <= value <= 179):
            raise ValueError("Rotation angle must be between -179 and 179")
        return value