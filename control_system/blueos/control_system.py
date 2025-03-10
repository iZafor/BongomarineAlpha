import math
from pymavlink import mavutil
import util
import lgpio
import threading
import time
import ms5837

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
                hover_height: float,
                max_hover_error: float,
                kill_switch_gpio: int,
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
        
        self.hover_height = hover_height
        self.max_hover_error = max_hover_error
        
        self.gpio_handle = None
        self.kill_switch_gpio = kill_switch_gpio

        self.bar_sensor = ms5837.MS5837_30BA()
        
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
    
    def claim_gpio(self) -> bool:
        try:
            self.gpio_handle = lgpio.gpiochip_open(0)
            try:
                lgpio.gpio_free(self.gpio_handle, self.kill_switch_gpio)
            except:
                pass
            
            lgpio.gpio_claim_input(self.gpio_handle, self.kill_switch_gpio)
            return True
        except Exception as e:
            print(f"Failed to calim gpio. Error: {e}")
            return False
    
    def free_gpio_kill_switch(self) -> bool:
        try:
            lgpio.gpio_free(self.gpio_handle, self.kill_switch_gpio)
            return True
        except Exception as e:
            print(f"Failed to free kill switch. Error: {e}")
            return False
    
    def init_bar_sensor(self) -> bool:
        try:
            return self.bar_sensor.init()
        except Exception as e:
            print(e)
        return False
    
    def read_bar_sensor(self) -> float:
        if self.bar_sensor.read():
            return self.bar_sensor.pressure()
        print("Failed to read pressure!")
        return None
    
    def read_kill_switch_status(self) -> bool:
        """
        returns True if kill switch is connected, None if falied to read status
        """
        try:
            return lgpio.gpio_read(self.gpio_handle, self.kill_switch_gpio) == False
        except Exception as e:
            print("Falied to read kill switch status! Error:", e)
            return None
    
    def read_ping_sensor(self) -> any:
        return util.read_message(self.conn, "DISTANCE_SENSOR", 1)
     
    def calculate_vertical_thrust(self, error: float, max_error: float) -> float:
        pwm = 1500
        
        if error < -5:
            pwm = util.map_value(error, 0, max_error, 1630, 1750)
        
        if error > -5:
            pwm = util.map_value(error, 0, max_error, 1680, 1650)
            
        return util.constrain_value(pwm, 1350, 1750)
            
    def control_hover(self, current_height: float):
        error = self.hover_height - current_height
        print("Error:", error)
        vertical_thrust = self.calculate_vertical_thrust(error, self.max_hover_error)
        for i in range(len(self.vertical_pwms)):
            self.vertical_pwms = vertical_thrust 
            
    def update_heading_control(self, target_angle: float, tolerance: float = 1.0):
        """Rotate the vehicle to a target angle using the shortest path
        
        Args:
            target_angle (float): Target angle in degrees (-179 to 179)
            tolerance (float, optional): Angle difference tolerance in degrees. Defaults to 5.0.
        """
        while True:
            attitude_data = util.read_message(self.conn, "ATTITUDE", 1)
            if attitude_data:
                # Convert current yaw to degrees (-179 to 179)
                current_yaw = math.degrees(attitude_data.yaw)
                print("current yaw:", current_yaw)
                
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
                pwm = min(abs(diff), 250)  # Limit maximum PWM deviation
                print("pwm:", pwm)
                pwms = [1500] * len(self.horizontal_thrusters)
                
                # Rotate clockwise if diff is positive, counterclockwise if negative
                if diff > 0:  # Clockwise rotation
                    # Row 1: Thruster 1 (CCW) forward, Thruster 2 (CW) backward
                    pwms[0] = 1500 + pwm  # Thruster 1 forward
                    pwms[1] = 1500 - pwm  # Thruster 2 backward
                    # Row 2: Thruster 3 (CW) forward, Thruster 4 (CCW) backward
                    pwms[2] = 1500 - pwm  # Thruster 3 forward
                    pwms[3] = 1500 + pwm  # Thruster 4 backward
                else:  # Counter-clockwise rotation
                    # Row 1: Thruster 1 (CCW) backward, Thruster 2 (CW) forward
                    pwms[0] = 1500 - pwm  # Thruster 1 backward
                    pwms[1] = 1500 + pwm  # Thruster 2 forward
                    # Row 2: Thruster 3 (CW) backward, Thruster 4 (CCW) forward
                    pwms[2] = 1500 + pwm  # Thruster 3 backward
                    pwms[3] = 1500 - pwm  # Thruster 4 forward
                    
                # Apply the PWM values to thrusters
                for ch, pwm in zip(self.horizontal_thrusters, pwms):    
                    self.conn.set_servo(ch, pwm)
                    print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
                    util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)
            else:
                break
        self.stop()

    def validate_pwms(self, min_pwm: float = 1000.0, max_pwm: float = 2000.0):
        for i in range(len(self.horizontal_pwms)):
            self.horizontal_pwms[i] = util.constrain_value(self.horizontal_pwms[i], min_pwm, max_pwm)
            self.vertical_pwms[i] = util.constrain_value(self.vertical_pwms[i], min_pwm, max_pwm)
    
    def move(self):
        for ch, pwm in zip(self.horizontal_thrusters, self.horizontal_pwms):
            self.conn.set_servo(ch, pwm)
            # print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            # util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)
            
        for ch, pwm in zip(self.vertical_thrusters, self.vertical_pwms):
            self.conn.set_servo(ch, pwm)
            # print(f"Sent MAV_CMD_DO_SET_SERVO: channel={ch}, PWM={pwm}, waiting for ACK...")
            # util.verify_command_received(self.conn, CONST.MAV_CMD_DO_SET_SERVO)

    def move_upward(self, vd: float):
        """
        Args:
            vd (float): pwm deviation from 1500 for vertical thrusters
        """
        
        if self.vertical_pwms:
            self.vertical_pwms[0] = 1500 + vd
            self.vertical_pwms[1] = 1500 + vd
            self.vertical_pwms[2] = 1500 + vd
            self.vertical_pwms[3] = 1500 + vd
        else:
            print("No horizontal thruster channel provided!")
            
        self.move()
        
    def move_downward(self, vd: float):
        """
        Args:
            vd (float): pwm deviation from 1500 for vertical thrusters
        """
        
        if self.vertical_pwms:
            self.vertical_pwms[0] = 1500 - vd
            self.vertical_pwms[1] = 1500 - vd
            self.vertical_pwms[2] = 1500 - vd
            self.vertical_pwms[3] = 1500 - vd
        else:
            print("No horizontal thruster channel provided!")

        self.move()      

    def move_forward(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 + hd
            self.horizontal_pwms[1] = 1500 + hd
            self.horizontal_pwms[2] = 1500 - hd
            self.horizontal_pwms[3] = 1500 - hd
        else:
            print("No horizontal thruster channel provided!")

        self.move()      
    
    def move_backward(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 - hd # backward
            self.horizontal_pwms[1] = 1500 - hd # backward
            self.horizontal_pwms[2] = 1500 + hd # forward
            self.horizontal_pwms[3] = 1500 + hd # forward
        else:
            print("No horizontal thruster channel provided!")

        self.move()
    
    def move_right(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 + hd 
            self.horizontal_pwms[1] = 1500 - hd 
            self.horizontal_pwms[2] = 1500 + hd 
            self.horizontal_pwms[3] = 1500 - hd 
        else:
            print("No horizontal thruster channel provided!")
        
        self.move()

    
    def move_left(self, hd: float):
        """
        Args:
            hd (float): pwm deviation from 1500 for horizontal thrusters 
        """
        if self.horizontal_thrusters:
            self.horizontal_pwms[0] = 1500 - hd 
            self.horizontal_pwms[1] = 1500 + hd 
            self.horizontal_pwms[2] = 1500 - hd 
            self.horizontal_pwms[3] = 1500 + hd 
        else:
            print("No horizontal thruster channel provided!")
        
        self.move()
    
    def stop(self):
        for i in range(len(self.horizontal_pwms)):
            self.horizontal_pwms[i] = 1500

        for i in range(len(self.vertical_pwms)):
            self.vertical_pwms[i] = 1500
            
        self.move()
                
    def manual_control(self):
        hd, vd = 100, 100
        kill_switch_active = False
        stop_thread = False

        if self.claim_gpio():
            print("GPIO claimed successfully")
        else:
            print("Failed to claim GPIO")
            
        if self.init_bar_sensor():
            print("Bar sensor initialized successfully")
        else:
            print("Failed to initialize bar sensor")

        def check_kill_switch():
            nonlocal kill_switch_active, stop_thread
            while not stop_thread:
                try:
                    if lgpio.gpio_read(self.gpio_handle, self.kill_switch_gpio):
                        if not kill_switch_active:
                            kill_switch_active = True
                            print("\nKill switch activated - Stopping motors")
                            self.stop()
                    else:
                        kill_switch_active = False
                    time.sleep(0.1)  # Check every 100ms
                except Exception as e:
                    print("Failed to read kill switch status!")
                    print(e)
                    break

        try:
            self.enable_rc_control()

            # Start kill switch monitoring thread
            kill_switch_thread = threading.Thread(target=check_kill_switch)
            kill_switch_thread.start()

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
ch <channel> <pwm> -> Set pwm value to channel
: """)

                if kill_switch_active:
                    print("Cannot execute command while kill switch is active")
                    continue

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
                    case x if x.startswith("ch "):
                        try:
                            ch = int(command.split()[1])
                            if not (1 <= ch <= 8):
                                raise ValueError("channel should be within 1 to 8")
                            pwm = float(command.split()[2])
                            if not (1000 <= pwm <= 2000):
                                raise ValueError("pwm should be within 1000 to 2000")
                            self.conn.set_servo(ch, pwm)
                            print(f"channel {ch} set to {pwm} pwm")
                        except ValueError as e:
                            print(f"Invalid value: {str(e)}")
                    case x if x.startswith("deg "):
                        try:
                            deg = ControlSystem.validate_rotation_deg(float(command.split()[1]))
                            print(f"Rotating to {deg} degrees")
                            self.update_heading_control(deg)
                        except ValueError as e:
                            print(f"Invalid value: {str(e)}")
                    case _:
                        print("Invalid command")
        except KeyboardInterrupt:
            print("\nKeyboard interrupt - Stopping motors and exiting...")
            self.stop()
        except Exception as e:
            print(f"Unexpected error occurred: {str(e)}")
            self.stop()
        finally:
            stop_thread = True  # Signal thread to stop
            kill_switch_thread.join()  # Wait for thread to finish
            try:
                lgpio.gpio_free(self.gpio_handle, self.kill_switch)
                lgpio.gpiochip_close(self.gpio_handle)
            except:
                print("Failed to free GPIO resources")
            print("GPIO resources freed")

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