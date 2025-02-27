import re
from control_system.control_system import ControlSystem

hd_pattern = r"^hd_(?:[0-9]|[1-9][0-9]|[1-4][0-9]{2}|500)$"
vd_pattern = r"^vd_(?:[0-9]|[1-9][0-9]|[1-4][0-9]{2}|500)$"

if __name__ == "__main__":
    con = ControlSystem(
        vertical_thrusters=[5, 6, 7, 8],
        horizontal_thrusters=[1, 2, 3, 4]
    )
    hd, vd = 100, 100
    
    try:
        con.enable_rc_control()
        while True:
            con.move_forward(hd, vd)
            pwm = input("Enter pwm value for hd and vd (e.g, hd_100 vd_50): ").split(" ")
            if pwm:
                if re.match(hd_pattern, pwm[0]):
                    hd = int(pwm[0].split("_")[1])
                if len(pwm) == 2 and re.match(vd_pattern, pwm[1]):
                    vd = int(pwm[1].split("_")[1])            
    except KeyboardInterrupt:
        print("\nKeyboard interrupt - Stopping motors and exiting...")
        con.stop()    
