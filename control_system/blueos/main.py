import control_system
import time
import math

if __name__ == "__main__":
    con = control_system.ControlSystem(
        vertical_thrusters=[5, 6, 7, 8],
        horizontal_thrusters=[1, 2, 3, 4],
        hover_height=0,
        max_hover_error=0,
        kill_switch_gpio=4
    )
    try:
        if not con.claim_gpio():
            raise RuntimeError("Failed to claim gpio!")
        if not con.init_bar_sensor():
            raise RuntimeError("Failed to init bar sensor!") 
                   
        con.manual_control()
        # while True:
        #     print(con.read_bar_sensor())
            # print(math.degrees(control_system.util.read_message(con.conn, "ATTITUDE", print_message=False).yaw))
    except Exception as e:
        print(e)
        con.stop()