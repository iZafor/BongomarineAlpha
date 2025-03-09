import control_system

if __name__ == "__main__":
    con = control_system.ControlSystem(
        vertical_thrusters=[5, 6, 7, 8],
        horizontal_thrusters=[1, 2, 3, 4],
        hover_height=0,
        max_hover_error=0
    )
    con.manual_control()