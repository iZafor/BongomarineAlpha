from flask import Flask, render_template, request, jsonify
import control_system
import threading
import time

app = Flask(__name__)
con = None
thrust_increment = 10

gpio_status = False
bar_status = False
current_direction = None
thread_started = False

try:
    con = control_system.ControlSystem(
        horizontal_thrusters=[1, 2, 3, 4],
        vertical_thrusters=[5, 6, 7, 8],
        hover_height=0,
        max_hover_error=0,
        kill_switch_gpio=24
    )
except Exception as e:
    print("Failed to initialize ControlSystem!", e)

def manage_control():
    global current_direction
    
    if not con:
        return
    
    while True:
        kill_switch_status = con.read_kill_switch_status()
        if not kill_switch_status:
            con.horizontal_pwms = [1500] * 4
            con.vertical_pwms = [1500] * 4
        elif current_direction is not None:
            direction = current_direction
            current_direction = None

            match direction:
                case "f":
                    con.horizontal_pwms[0] += thrust_increment
                    con.horizontal_pwms[1] += thrust_increment
                    con.horizontal_pwms[2] -= thrust_increment
                    con.horizontal_pwms[3] -= thrust_increment
                    print("Moving forward")
                case "b":
                    con.horizontal_pwms[0] -= thrust_increment
                    con.horizontal_pwms[1] -= thrust_increment
                    con.horizontal_pwms[2] += thrust_increment
                    con.horizontal_pwms[3] += thrust_increment
                    print("Moving backward")
                case "l":
                    con.horizontal_pwms[0] -= thrust_increment
                    con.horizontal_pwms[1] += thrust_increment
                    con.horizontal_pwms[2] -= thrust_increment
                    con.horizontal_pwms[3] += thrust_increment
                    print("Moving left")
                case "r":
                    con.horizontal_pwms[0] += thrust_increment
                    con.horizontal_pwms[1] -= thrust_increment
                    con.horizontal_pwms[2] += thrust_increment
                    con.horizontal_pwms[3] -= thrust_increment
                    print("Moving right")
                case "u":
                    con.vertical_pwms[0] += thrust_increment
                    con.vertical_pwms[1] += thrust_increment
                    con.vertical_pwms[2] += thrust_increment
                    con.vertical_pwms[3] += thrust_increment
                    print("Moving up")
                case "d":
                    con.vertical_pwms[0] -= thrust_increment
                    con.vertical_pwms[1] -= thrust_increment
                    con.vertical_pwms[2] -= thrust_increment
                    con.vertical_pwms[3] -= thrust_increment
                    print("Moving down")
                case "s":
                    con.horizontal_pwms = [1500] * 4
                    con.vertical_pwms = [1500] * 4
                    print("Stopping")
        
        con.validate_pwms()
        con.move()
        time.sleep(0.1)

control_thread = threading.Thread(target=manage_control)
control_thread.daemon = True

@app.route("/")
def index():
    global control_thread, thread_started, gpio_status, bar_status
    control_system_status = con is not None
    if con:
        if not thread_started:
            gpio_status = con.claim_gpio()
            bar_status = con.init_bar_sensor()
            control_thread.start()
            thread_started = True
    print("Returning gamepad interface...")
    return render_template("gamepad.html", control_system_status=control_system_status, gpio_status=gpio_status, bar_status=bar_status)

@app.route("/control", methods=["POST"])
def control():
    global current_direction
    if request.method == "POST":
        try:
            data = request.get_json()
            direction = data.get("direction")
            if direction:
                print(f"Received direction: {direction}")
                current_direction = direction
        except Exception as e:
            print("Failed to parse data!", e)
            return jsonify({"error": "Failed to parse data!"})
    return jsonify({"message": "received"})

if __name__ == "__main__":    
    app.run(debug=True, host="0.0.0.0")
    print(con.free_gpio_kill_switch())
