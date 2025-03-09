from flask import Flask, render_template, request, jsonify
import control_system

app = Flask(__name__)
con = None
thrust_increment = 10

try:
    con = control_system.ControlSystem(
        horizontal_thrusters=[1, 2, 3, 4],
        vertical_thrusters=[5, 6, 7, 8],
        hover_height=0,
        max_hover_error=0,
        kill_switch_gpio = 17
    )
except:
    print("Failed to initialize ControlSystem!")

@app.route("/")
def index():
    control_system_status= con != None
    gpio_status = False
    bar_status = False
    if con:
        gpio_status = con.claim_gpio()
        bar_status = con.init_bar_sensor()
    return render_template("gamepad.html", control_system_status=control_system_status, gpio_status=gpio_status, bar_status=bar_status)

@app.route("/control", methods=["POST"])
def control():
    if request.method == "POST":
        try:
            data = request.get_json()
            direction = data.get("direction")
            if direction:
                print(f"Received direction: {direction}")
                if con:
                    match direction:
                        case "f":
                            for i in range(4):
                                con.horizontal_pwms[i] += thrust_increment
                            print("Moving forward")
                        case "b":
                            for i in range(4):
                                con.horizontal_pwms[i] -= thrust_increment
                            print("Moving backward")
                        case "l":
                            print("Moving left")
                        case "r":
                            print("Moving right")
                        case "u":
                            for i in range(4):
                                con.vertical_pwms[i] += thrust_increment
                            print("Moving up")
                        case "d":
                            for i in range(4):
                                con.vertical_pwms[i] -= thrust_increment
                            print("Moving down")
                        case "s":
                            con.stop()
                            print("Stopping")
                    con.move()
                else:
                    print("ControlSystem not initialized!")
                    return jsonify({"error": "ControlSystem not initialized!"})
        except:
            print("Failed to parse data!")
            return jsonify({"error": "Failed to parse data!"})
    return jsonify({"message": "received"})

if __name__ == "__main__":
    app.run(debug=True)