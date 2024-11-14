from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from serial.tools import list_ports
import serial

app = Flask(__name__)
sock = SocketIO(app) 

@app.route("/")
def index():
    ports = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
    return render_template("index.html", ports=ports)

@sock.on("connect")
def handle_sock_connect():
    print("client connected")
    send("connected to the server")

@sock.on("connect-arduino")
def handle_arduino_connection(data):
    global ser
    
    port = data.get("port")
    baud_rate = data.get("baudRate")
    
    if port and baud_rate:
        try:
            ser = serial.Serial(port, baud_rate, timeout=10, write_timeout=3)
            print("connected to Arduino")
            emit("connect-arduino", {"status": True})
        except serial.SerialException as e:
            print(f"error connecting to Arduino: {e}")
            emit("connect-arduino", {"status": False, "error": e.strerror})
    else:
        emit("connect-arduino", {"status": False})

@sock.on("send-byte-to-arduino")
def handle_send_byte_to_arduino(data):
    global ser

    if not ser or not ser.is_open:
        ports = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
        print(f"ports: {ports}")
        return emit("connect-arduino", {"status": False, "ports": ports})    
    
    byte = data.get("byte")
    if not byte:
        return emit("send-byte-to-arduino", {"status": False})
    else:
        try:
            ser.write(byte.encode("utf-8"))
            print(f"byte wrote: {byte}")
            return emit("send-byte-to-arduino", {"status": True})
        except serial.SerialTimeoutException as e:
            print(f"write operation timed out. error: {e}")
            ser.close()
            ports = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
            emit("connect-arduino", {"status": False})
            return emit("send-byte-to-arduino", {"status": False, "ports": ports})
        except serial.SerialException as e:
            print(f"unexpected error occurred! error: {e}")
            ser.close()
            ports = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
            emit("connect-arduino", {"status": False})
            return emit("send-byte-to-arduino", {"status": False, "ports": ports})

if __name__ == '__main__':
    global ser
    ser = None
    sock.run(app, debug=True)
