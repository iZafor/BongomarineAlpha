document.addEventListener("DOMContentLoaded", () => {
    const portOptions = document.getElementById("port-options");
    const baudRateOptions = document.getElementById("baud-rate-options");
    const connectButton = document.getElementById("arduino-connect-button");
    const connectionStatusText = document.querySelector
    (".connection-status");

    const arduinoConnectionPath = "connect-arduino";
    const sendByteToArduinoPath = "send-byte-to-arduino";

    const sock = io();
    
    sock.on("connect", () => {
        console.log("connected to the server");
    });
    
    sock.on(arduinoConnectionPath, (data) => {
        if (data?.status) {
            connectionStatusText.innerHTML = "Connected";
            connectionStatusText.style.color = "#16a34a";
        } else {
            connectionStatusText.innerHTML = "Not Connected!";
            connectionStatusText.style.color = "#dc2626";
        }
    });

    sock.on(sendByteToArduinoPath, (data) => {
        console.log(data);
    });

    connectButton.addEventListener("click", () => {
        const port = portOptions.value;
        const baudRate = baudRateOptions.value;
        
        if (port && baudRate) {
            sock.emit(arduinoConnectionPath, {port, baudRate});
        } else {
            alert("select port and baud rate properly");
        }
    });

    document.querySelectorAll(".control-button")
    .forEach(button => button.addEventListener("click", (event) => {
        const byte = event.target.value;
        sock.emit(sendByteToArduinoPath, { byte });
    }));
});