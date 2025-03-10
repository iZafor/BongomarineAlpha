from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

mav = mavlink1.MAVLink()
mav.wait_heartbeat()

