from pydobotplus import auto_connect_dobot
import time

device = auto_connect_dobot()
print("[INFO] Connected to DOBOT. Starting conveyor test on STP2.")

# Set movement speed
speed = 0.5  # Between 0.0 and 1.0
interface = 1  # STP2
direction = 1  # Use -1 to reverse

# Run conveyor for 2 seconds
device.conveyor_belt(speed=speed, direction=direction, interface=interface)
print("[ACTION] Conveyor running...")
time.sleep(2)

# Stop conveyor
device.conveyor_belt(speed=0.0, direction=direction, interface=interface)
print("[ACTION] Conveyor stopped.")
time.sleep(2)

# Run again
device.conveyor_belt(speed=speed, direction=direction, interface=interface)
print("[ACTION] Conveyor running again...")
time.sleep(2)

# Stop and disconnect
device.conveyor_belt(speed=0.0, direction=direction, interface=interface)
device.close()
print("[INFO] Conveyor test complete and device closed.")
