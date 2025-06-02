from pydobotplus import auto_connect_dobot
import time

dobot = auto_connect_dobot()
dobot.speed(velocity=100, acceleration=100)

# New safer Z and radius
z = 0
radius = 30
center_x, center_y = 220, 0
start_x = center_x
start_y = center_y - radius

# Move to start
dobot.move_to(start_x, start_y, z)

try:
    while True:
        # First half arc (bottom → top, left curve)
        arc1 = dobot.go_arc(center_x, center_y + radius, z, 0, center_x - radius, center_y, z, 0)
        dobot.wait_for_cmd(arc1)

        # Second half arc (top → bottom, right curve)
        arc2 = dobot.go_arc(center_x, center_y - radius, z, 0, center_x + radius, center_y, z, 0)
        dobot.wait_for_cmd(arc2)

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopped.")
    dobot.move_to(start_x, start_y, z)
    dobot.close()