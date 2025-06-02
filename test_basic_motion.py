from pydobotplus import auto_connect_dobot

# Connect to DOBOT automatically
device = auto_connect_dobot()

# Optional: Set movement speed (tune these values as needed)
device.speed(velocity=100, acceleration=100)

# Move to a test position with queue blocking
print("\n--- Moving to test position ---")
device.move_to(x=200.0, y=0.0, z=50.0, r=0.0, wait=True)

# Return to origin/home with wait
print("\n--- Returning to start position ---")
device.move_to(x=200.0, y=0.0, z=70.0, r=0.0, wait=True)

# Done
device.close()
print("\n--- Motion test complete ---")
