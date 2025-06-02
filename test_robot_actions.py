from pydobotplus import auto_connect_dobot

device = auto_connect_dobot()

# Test reset (home + open gripper)
print("\n--- RESET ---")
device.reset()

# Test pick
print("\n--- PICKING OBJECT ---")
device.pickOrPlace(x=200.0, y=0.0, z=-20.0, r=0.0, do_pick=True)

# Test place
print("\n--- PLACING OBJECT ---")
device.pickOrPlace(x=250.0, y=50.0, z=-20.0, r=0.0, do_pick=False)

# Final home
print("\n--- RETURNING HOME ---")
device.home()

# Close connection
device.close()
print("\n--- TEST COMPLETE ---")
