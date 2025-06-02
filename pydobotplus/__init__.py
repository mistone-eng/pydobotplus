from pydobotplus.dobotplus import Dobot, DobotException
from serial.tools import list_ports
import os

def auto_connect_dobot():
    print("[INFO] Scanning available serial ports...")
    ports = list_ports.comports()

    # First try auto-detected ports
    for p in ports:
        print(f"Checking port {p.device} (VID={p.vid}, PID={p.pid}, Name={p.name})")
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            print(f"[INFO] Attempting to connect to DOBOT on port: {p.device}")
            try:
                return Dobot(port=p.device)
            except Exception as e:
                print(f"[WARN] Failed to connect on {p.device}: {e}")

    # Manual fallback if /dev/ttyACM0 exists
    if os.path.exists("/dev/ttyACM0"):
        print("[INFO] Using fallback: /dev/ttyACM0")
        try:
            return Dobot(port="/dev/ttyACM0")
        except Exception as e:
            raise Exception(f"Fallback failed on /dev/ttyACM0: {e}")

    raise Exception("DOBOT not found on any connected serial port.")
