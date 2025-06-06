from .dobotplus import Dobot, DobotException
from serial.tools import list_ports
import sys, os

def auto_connect_dobot():
    print("[INFO] Scanning available serial ports...")
    ports = list_ports.comports()
    fallback_port = "/dev/ttyACM0"
    attempted = set()

    # First try auto-detected ports
    for p in ports:
        # print(f"[INFO] Checking port {p.device} (VID={p.vid}, PID={p.pid}, Name={p.name})")
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            print(f"[INFO] Attempting to connect to DOBOT on port: {p.device}")
            attempted.add(p.device)
            try:
                device = Dobot(port=p.device)
                print(f"[SUCCESS] Connected to DOBOT on port: {p.device}")
                return device
            except DobotException:
                print(f"[WARN] Failed to connect on {p.device}")

    # Manual fallback only if not already tried
    if fallback_port not in attempted and os.path.exists(fallback_port):
        print(f"[INFO] Using fallback: {fallback_port}")
        try:
            device = Dobot(port=fallback_port)
            print(f"[SUCCESS] Connected to DOBOT on port: {fallback_port}")
            return device
        except DobotException:
            print(f"[ERROR] Could not connect to DOBOT on {fallback_port}.")

    print("[ERROR] DOBOT not found on any connected serial port.")
    sys.exit(1)