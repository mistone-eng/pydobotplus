import os
import time
import threading
import math
import speech_recognition as sr
from pydobotplus import auto_connect_dobot

# --- Suppress ALSA/JACK warnings ---
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import ctypes
from ctypes.util import find_library
try:
    asound = ctypes.CDLL(find_library("asound"))
    asound.snd_lib_error_set_handler(None)
except Exception:
    pass

# --- Connect to DOBOT ---
device = auto_connect_dobot()
print("[INFO] DOBOT connected successfully.")

device.speed(velocity=100, acceleration=100)
print("[INFO] Speed configured.")

# --- Global state ---
quit_program = False
step_size = 20.0  # in mm

# --- Track position internally ---
current_position = {
    "x": 200.0,
    "y": 0.0,
    "z": 70.0,
    "r": 0.0
}

device.move_to(**current_position, wait=True)

# --- Move arm relative to internal position ---
def move_direction(dx=0, dy=0, dz=0, dr=0):
    current_position["x"] += dx
    current_position["y"] += dy
    current_position["z"] += dz
    current_position["r"] += dr

    device.move_to(**current_position, wait=True)
    print(f"[MOVE] Moved to ({current_position['x']:.1f}, {current_position['y']:.1f}, {current_position['z']:.1f}, {current_position['r']:.1f})")

# --- Voice command listener ---
def listen_for_commands():
    global quit_program

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("[VOICE] Listening for: 'up', 'down', 'left', 'right', 'forward', 'backward', 'home', 'grip', 'ungrip', 'quit'")

        while not quit_program:
            try:
                print(">> Speak now...")
                audio = recognizer.listen(source)
                command = recognizer.recognize_google(audio).lower()
                print(f"[VOICE] You said: {command}")

                if "up" in command:
                    move_direction(dz=step_size)

                elif "down" in command:
                    move_direction(dz=-step_size)

                elif "left" in command:
                    move_direction(dy=-step_size)

                elif "right" in command:
                    move_direction(dy=step_size)

                elif "forward" in command:
                    move_direction(dx=step_size)

                elif "backward" in command:
                    move_direction(dx=-step_size)

                elif "home" in command:
                    print("[ACTION] Returning to home.")
                    device.home()
                    current_position.update({"x": 200.0, "y": 0.0, "z": 70.0, "r": 0.0})

                elif "grip" in command:
                    print("[ACTION] Gripper closing.")
                    device.grip(True)

                elif "release" in command:
                    print("[ACTION] Gripper opening.")
                    device.grip(False)

                elif "quit" in command:
                    print("[ACTION] Shutting down. Returning home.")
                    quit_program = True
                    device.home()
                    break

            except sr.UnknownValueError:
                print("[WARN] Could not understand audio.")
            except sr.RequestError:
                print("[ERROR] Google Speech Recognition unavailable.")
            except Exception as e:
                print(f"[ERROR] {e}")

# --- Start listener thread ---
listener_thread = threading.Thread(target=listen_for_commands)
listener_thread.start()
listener_thread.join()

# --- Final cleanup ---
device.close()
print("[INFO] Program ended. DOBOT safely disconnected.")
