import time
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from CREATE_motor_control.motor_control.slcan_serial import *
from CREATE_motor_control.motor_control.motor import *

from arm_controller import ArmController

from config import *


def main():

    try:
        controller = ArmController(PORT_FEETECH, PORT_HO8110)
        print("ArmController initialized successfully.\n")
    except Exception as e:
        print(f"Failed to initialize ArmController: {e}")
        return

    motors = controller.ho8110

    if not motors:
        print("No motors registered. Exiting.")
        return

    print("Available commands:")
    print("  enable <id>       → Enable motor")
    print("  disable <id>      → Disable motor")
    print("  goto <id> <pos>   → Move motor to position (degrees)")
    print("  speed <id> <spd>  → Set speed")
    print("  show <id>         → Toggle feedback display")
    print("  exit              → Exit")
    
    while True:
        cmd = input(">>> ").strip().lower()
        if not cmd:
            continue

        if cmd == "exit":
            print("Stopping motors...")
            for m in motors.values():
                m.thread_stop()
            print("Exiting.")
            break

        parts = cmd.split()
        if len(parts) < 2:
            print("⚠️ Invalid command format.")
            continue

        action = parts[0]
        try:
            motor_id = int(parts[1])
            motor = motors.get(motor_id)
            if not motor:
                print(f"No motor with ID {motor_id}")
                continue
        except ValueError:
            print("Motor ID must be an integer.")
            continue

        if action == "enable":
            motor.set_enable(True)
            print(f"Motor {motor_id} enabled.")
        elif action == "disable":
            motor.set_enable(False)
            print(f"Motor {motor_id} disabled.")
        elif action == "goto":
            if len(parts) != 3:
                print("⚠️ Usage: goto <id> <position>")
                continue
            try:
                position = float(parts[2])
                motor.go_to_position(position)
                print(f"Motor {motor_id} sent to {position}°")
            except ValueError:
                print("Invalid position.")
        elif action == "speed":
            if len(parts) != 3:
                print("⚠️ Usage: speed <id> <speed>")
                continue
            try:
                speed = float(parts[2])
                motor.speed_control(speed)
                print(f"Motor {motor_id} speed set to {speed}")
            except ValueError:
                print("Invalid speed.")
        elif action == "show":
            motor.toggle_display()
            print(f"Display toggled for motor {motor_id}")
        else:
            print("Unknown command.")

if __name__ == "__main__":
    main()
        

    time.sleep(0.01) 



    

