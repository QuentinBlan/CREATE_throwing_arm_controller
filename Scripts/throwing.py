import time
import os
import sys
import logging

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from arm_controller import ThrowingArmController
from config import PORT_FEETECH, PORT_HO8110

def main(): 
    try:
        arm_controller = ThrowingArmController(PORT_FEETECH, PORT_HO8110)
        arm_controller.start()
        print("ArmController initialized successfully.\n")
    except Exception as e:
        print(f"Failed to initialize ArmController: {e}")
        return


    input("Press ENTER to start homing...")
    arm_controller.set_home()
    time.sleep(1) #is necessary ?
    print_status(arm_controller)
    
    try:
        while True:
            y = input("Press ENTER to continue to throw or ctrl c to quit or h to home...")
            if y.lower() == 'h':
                arm_controller.set_home()
                time.sleep(1)
                print_status(arm_controller)
            else:
                #x = int(input("init pos (ho) = ") or 3000)
                x = float(input("time in s") or 0.5)
                #y = int(input("end position (ft)= ") or 1500)
                #z = int(input("end position (ho) = ") or 100)
                #arm_controller.throwing(init_position=x, end_position=y, stop_pos=z)
                arm_controller.throwing(throwing_time=x)
                print_status(arm_controller)
    except KeyboardInterrupt:
        print("Exiting throwing loop.")
    
    print_status(arm_controller)

    try:
        arm_controller.stop()
    except Exception as e:
        print(f"Error during stopping ArmController: {e}")

def print_status(arm_controller : ThrowingArmController):
    (pos1, pos2) = arm_controller.read_feetech_position()
    print(f"Feetech positions: {pos1}, {pos2}")
    (posh1, posh2, posh3) = arm_controller.read_ho_position()
    print(f"HO positions: {posh1}, {posh2}, {posh3}\n")

if __name__ == "__main__":

    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)
        
    logging.basicConfig(level=logging.DEBUG)
    main()

   