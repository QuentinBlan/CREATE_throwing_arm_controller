import time
import os
import sys
import logging

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from arm_controller import ThrowingArmController
from config import PORT_FEETECH, PORT_HO8110

def main(): 
    try:
        controller = ThrowingArmController(PORT_FEETECH, PORT_HO8110)
        controller.start()
        print("ArmController initialized successfully.\n")
    except Exception as e:
        print(f"Failed to initialize ArmController: {e}")
        return
    
    time.sleep(2)
    controller.set_home()
    time.sleep(2)

    try:
        controller.stop()
    except Exception as e:
        print(f"Error during stopping ArmController: {e}")

if __name__ == "__main__":

    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)
        
    logging.basicConfig(level=logging.DEBUG)
    main()

   