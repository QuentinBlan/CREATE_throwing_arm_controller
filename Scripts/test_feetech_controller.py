import logging
import time
import sys
import os

from config import *

TEST_ID_CHANGE = False
TEST_ZERO_OFFSET = False
TEST_SET_POSITION = False
TEST_SET_SPEED = False

ROT_STEPS = 4096  # Number of steps per full rotation for the Feetech servos


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from feetech_controller import FeetechController

def main() -> bool:
    loop_counter_1 : int = 0
    loop_counter_2 : int = 0

    feetech = FeetechController(port_name=PORT_FEETECH, baudrate=BAUDRATE_FEETECH)

    feetech.connect()
    ft_ids = feetech.ping_servos()
    print(f"Feetech servos found: {ft_ids}")

    if TEST_ID_CHANGE:
        if set(ft_ids) == {1, 2}:
            feetech.change_id(1, 3)
            ft_ids = feetech.ping_servos()
            print(f"Feetech servos found after ID change: {ft_ids}")
            if set(ft_ids) == {2, 3}:
                print("Successfully changed servo IDs to 3 and 4")
                feetech.change_id(3, 1)
                ft_ids = feetech.ping_servos()
                if set(ft_ids) == {1, 2}:
                    print("Successfully changed servo IDs back to 1 and 2")
                else:
                    print("Failed to change servo IDs back to 1 and 2")
            else:
                print("Failed to change servo IDs to 3 and 4")
        else :
            print("Feetech servos 1 and 2 not found, unable to test ")


    if TEST_ZERO_OFFSET:
        pos1 = feetech.read_servo_state(1)
        pos2 = feetech.read_servo_state(2)
        print(f"Feetech positions: {pos1}, {pos2}")
        feetech.set_zero_offset(1, -pos1[0])
        feetech.set_zero_offset(2, -pos2[0])
        time.sleep(0.5)
        pos1 = feetech.read_servo_state(1)
        pos2 = feetech.read_servo_state(2)
        print(f"Feetech positions after zero offset: {pos1}, {pos2}")


    if TEST_SET_POSITION:
            
            try:
                while True:
                    pos = input("Enter target position" )
                    pos = int(pos)
                    feetech.set_position(1, pos, multi_turn_enable=True)
                    feetech.set_position(2, pos, multi_turn_enable=True)
                    loop_counter_1 = pos // ROT_STEPS
                    print(f"Loop counter 1: {loop_counter_1}")

                    timeout = 0
                    while timeout < 20:
                        timeout += 1
                        print_posspeeds(feetech)
                        time.sleep(0.1)
                    
                    
            except KeyboardInterrupt:
                print("\nLoop stopped by user.")


    if TEST_SET_SPEED:
        if feetech.change_mode(1, 1):
            print("Changed servo 1 to speed mode")
            print(f"Current servo 1 mode: {feetech.read_mode(1)}")
            feetech.set_speed(1, 2000)

            print_posspeeds(feetech)
            time.sleep(1)
            print_posspeeds(feetech)
            time.sleep(5)
            print_posspeeds(feetech)
            time.sleep(1)
            print_posspeeds(feetech)
            
            feetech.change_mode(1, 0)
            print("Changed servo 1 back to position mode")
            print(f"Current servo 1 mode: {feetech.read_mode(1)}")
            feetech.set_position(1, 2048)  # Center position
        else:
            print("Failed to change servo 1 to speed mode")

    
    try:
        while True:
            print_posspeeds(feetech)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nLoop stopped by user.")

    feetech.disable_torque([1,2])
    feetech.disconnect()
    print("Feetech controller test completed")
    return True


def print_posspeeds(feetech: FeetechController):
    pos1 = feetech.read_servo_state(1)
    pos2 = feetech.read_servo_state(2)
    print(f"Feetech positions: {pos1}, {pos2}")

if __name__ == "__main__":
    
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)
        
    logging.basicConfig(level=LOG_LEVEL)

    main()