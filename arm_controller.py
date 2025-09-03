
import logging
import time

from feetech_controller import FeetechController
from CREATE_motor_control.motor_control.ho8110_controller import HO8110Controller

logger = logging.getLogger(__name__)

HO8110_TORQUE_CST = 0.4 / 0.01  # [Nm/A]
H08110_RADIUS = 0.01  # [m]

PULLBACK_SPEED = 1000  # Speed for pulling back the throwing arm
FEETECH_MAX_SPEED = 2000  # Maximum speed for Feetech servos
HO_DEFAULT_SPEED = 10  # Default speed for HO8110 motors in position mode

class ThrowingArmController:

    def __init__(self, feetech_port="/dev/ttyUSB0", ho_port="/dev/ttyACM0"):
        self.feetech = FeetechController(port_name=feetech_port, baudrate=115200)
        self.ho = HO8110Controller(active_ids=[1, 2, 3], port=ho_port)
        self.is_running = False
        self.home_position_feetech = {1: 0, 2: 0}
        self.home_position_ho = {1: 0.0, 2: 0.0, 3: 0.0}
        self.loop_counter_1 = 0
        self.loop_counter_2 = 0
        

    def start(self) -> bool:
        """Start both controllers and verify motors."""
        
        # HO8110
        if not self.ho.start_controller():
            self.disconnect_controllers()
            raise Exception("H08110 connection failed : H08110 motors not connected")
        time.sleep(0.25)  # let motors initialize
        if self.ho.are_online() != {1: True, 2: True, 3: True}:
            self.disconnect_controllers()
            raise Exception(f"H08110 connection failed : H08110 motors 1, 2 and 3 not found or not online")
        
        # Feetech
        if not self.feetech.connect():
            self.disconnect_controllers()
            raise Exception("Feetech connection failed : Feetech servos not connected")
        ft_ids = self.feetech.ping_servos()
        if set(ft_ids) != {1, 2}:
            self.disconnect_controllers()
            raise Exception("Feetech connection failed : Feetech servos 1 and 2 not found")

        self.is_running = True
        logger.info("ThrowingArmController started successfully.")

        self.set_default_speed()

        return True
    
    def set_default_speed(self):
        self.feetech.set_speed(1, 1000)
        self.feetech.set_speed(2, 1000)
        

    def stop(self):
        """Stop everything safely."""
        logger.info("Stopping ThrowingArmController...")
        try:
            self.safe_stop()
            pass
        except Exception as e:
            logger.error(f"Error during safe stop: {e}")

        self.disconnect_controllers()

        self.is_running = False
    

    def disconnect_controllers(self):
        """Immediate stop without returning to home."""
        logger.info("Disconnecting controllers...")
        try:
            self.feetech.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting Feetech: {e}")

        try:
            self.ho.stop_controller()
        except Exception as e:
            logger.error(f"Error stopping HO8110 controller: {e}")

        self.is_running = False


    def safe_stop(self):
        """Return to home position as a safety stop."""
        logger.info("Performing safe stop...")

        # Stop HO motors (hold position at home)
        self.ho.batch_position(self.home_position_ho, max_speed=20.0)

        # Return Feetech to home
        self.set_feetech_position(0, 0) 

        time.sleep(2)  # let them settle

        for sid, pos in self.home_position_feetech.items():
            self.feetech.disable_torque(sid)

        logger.info("Robot returned to home position.")


    def set_home(self):

        (pos1, pos2) = self.read_feetech_position()

        feedbacks = self.ho.get_all_feedback()
        
        posh1 = feedbacks.get(1)[0] if feedbacks.get(1) else None
        posh2 = feedbacks.get(2)[0] if feedbacks.get(2) else None
        posh3 = feedbacks.get(3)[0] if feedbacks.get(3) else None

        self.home_position_feetech = {1: pos1, 2: pos2}
        self.home_position_ho = {1: posh1, 2: posh2, 3: posh3}
        logger.info(f"Home position set: Feetech {self.home_position_feetech}, HO8110 {self.home_position_ho}")
    
    def set_feetech_position(self, pos1 = None, pos2 = None):
        """Set Feetech servos to specific positions."""
        if pos1 is not None and pos2 is not None:
            pos1 = pos1 + self.home_position_feetech[1]
            pos2 = pos2 + self.home_position_feetech[2]
            if self.feetech.set_position(1, pos1, multi_turn_enable=True):
                self.loop_counter_1 = pos1 // 4096
            if self.feetech.set_position(2, pos2, multi_turn_enable=True):
                self.loop_counter_2 = pos2 // 4096
            return
        
        if pos1 is not None and pos2 is None:
            pos1 = pos1 + self.home_position_feetech[1]
            if self.feetech.set_position(1, pos1, multi_turn_enable=True):
                self.loop_counter_1 = pos1 // 4096
            return

        if pos2 is not None and pos2 is None:
            pos2 = pos2 + self.home_position_feetech[2]
            if self.feetech.set_position(2, pos2, multi_turn_enable=True):
                self.loop_counter_2 = pos2 // 4096
            return

        
        
    
    def read_feetech_position(self):
        """Read Feetech servos positions.""" 
        pos1 = self.feetech.read_servo_state(1)[0]
        pos2 = self.feetech.read_servo_state(2)[0]

        pos1 = pos1 + self.loop_counter_1 * 4096
        pos2 = pos2 + self.loop_counter_2 * 4096

        return (pos1 - self.home_position_feetech[1], pos2 - self.home_position_feetech[2])
    
    def read_ho_position(self):
        """Read HO8110 motors positions."""
        feedbacks = self.ho.get_all_feedback()
        posh1 = feedbacks.get(1)[0] if feedbacks.get(1) else None
        posh2 = feedbacks.get(2)[0] if feedbacks.get(2) else None
        posh3 = feedbacks.get(3)[0] if feedbacks.get(3) else None

        return (posh1 - self.home_position_ho[1], posh2 - self.home_position_ho[2], posh3 - self.home_position_ho[3])

    def throwing(self,end_position = 0, init_position = 160, direction = 0, release_pos=0.0, throw_speed=30.0, prep_time = 1, unwind_time = 2.0):
  
        logger.info("Starting pulling back...")
        self.feetech.set_speed([1,2],PULLBACK_SPEED)
        self.set_feetech_position(int(init_position * 9.4), int(init_position * 9.4)) 

        input("Press ENTER to start throwing...")

        logger.info("Preparing to throw...")
        self.ho.enable_motor(2)
        self.ho.enable_motor(1)
        self.ho.set_position(2, init_position, max_speed = HO_DEFAULT_SPEED)
        self.ho.set_position(1, direction, max_speed = HO_DEFAULT_SPEED)
        time.sleep(prep_time)

        self.feetech.set_speed([1,2], FEETECH_MAX_SPEED)
        self.set_feetech_position(end_position, end_position)
        time.sleep(unwind_time)

        logger.info("Throwing...")
        #could be speed mode but could be dangerous
        self.ho.set_position(2, release_pos, max_speed=throw_speed)
        self.ho.set_position(1, release_pos, max_speed=throw_speed)

        self.set_default_speed()
        pass
