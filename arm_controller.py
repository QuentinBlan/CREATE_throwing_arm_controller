
import logging
import time

from feetech_controller import FeetechController
from CREATE_motor_control.motor_control.ho8110_controller import HO8110Controller

logger = logging.getLogger(__name__)

RPM_TO_RAD_PER_SEC = 2 * 3.141592653589793 / 60 # Conversion factor from RPM to rad/s
DEG_TO_FTSTEPS = 4096 / 360 # ~11.4 steps per degree

HO8110_TORQUE_CST = 0.4 / 0.01  # [Nm/A]
H08110_TENDON_RADIUS = 0.01  # [m]
HO_DEFAULT_SPEED_RPM = 100 # Default speed for HO8110 motors in position mode
H0_MAX_SPEED_RPM = 700 # Max speed for HO8110 [rpm] (real max mech speed is 900rpm)

HO_MAX_SPEED = H0_MAX_SPEED_RPM * RPM_TO_RAD_PER_SEC
HO_DEFAULT_SPEED = HO_DEFAULT_SPEED_RPM * RPM_TO_RAD_PER_SEC

# Feetech speeds in steps/s
FT_PULLBACK_SPEED = 1000  
FT_UNWIND_SPEED = 2000


class ThrowingArmController:

    def __init__(self, feetech_port="/dev/ttyUSB0", ho_port="/dev/ttyACM0"):
        #CONTROLLERS
        self.feetech = FeetechController(port_name=feetech_port, baudrate=115200)
        self.ho = HO8110Controller(active_ids=[1, 2, 3], port=ho_port)
        self.is_running = False
        self.home_position_feetech = {1: 0, 2: 0}
        self.home_position_ho = {1: 0.0, 2: 0.0, 3: 0.0}
        self.loop_counter_1 = 0
        self.loop_counter_2 = 0
        

    def start(self) -> bool:
        """Start both controllers and check that all motors are available"""
        
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

        self.reset_speed()
        self.ho.broadcast_enable(enable=False)
        self.feetech.disable_torque([1,2])

        return True
    
    def reset_speed(self):
        self.feetech.set_speed(1, 1000)
        self.feetech.set_speed(2, 1000)
        self.ho.broadcast_speed(HO_DEFAULT_SPEED)
        

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

        self.ho.broadcast_current(0.1)
        self.ho.broadcast_enable()

        time.sleep(2)

        self.ho.broadcast_enable(enable=False)

        time.sleep(1)

        pos1 = self.feetech.read_servo_state(1)[0]
        pos2 = self.feetech.read_servo_state(2)[0]

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

    def throwing(self, current = 0.5, end_position = 0, stop_pos = 150, throwback_pos = 100, ahead_pos = -100, init_position = 3000, direction = 0, release_pos=0, throw_speed_percent = 50, throwing_time = 0.4, unwind_time = 2.0):
        """ 
            throw_speed in percentage of the maximum allowed speed
            direction: not used yet
            end_position: Feetech end position in ticks
            init_position: Feetech initial position in ticks
            stop_pos: HO8110 motor 2 stop position in degrees
            release_pos: HO8110 motor 1 release position in degrees
            prep_time: time to wait after pulling back before throwing
            unwind_time: time to wait after pulling back before throwing
        """
        if throw_speed_percent > 100 or throw_speed_percent < 0:
            raise ValueError("throw_speed must be between 0 and 100")
        
        ho_speed = (throw_speed_percent / 100) * HO_MAX_SPEED
  
        self.ho.broadcast_enable(enable=False)


        logger.info("arming ...")
        #1 setting for pulling back
        self.feetech.set_speed([1,2],FT_PULLBACK_SPEED)
        self.ho.broadcast_current(0.1)

        #2 pulling back
        self.set_feetech_position(init_position, init_position)
        self.ho.broadcast_enable()

        '''timeout = 0
        while timeout < 100:
            timeout += 1
            d = self.feetech.read_servo_state([1,2])
            print(d)
            time.sleep(0.1)'''

        #3 user can place the ball then press enter
        input("Press ENTER to start throw")

        #CREATE A GETTER
        pos = None
        timeout = 0
        while pos is None:
            pos = self.ho.get_feedback(2)[0]
            timeout += 1
            if timeout > 50:
                logger.error("Timeout waiting for HO8110 motor 2 feedback.")
                self.stop()
                return
            
        pos = pos + 10
        print(pos)

        logger.info("locking ...")
        #4 locking in position
        self.ho.set_position(2, pos, max_speed = 90, max_current = 6)

        #5 setting and unwinding the feetech motors
        self.feetech.set_speed(1, FT_PULLBACK_SPEED)
        self.feetech.set_speed(2, FT_PULLBACK_SPEED)
        self.set_feetech_position(end_position, end_position)
        time.sleep(unwind_time)

        #6 HO 1 and 3 throw / 2 follows
        self.ho.batch_position({1 : stop_pos, 2 : ahead_pos, 3 : stop_pos}, max_speed = 90)

        time.sleep(throwing_time)

        '''pos = stop_pos + 100 #we don't talk about that one
        timeout = 0
        while pos >= stop_pos + 5:
            time.sleep(0.01)
            print("wait")
            pos = self.ho.get_feedback(1)[0]
            timeout += 1
            if timeout > 100:
                logger.error("Timeout waiting for end of throw")
                self.stop()
                break
            
        print(pos)'''

        #7 HO 2 flings back
        self.ho.set_position(2, throwback_pos , max_speed = 90, max_current = 6)
        time.sleep(1)

        #8 going back to home position
        self.reset_speed()
        self.ho.batch_position(self.home_position_ho, max_speed = HO_DEFAULT_SPEED)
        self.set_feetech_position(0,0)

        time.sleep(2)

        #9 putting the robot back in 
        self.ho.broadcast_enable(enable=False)
        self.feetech.disable_torque([1,2])
        


    def block(self):
        self.ho.broadcast_enable()
        self.ho.broadcast_position(20, max_speed=10.0)