from logging import DEBUG, INFO, WARNING, ERROR, CRITICAL

PORT_HO8110 = "/dev/ttyACM0" # Update COM port based on your setup
MOTOR_ID = [1,2,3] # Set the correct CAN ID
PORT_FEETECH = "/dev/ttyUSB0" # Update COM port based on your setup
BAUDRATE_FEETECH = 115200
FEETECH_IDS = [1, 2] # Set the correct IDs for your Feetech servos
LOG_LEVEL = DEBUG  # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
