#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time
import RPi.GPIO as GPIO

# Next 3 lines for development, local library testing import
# Comment out in production release and change RpiMotorLib.BYJMotor to BYJMotor
# import sys
# sys.path.insert(0, '/home/pi/Documents/tech/RpiMotorLib/RpiMotorLib')
# from RpiMotorLib import BYJMotor

# Production installed library import
# from RpiMotorLib import RpiMotorLib
import stepper_lib as RpiMotorLib

# from stepper_lib import RpiMotorLib

# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive
motor_A = RpiMotorLib.BYJMotor("motor_A", "Nema")
motor_B = RpiMotorLib.BYJMotor("motor_B", "Nema")

"""
# To Test motor stop put push button to VCC on GPIO 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
"""


def main():
    """main function loop"""

    # ====== tests for motor L298STepTest ====

    # To Test motor stop , put push button to VCC on GPIO 17
    # GPIO.add_event_detect(17, GPIO.RISING, callback=button_callback)

    GpioPins_MA = [13, 11, 15, 12]
    GpioPins_MB = [37, 33, 35, 31]

    speed = [0.002, 0.003, 0.005]
    speed_h = [0.05, 0.001, 0.0008, 0.0006]

    t = 2

    # Arguments  for motor run function
    # (GPIOPins, stepdelay, steps, counterclockwise, verbose, steptype, initdelay)

    for s in speed:
        input("Press <Enter> to continue Test. Speed: " + str(s))
        motor_A.motor_run(GpioPins_MA, s, 200, True, True, "full", 1)
        motor_B.motor_run(GpioPins_MB, s, 200, True, True, "full", 1)
        time.sleep(1)
    for s in speed:
        input("Press <Enter> to continue Test. Speed: " + str(s))
        motor_A.motor_run(GpioPins_MA, s, 200, True, True, "wave", 1)
        time.sleep(1)
    for s in speed_h:
        input("Press <Enter> to continue Test. Speed: " + str(s))
        motor_A.motor_run(GpioPins_MA, s, 100, True, True, "half", 1)
        time.sleep(1)


"""
# needed for testing motor stop
def button_callback(channel):
    print("Test file: Stopping motor")
    mymotortest.motor_stop()
"""


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    GPIO.cleanup()
    exit()


# =====================END===============================
