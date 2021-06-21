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

import threading

# from stepper_lib import RpiMotorLib
max_speed = 0.002
min_speed = 0.006
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive
motor_A = RpiMotorLib.BYJMotor("motor_A", "Nema", max_speed)
motor_B = RpiMotorLib.BYJMotor("motor_B", "Nema", max_speed)

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
    GpioPins_MB = [37, 33, 35, 16]

    # speed = [0.002, 0.003, 0.005]
    # speed_h = [0.05, 0.001, 0.0008, 0.0006]

    t = 2

    motor_range = [0.003, 0.002]
    # Arguments  for motor run function
    # (GPIOPins, stepdelay, steps, counterclockwise, verbose, steptype, initdelay)

    for s in range(0, 255):
        print("Speed: " + str(s) + " T: " + str(len(threading.enumerate())))
        for index, thread in enumerate(threading.enumerate()):
            # print(index, thread.name, thread.is_alive())
            if "MA" in thread.name:
                # thread.stop()
                motor_A.motor_stop()
                print("stopped " + thread.name)
            if "MB" in thread.name:
                # thread.stop()
                motor_B.motor_stop()
                print("stopped " + thread.name)
        t1 = threading.Thread(
            name="MA",
            target=motor_A.motor_run,
            args=(GpioPins_MA, s, 3, True, False, "full"),
        )
        # motor_A.motor_run(GpioPins_MA, s, 200, True, True, "full", 1)
        t1.start()
        t2 = threading.Thread(
            name="MB",
            target=motor_B.motor_run,
            args=(GpioPins_MB, -s, 3, True, False, "full"),
        )
        t2.start()
        time.sleep(0.1)

    # for s in range(0, 6):
    #     print("Speed: " + str(s * 51) + " T: " + str(len(threading.enumerate())))
    #
    #     motor_B.motor_run(GpioPins_MB, -s * 51, 100, True, False, "full", 1)
    # for s in speed:
    #     input("Press <Enter> to continue Test. Speed: " + str(s))
    #     motor_A.motor_run(GpioPins_MA, s, 200, True, True, "wave", 1)
    #     time.sleep(1)
    # for s in speed_h:
    #     input("Press <Enter> to continue Test. Speed: " + str(s))
    #     motor_A.motor_run(GpioPins_MA, s, 100, True, True, "half", 1)
    #     time.sleep(1)


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
