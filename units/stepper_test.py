#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time

import pi_stepper as s

# from stepper_lib import RpiMotorLib
max_speed = 0.002
min_speed = 0.06
GpioPins_MA = [13, 11, 15, 12]
GpioPins_MB = [37, 33, 35, 16]
# GpioPins_MA = [10, 10, 10, 10]
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive
motor_A = s.Stepper(
    "motor_A", "Nema", GpioPins_MA, max_speed, min_speed, "d", "half", True
)
#
# name="BYJMotorX",
# motor_type="28BYJ",
# gpiopins=[0, 0, 0, 0],
# max_speed=0.002,
# min_speed=0.06,
# move_units="d",
# steptype="half",
# simulation=False,


def main():
    """main function loop"""

    # ====== tests for motor L298STepTest ====

    print("Test #1")
    motor_A.set_target(-30)
    motor_A.set_speed(100)
    i = 0
    while motor_A.steps_to_go():
        motor_A.motor_run(verbose=False)
        # motor_B.motor_run(verbose=False)
        print(motor_A.motor_name(), motor_A.steps_to_go(), motor_A.target)

    time.sleep(1)


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    exit()


# =====================END===============================
