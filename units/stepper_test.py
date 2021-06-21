#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time

import stepper_lib as s

# from stepper_lib import RpiMotorLib
max_speed = 0.002
min_speed = 0.06
GpioPins_MA = [13, 11, 15, 12]
GpioPins_MB = [37, 33, 35, 16]
# GpioPins_MA = [10, 10, 10, 10]
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive
motor_B = s.Stepper("motor_B", "Nema", GpioPins_MB, max_speed, min_speed, "d", False)
motor_A = s.Stepper("motor_A", "Nema", GpioPins_MA, max_speed, min_speed, "d", False)


def main():
    """main function loop"""

    # ====== tests for motor L298STepTest ====

    print("Test #1: run two motors simultaneously")
    motor_A.set_target(300)
    motor_A.set_speed(255)
    motor_B.set_target(-300)
    motor_B.set_speed(255)
    i = 0
    while 1:
        motor_A.motor_run(verbose=False)
        # motor_B.motor_run(verbose=False)
        i += 1
        if i == 20000:
            motor_A.set_target(500)
            motor_A.set_speed(255)
        print(
            motor_A.motor_name(),
            motor_A.steps_to_go(),
            motor_A.motor_run_loop_time(),
            motor_B.motor_name(),
            motor_B.steps_to_go(),
            motor_B.motor_run_loop_time(),
        )

    time.sleep(1)
    # print("Test #2: change target in the middle of the test")
    # motor_A.set_target(100)
    # motor_A.set_speed(100)
    # motor_B.set_target(-300)
    # motor_B.set_speed(255)
    # i = 0
    # while i < 500:
    #     motor_A.motor_run(verbose=False)
    #     if i == 50:
    #         motor_A.set_target(200)
    #     motor_B.motor_run(verbose=False)
    #     i += 1
    #     # print(
    #     #     motor_A.motor_name(),
    #     #     motor_A.steps_to_go(),
    #     #     motor_B.motor_name(),
    #     #     motor_B.steps_to_go(),
    #     # )
    #
    # time.sleep(1)
    # print("Test #3: change target in the middle of the test")
    # motor_A.set_target(100)
    # motor_A.set_speed(100)
    # motor_B.set_target(-300)
    # motor_B.set_speed(255)
    # i = 0
    # while i < 500:
    #     motor_A.motor_run(verbose=False)
    #     if i == 50:
    #         motor_A.set_speed(1)
    #         motor_A.set_target(200)
    #
    #     motor_B.motor_run(verbose=False)
    #     i += 1
    #     # print(
    #     #     motor_A.motor_name(),
    #     #     motor_A.steps_to_go(),
    #     #     motor_B.motor_name(),
    #     #     motor_B.steps_to_go(),
    #     # )


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    exit()


# =====================END===============================
