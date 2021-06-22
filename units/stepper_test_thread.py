#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time

import py_stepper as s
import threading

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


def thread_motor(thread_name, motor):
    while 1:
        motor.motor_run()


def main():
    """main function loop"""

    # ====== tests for motor L298STepTest ====
    t1 = threading.Thread(name="MA", target=thread_motor, args=("MA", motor_A))
    # Started the threads
    t1.start()
    t2 = threading.Thread(name="MB", target=thread_motor, args=("MB", motor_B))
    # Started the threads
    t2.start()

    print("Test #0: move 90°")
    motor_A.set_target(90)
    motor_A.set_speed(100)
    motor_B.set_target(-90)
    motor_B.set_speed(100)

    time.sleep(5)
    print("Test #1: run two motors simultaneously")
    motor_A.set_target(300)
    motor_A.set_speed(255)
    motor_B.set_target(-300)
    motor_B.set_speed(255)
    i = 0

    time.sleep(5)
    print("Test #2: slow MB")
    motor_B.set_target(300)
    motor_B.set_speed(1)

    print("Test #3: slow MB")

    time.sleep(3)
    motor_A.set_target(0)
    motor_A.set_speed(100)

    print("Test #4: slow MB")

    time.sleep(3)
    motor_A.set_target(-200)
    motor_A.set_speed(100)


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    exit()


# =====================END===============================
