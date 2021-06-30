#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time

import pi_stepper as s
import threading

# from stepper_lib import RpiMotorLib
max_speed = 0.002
min_speed = 0.06
GpioPins_MA = [13, 11, 15, 12]
GpioPins_MB = [37, 33, 35, 16]
# GpioPins_MA = [10, 10, 10, 10]
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive

motor_A = s.Stepper(
    "motor_A", "Nema", GpioPins_MA, max_speed, min_speed, "s", "full", False
)
motor_B = s.Stepper(
    "motor_B", "Nema", GpioPins_MB, max_speed, min_speed, "d", "full", False
)


def thread_motor(thread_name, motor):
    while 1:
        motor.motor_run()


# import the random module
import random

# determining the values of the parameters
mu = 100
sigma = 50


def main():
    """main function loop"""

    # ====== tests for motor L298STepTest ====
    t1 = threading.Thread(name="MA", target=thread_motor, args=("MA", motor_A))
    # Started the threads
    t1.start()
    t2 = threading.Thread(name="MB", target=thread_motor, args=("MB", motor_B))
    # Started the threads
    t2.start()
    motor_A.set_speed(255)
    motor_B.set_speed(255)

    print("Test #0: move 90°")
    motor_A.set_target(180)
    motor_B.set_target(-180)

    time.sleep(3)
    # motor_A.move_speed_control(255, True)
    # time.sleep(10)

    # while 1:
    #     # using the gauss() method
    #     pos = random.gauss(mu, sigma)
    #     speed = random.choice([10, 50, 100, 150, 200, 250, 255, 245])
    #     sign = random.choice([0, 1])
    #     print(
    #         "---------------"
    #         + str(speed)
    #         + "  "
    #         + str(sign)
    #         + " "
    #         + str(motor_A.current_pos)
    #     )
    #     a = 1
    #     motor_A.move_speed_control(int(speed), sign)
    #     time.sleep(5)


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    exit()


# =====================END===============================
