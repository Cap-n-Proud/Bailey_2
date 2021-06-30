#!/usr/bin/env python3
""" test example file for rpiMotorlib.py L298 stepper tests"""

import time
import random
import pi_stepper as s

# from stepper_lib import RpiMotorLib
max_speed = 0.001
min_speed = 0.06
GpioPins_MA = [13, 11, 15, 12]
# GpioPins_MA = [10, 10, 10, 10]
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive
motor_A = s.Stepper(
    "motor_A", "Nema", GpioPins_MA, max_speed, min_speed, "d", "half", False
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
    # motor_A.pins_off()

    # ====== tests for motor L298STepTest ====
    print("Test #1")
    motor_A.set_target(180)
    motor_A.set_speed(200)
    time.sleep(2)
    print("Test #1")
    i = 1
    speed = 255
    counter = 0
    # motor_A.move_speed_control(speed, True)
    while i < 3000:
        try:
            motor_A.motor_run(verbose=False)
            if counter == 500:
                counter = 0
                speed = 0.9 * speed
                dir = random.choice(["True", "False"])
                motor_A.move_speed_control(int(speed), dir)
                print("----------------" + str(int(speed)) + " " + str(dir))
                # time.sleep(3)
                # print(
            #     motor_A.motor_name(),
            #     motor_A.steps_to_go(),
            #     motor_A.target,
            #     motor_A.step_size,
            # )
            counter += 1
        except (KeyboardInterrupt, SystemExit):
            print("\nkeyboardinterrupt caught (again)")
            print("\n...Program Stopped Manually!")
            motor_A.motorStop()

            raise

    time.sleep(1)


# ===================MAIN===============================

if __name__ == "__main__":

    print("START")
    main()
    exit()


# =====================END===============================
