# 31 dir
# 33 steps

import time
import random
import Easy_Driver as es

import RPi.GPIO as GPIO


en = 19
step = 21
dir = 23
en = 26
step = 24
dir = 22


max_speed = 0.0001
min_speed = 0.02

motor_A = es.ES("motor_A", False, step, dir, en, max_speed, min_speed)

# GPIO.setmode(GPIO.BOARD)
# GPIO.setwarnings(False)
# GPIO.setup(en, GPIO.OUT)
# GPIO.output(en, True)

# for pin in self.gpiopins:
#     GPIO.setup(pin, GPIO.OUT)
#     GPIO.output(pin, False)
# time.sleep(initdelay)
counter = 0
speed = 255
motor_A.set_speed(speed)
# motor_A.disable()
motor_A.set_direction(1)
i = 1
# while i > 10000:
#
#     try:
#         motor_A.step()
#         if counter == 500:
#             counter = 0
#             speed = 0.9 * speed
#             dir = random.choice([-1, 1])
#             motor_A.set_speed(int(speed))
#             motor_A.set_direction(dir)
#             print(str(i) + "----------------" + str(int(speed)) + " " + str(dir))
#
#     except (KeyboardInterrupt, SystemExit):
#         print("\nkeyboardinterrupt caught (again)")
#         print("\n...Program Stopped Manually!")
#         motor_A.disable()
#
#         raise
#     counter += 1
#     i += 1


motor_A.set_speed(speed)
# while i < 1000:
#
#     motor_A.step()
#
#     i += 1

#
# motor_A.disable()
print("all finished")
motor_A.print_info()
motor_A.set_target(90)
i = 0
while i < 1000:

    motor_A.step_to_target()

    # print(round(motor_A.steps_to_go(), 2), motor_A.current_pos)

    i += 1
