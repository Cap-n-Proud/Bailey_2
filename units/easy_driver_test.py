# 31 dir
# 33 steps

import time
import random
import Easy_Driver as es

import RPi.GPIO as GPIO

en = 33
step = 29
dir = 31
delay_pwm = 0.00019


motor_A = es.ES("motor_A", False, step, dir, en, 0.0002, 0.02)

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
motor_A.disable()
motor_A.set_direction(1)
i = 1
while i < 10000:

    try:
        motor_A.step()
        if counter == 500:
            counter = 0
            speed = 0.9 * speed
            dir = random.choice([-1, 1])
            motor_A.set_speed(int(speed))
            motor_A.set_direction(dir)
            print(str(i) + "----------------" + str(int(speed)) + " " + str(dir))
            # time.sleep(3)
            # print(
        #     motor_A.motor_name(),
        #     motor_A.steps_to_go(),
        #     motor_A.target,
        #     motor_A.step_size,
        # )
    except (KeyboardInterrupt, SystemExit):
        print("\nkeyboardinterrupt caught (again)")
        print("\n...Program Stopped Manually!")
        motor_A.disable()

        raise
    counter += 1
    i += 1


motor_A.disable()
print("all finished")
# motor_A.finish()
while 1:
    a = 1
