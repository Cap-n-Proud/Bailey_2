#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as gpio
import time, sys


class ES(object):
    def __init__(
        self,
        name="Motor_A",
        simulation=False,
        pin_step=0,
        pin_direction=0,
        pin_enable=0,
        max_speed=0.002,
        min_speed=0.06,
        pin_ms1=0,
        pin_ms2=0,
        pin_ms3=0,
        pin_sleep=0,
        pin_reset=0,
        move_units="d",
        step_type="full",
    ):
        self.name = name
        self.pin_step = pin_step
        self.speed_motor_range = [min_speed, max_speed]
        self.speed_lib_range = [0, 255]
        self.current_pos = 0
        self.pin_direction = pin_direction
        self.pin_microstep_1 = pin_ms1
        self.pin_microstep_2 = pin_ms2
        self.pin_microstep_3 = pin_ms3
        self.pin_sleep = pin_sleep
        self.pin_enable = pin_enable
        self.pin_reset = pin_reset
        self.simulation = simulation
        self.step_interval = 0
        self.target = 0
        self.current_pos = 0
        self.step_type = step_type

        gpio.setmode(gpio.BOARD)
        gpio.setwarnings(False)

        if self.pin_step > 0:
            gpio.setup(self.pin_step, gpio.OUT)
        if self.pin_direction > 0:
            gpio.setup(self.pin_direction, gpio.OUT)
            gpio.output(self.pin_direction, True)
        if self.pin_microstep_1 > 0:
            gpio.setup(self.pin_microstep_1, gpio.OUT)
            gpio.output(self.pin_microstep_1, False)
        if self.pin_microstep_2 > 0:
            gpio.setup(self.pin_microstep_2, gpio.OUT)
            gpio.output(self.pin_microstep_2, False)
        if self.pin_microstep_3 > 0:
            gpio.setup(self.pin_microstep_3, gpio.OUT)
            gpio.output(self.pin_microstep_3, False)
        if self.pin_sleep > 0:
            gpio.setup(self.pin_sleep, gpio.OUT)
            gpio.output(self.pin_sleep, True)
        if self.pin_enable > 0:
            gpio.setup(self.pin_enable, gpio.OUT)
            gpio.output(self.pin_enable, True)
            print(self.pin_enable)
        if self.pin_reset > 0:
            gpio.setup(self.pin_reset, gpio.OUT)
            gpio.output(self.pin_reset, True)
        self.disable()

    def steps_to_go(self):
        return self.target - self.current_pos

    def set_target(self, t):
        if self.move_units == "d":
            self.target = steps_calc(t, self.steptype)

        else:
            self.target = t

    def step(self):
        self.enable()
        gpio.output(self.pin_step, True)
        time.sleep(self.step_interval / 2)
        gpio.output(self.pin_step, False)
        time.sleep(self.step_interval / 2)
        self.disable()

    def step_to(self):
        a = 1
        # if if (
        #     abs(steps_remaining) >= self.step_size
        # ):  # self.step_size:  # and self.is_step_due():
        #     # for pin_list in self.step_sequence:
        #     for pin in self.gpiopins:
        #         if self.stop_motor:
        #             raise StopMotorInterrupt
        #         else:
        #             if (
        #                 pin in self.step_sequence[self.current_step_sequence]
        #                 and not self.simulation
        #             ):
        #                 GPIO.output(pin, True)
        #             else:
        #                 GPIO.output(pin, False)
        #     # TODO: Can  change the mapping below to control speed, acceleration, etc
        #     time.sleep(self.stepInterval)
        #     self.current_step_sequence += 1
        #     if self.current_step_sequence > self.step_sequence_len:
        #         self.current_step_sequence = 0
        #     if self.ccwise:
        #         self.current_pos -= self.step_size
        #     else:
        #         self.current_pos += self.step_size
        #     # print_status(pin_list)
        #     self.lastStepTime = time.time()
        # else:
        #     return False

    def set_step_type(self, step_type):
        if self.steptype == "half":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, False)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "half"
        elif self.steptype == "full":
            gpio.output(self.pin_microstep_1, False)
            gpio.output(self.pin_microstep_2, False)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "full"
        elif self.steptype == "1/4":
            gpio.output(self.pin_microstep_1, False)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "1/4"
        elif self.steptype == "1/8":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "1/8"
        elif self.steptype == "1/16":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, True)
            self.step_type = "1/16"

    def motor_name(self):
        return str(self.name)

    def sleep(self):
        gpio.output(self.pin_sleep, False)

    def wake(self):
        gpio.output(self.pin_sleep, True)

    def disable(self):
        gpio.output(self.pin_enable, True)

    def enable(self):
        gpio.output(self.pin_enable, False)

    def reset(self):
        gpio.output(self.pin_reset, False)
        time.sleep(1)
        gpio.output(self.pin_reset, True)

    def map_range(self, a, b, s):
        # a = [from_lower, from_upper]
        # b = [to_lower, to_upper]
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

    def steps_calc(degree, step_type):
        degree_value = {
            "full": 1.8,
            "half": 0.9,
            "1/4": 0.45,
            "1/8": 0.225,
            "1/16": 0.1125,
            "1/32": 0.05625,
            "1/64": 0.028125,
            "1/128": 0.0140625,
        }
        return degree / degree_value[step_type]

    def set_speed(self, s):
        self.speed = s
        self.step_interval = self.map_range(
            self.speed_lib_range, self.speed_motor_range, self.speed
        )

    def set_direction(self, direction):
        if direction == -1:
            gpio.output(self.pin_direction, False)
        else:
            gpio.output(self.pin_direction, True)

    def finish(self):
        gpio.cleanup()
