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
        step_type="1/16",
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
        self.move_units = move_units
        self.version = "1.0.0"
        self.direction = 1
        self.step_size = 1 / 8
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
        if self.pin_reset > 0:
            gpio.setup(self.pin_reset, gpio.OUT)
            gpio.output(self.pin_reset, True)
        self.enable()

    def steps_to_go(self):
        return self.target - self.current_pos

    def update_current_pos(self):
        self.current_pos += self.direction

    def current_pos(self):
        return self.current_pos

    def set_target(self, t):
        if self.move_units == "d":
            self.target = self.degrees_to_steps(t, self.step_type)

        else:
            self.target = t

    def step(self):
        self.enable()
        gpio.output(self.pin_step, True)
        time.sleep(self.step_interval / 2)
        gpio.output(self.pin_step, False)
        time.sleep(self.step_interval / 2)
        self.disable()
        self.update_current_pos()

    def set_move_units(self, mu):
        if mu == "d":
            self.move_units == "d"
        elif mu == "s":
            self.move_units == "s"
        else:
            print("ERROR")

    def step_to_target(self):
        steps_remaining = self.steps_to_go()
        if self.steps_to_go() >= 0:
            self.set_direction(self.sign(steps_remaining))
            self.step()
            # self.update_current_pos()
        else:
            return False

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

    def set_step_type(self, step_type):
        if self.step_type == "half":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, False)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "half"
        elif self.step_type == "full":
            gpio.output(self.pin_microstep_1, False)
            gpio.output(self.pin_microstep_2, False)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "full"
        elif self.step_type == "1/4":
            gpio.output(self.pin_microstep_1, False)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "1/4"
        elif self.step_type == "1/8":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, False)
            self.step_type = "1/8"
        elif self.step_type == "1/16":
            gpio.output(self.pin_microstep_1, True)
            gpio.output(self.pin_microstep_2, True)
            gpio.output(self.pin_microstep_3, True)
            self.step_type = "1/16"

    def motor_name(self):
        return str(self.name)

    def current_pos(self):
        return str(self.current_pos)

    def reset_current_pos(self):
        self.current_pos = 0

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

    def print_info(self):

        print("Step size:\t" + str(self.name))
        print("Step size:\t" + str(self.step_size))
        print("Step type:\t" + str(self.step_type))
        print("[min, max] speed:\t" + str(self.speed_motor_range))
        print("Speed:\t" + str(self.speed))

    def map_range(self, a, b, s):
        # a = [from_lower, from_upper]
        # b = [to_lower, to_upper]
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

    def degrees_to_steps(self, degree, step_type):
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

    def sign(self, a):
        return (a > 0) - (a < 0)
