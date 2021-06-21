#!/usr/bin/env python3
"""A python 3 library for various
 motors and servos to connect to a raspberry pi"""
# ========================= HEADER ===================================
# title             :rpiMotorlib.py
# description       :A python 3 library for various motors
# and servos to connect to a raspberry pi
# This file is for stepper motor tested on
# 28BYJ-48 unipolar stepper motor with ULN2003  = BYJMotor class
# Bipolar Nema stepper motor with L298N = BYJMotor class.
# Bipolar Nema Stepper motor TB6612FNG = BYJMotor class
# Bipolar Nema Stepper motor A4988  Driver = A4988Nema class
# Bipolar Nema Stepper motor DRV8825 Driver = A4988Nema class
# Bipolar Nema Stepper motor LV8729  Driver = A4988Nema class
# Bipolar Nema Stepper motor A3967 Easy Driver = A3967EasyNema class
# Main author       :Gavin Lyons
# Version           :See changelog at url
# url               :https://github.com/gavinlyonsrepo/RpiMotorLib
# mail              :glyons66@hotmail.com
# python_version    :3.5.3

# ========================== IMPORTS ======================
# Import the system modules needed to run rpiMotorlib.py
import sys
import time
import RPi.GPIO as GPIO

# ==================== CLASS SECTION ===============================


class StopMotorInterrupt(Exception):
    """ Stop the motor """

    pass


class BYJMotor(object):
    """class to control a 28BYJ-48 stepper motor with ULN2003 controller
    by a raspberry pi"""

    def __init__(
        self, name="BYJMotorX", motor_type="28BYJ", max_speed=0.002, min_speed=0.06
    ):
        self.name = name
        self.motor_type = motor_type
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.stop_motor = False
        self.motor_range = [min_speed, max_speed]
        self.lib_range = [-255, 255]

    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def map_range(self, a, b, s):
        # a = [from_lower, from_upper]
        # b = [to_lower, to_upper]
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

    def motor_run(
        self,
        gpiopins,
        speed=0.01,
        steps=512,
        ccwise=False,
        verbose=False,
        steptype="half",
        initdelay=0.001,
    ):
        """motor_run,  moves stepper motor based on 7 inputs

         (1) GPIOPins, type=list of ints 4 long, help="list of
         4 GPIO pins to connect to motor controller
         These are the four GPIO pins we will
         use to drive the stepper motor, in the order
         they are plugged into the controller board. So,
         GPIO 18 is plugged into Pin 1 on the stepper motor.
         (2) wait, type=float, default=0.001, help=Time to wait
         (in seconds) between steps.
         (3) steps, type=int, default=512, help=Number of steps sequence's
         to execute. Default is one revolution , 512 (for a 28BYJ-48)
         (4) counterclockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) steptype, type=string , default=half help= type of drive to
         step motor 3 options full step half step or wave drive
         where full = fullstep , half = half step , wave = wave drive.
         (7) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.

        """
        try:
            # self.stop_motor = False
            for pin in gpiopins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, False)
            time.sleep(initdelay)

            # select step based on user input
            # Each step_sequence is a list containing GPIO pins that should be set to High
            if steptype == "half":  # half stepping.
                step_sequence = list(range(0, 8))
                step_sequence[0] = [gpiopins[0]]
                step_sequence[1] = [gpiopins[0], gpiopins[1]]
                step_sequence[2] = [gpiopins[1]]
                step_sequence[3] = [gpiopins[1], gpiopins[2]]
                step_sequence[4] = [gpiopins[2]]
                step_sequence[5] = [gpiopins[2], gpiopins[3]]
                step_sequence[6] = [gpiopins[3]]
                step_sequence[7] = [gpiopins[3], gpiopins[0]]
            elif steptype == "full":  # full stepping.
                step_sequence = list(range(0, 4))
                step_sequence[0] = [gpiopins[0], gpiopins[1]]
                step_sequence[1] = [gpiopins[1], gpiopins[2]]
                step_sequence[2] = [gpiopins[2], gpiopins[3]]
                step_sequence[3] = [gpiopins[0], gpiopins[3]]
            elif steptype == "wave":  # wave driving
                step_sequence = list(range(0, 4))
                step_sequence[0] = [gpiopins[0]]
                step_sequence[1] = [gpiopins[1]]
                step_sequence[2] = [gpiopins[2]]
                step_sequence[3] = [gpiopins[3]]

            else:
                print("Error: unknown step type ; half, full or wave")
                quit()
            if speed < 0 or steps < 0:
                ccwise = True
                speed = abs(speed)
                steps = abs(steps)
                #  To run motor in reverse we flip the sequence order.
                # if ccwise:
                step_sequence.reverse()

            def display_degree():
                """ display the degree value at end of run if verbose"""
                if self.motor_type == "28BYJ":
                    degree = 1.422222
                    print(
                        "Size of turn in degrees = {}".format(round(steps / degree, 2))
                    )
                elif self.motor_type == "Nema":
                    degree = 7.2
                    print(
                        "Size of turn in degrees = {}".format(round(steps * degree, 2))
                    )
                else:
                    # Unknown Motor type
                    print(
                        "Size of turn in degrees = N/A Motor: {}".format(
                            self.motor_type
                        )
                    )

            def print_status(enabled_pins):
                """   Print status of pins."""
                if verbose:
                    print(
                        "Next Step: Step sequence remaining : {} ".format(
                            steps_remaining
                        )
                    )
                    for pin_print in gpiopins:
                        if pin_print in enabled_pins:
                            print("GPIO pin on {}".format(pin_print))
                        else:
                            print("GPIO pin off {}".format(pin_print))

            # Iterate through the pins turning them on and off.
            steps_remaining = steps
            while steps_remaining > 0:
                for pin_list in step_sequence:
                    for pin in gpiopins:
                        if self.stop_motor:
                            print("STOP")
                            self.stop_motor = False
                            return
                            raise StopMotorInterrupt
                        else:
                            if pin in pin_list:
                                GPIO.output(pin, True)
                            else:
                                GPIO.output(pin, False)
                    print_status(pin_list)
                    if speed != 0:
                        time.sleep(
                            self.map_range(self.lib_range, self.motor_range, speed)
                        )
                steps_remaining -= 1

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib: ")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status if everything went well
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.motor_type))
                print("Initial delay = {}".format(initdelay))
                print("GPIO pins = {}".format(gpiopins))
                print("Speed = {}".format(speed))
                print("Number of step sequences = {}".format(steps))
                print("Size of step sequence = {}".format(len(step_sequence)))
                print("Number of steps = {}".format(steps * len(step_sequence)))
                display_degree()
                print("Counter clockwise = {}".format(ccwise))
                print("Verbose  = {}".format(verbose))
                print("Steptype = {}".format(steptype))
        finally:
            # switch off pins at end
            for pin in gpiopins:
                GPIO.output(pin, False)


def degree_calc(steps, steptype):
    """ calculate and returns size of turn in degree
    , passed number of steps and steptype"""
    degree_value = {
        "Full": 1.8,
        "Half": 0.9,
        "1/4": 0.45,
        "1/8": 0.225,
        "1/16": 0.1125,
        "1/32": 0.05625,
        "1/64": 0.028125,
        "1/128": 0.0140625,
    }
    degree_value = steps * degree_value[steptype]
    return degree_value


def steps_calc(degree, steptype):
    degree_value = {
        "Full": 1.8,
        "Half": 0.9,
        "1/4": 0.45,
        "1/8": 0.225,
        "1/16": 0.1125,
        "1/32": 0.05625,
        "1/64": 0.028125,
        "1/128": 0.0140625,
    }
    steps = degree_value / degree_value[steptype]
    return degree_value


def importtest(text):
    """ testing import """
    # print(text)
    text = " "


# ===================== MAIN ===============================


if __name__ == "__main__":
    importtest("main")
else:
    importtest("Imported {}".format(__name__))


# ===================== END ===============================