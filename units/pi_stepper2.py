#!/usr/bin/env python3

# https://www.airspayce.com/mikem/arduino/AccelStepper/
"""A python 3 library for various
 motors and servos to connect to a raspberry pi"""
# ========================= HEADER ===================================
# title             :pi_stepper.py
# description       :A python 3 library for stepper
# motors to connect to a raspberry pi
# Main author       :Paolo Negrini
# Version           :See changelog at url
# credit            :https://github.com/gavinlyonsrepo/RpiMotorLib
# credit            :https://www.airspayce.com/mikem/arduino/AccelStepper/
# mail              :
# python_version    :3.5.3

# ========================== IMPORTS ======================
# Import the system modules needed to run pi_stepper.py
import sys
import time
import RPi.GPIO as GPIO

# ==================== CLASS SECTION ===============================


class StopMotorInterrupt(Exception):
    """ Stop the motor """

    pass


class Stepper(object):
    def __init__(
        self,
        name="BYJMotorX",
        motor_type="28BYJ",
        gpiopins=[0, 0, 0, 0],
        max_speed=0.002,
        min_speed=0.06,
        steptype="half",
        move_units="d",
        simulation=False,
    ):
        self.name = name
        self.motor_type = motor_type
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.stop_motor = False
        self.speed_motor_range = [min_speed, max_speed]
        self.speed_lib_range = [0, 255]
        self.target = 0
        self.current_pos = 0
        self.deg_per_step = 7.2
        self.steps_per_rev = int(360 / self.deg_per_step)
        self.step_angle = 0  # Assume the way it is pointing is zero degrees
        self.stepInterval = 0
        self.lastStepTime = 0
        self.move_units = "d"
        self.speed = 0.001
        self.gpiopins = gpiopins
        self.simulation = simulation
        self.m_run_time = 0
        self.steptype = steptype
        self.step_sequence = 0
        self.set_step_sequence()

    def motor_run_loop_time(self):
        return self.m_run_time

    def motor_name(self):
        return str(self.name)

    def steps_to_go(self):
        return self.target - self.current_pos

    def show_pos(self):
        return self.current_pos

    def is_step_due(self):
        if time.time() - self.lastStepTime <= self.stepInterval:
            return False
        else:
            return True

    def set_target(self, t):
        if self.move_units == "d":
            self.target = steps_calc(t, self.steptype)
        else:
            self.target = t

    def set_speed(self, s):
        self.speed = s
        self.stepInterval = self.map_range(
            self.speed_lib_range, self.speed_motor_range, self.speed
        )

    def set_step_type(self, steptype):
        self.step_type = steptype

    def set_move_units(self, mu):
        if mu == "d":
            self.move_units == "d"
        elif mu == "s":
            self.move_units == "s"
        else:
            print("ERROR")

    def set_step_sequence(self):
        # Each step_sequence is a list containing GPIO pins that should be set to High
        if self.steptype == "half":  # half stepping.
            self.step_sequence = list(range(0, 8))
            self.step_sequence[0] = [self.gpiopins[0]]
            self.step_sequence[1] = [self.gpiopins[0], self.gpiopins[1]]
            self.step_sequence[2] = [self.gpiopins[1]]
            self.step_sequence[3] = [self.gpiopins[1], self.gpiopins[2]]
            self.step_sequence[4] = [self.gpiopins[2]]
            self.step_sequence[5] = [self.gpiopins[2], self.gpiopins[3]]
            self.step_sequence[6] = [self.gpiopins[3]]
            self.step_sequence[7] = [self.gpiopins[3], self.gpiopins[0]]
            self.step_size = 0.5
        elif self.steptype == "full":  # full stepping.
            self.step_sequence = list(range(0, 4))
            self.step_sequence[0] = [self.gpiopins[0], self.gpiopins[1]]
            self.step_sequence[1] = [self.gpiopins[1], self.gpiopins[2]]
            self.step_sequence[2] = [self.gpiopins[2], self.gpiopins[3]]
            self.step_sequence[3] = [self.gpiopins[0], self.gpiopins[3]]
            self.step_size = 1
        elif self.steptype == "wave":  # wave driving
            self.step_sequence = list(range(0, 4))
            self.step_sequence[0] = [self.gpiopins[0]]
            self.step_sequence[1] = [self.gpiopins[1]]
            self.step_sequence[2] = [self.gpiopins[2]]
            self.step_sequence[3] = [self.gpiopins[3]]

        else:
            print("Error: unknown step type ; half, full or wave")
            quit()
        self.step_sequence_len = len(self.step_sequence)
        self.current_step_sequence = 0

        print("-----------------------------------------------------")
        print(self.step_sequence)
        print(self.step_sequence[0])
        for pin_list in self.step_sequence[self.current_step_sequence]:
            print(pin_list)

    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def map_range(self, a, b, s):
        # a = [from_lower, from_upper]
        # b = [to_lower, to_upper]
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

    # TODO: see if it si possible to create a threaded motor
    def motor_thread(self):
        print(self)

    def motor_run(self, ccwise=False, verbose=False, initdelay=0.001):
        motor_run_start = time.time()
        # Needs to be called in a loop. It checks if a step is due
        """Runs motor until target position is reached. Need to be called in a loop as it moves one step
        motor_run,  moves stepper motor based on 7 inputs

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
            for pin in self.gpiopins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, False)
            time.sleep(initdelay)

            def display_degree():
                """ display the degree value at end of run if verbose"""
                if self.motor_type == "28BYJ":
                    degree = 1.422222
                    print(
                        "Size of turn in degrees = {}".format(
                            round(self.target / degree, 2)
                        )
                    )
                elif self.motor_type == "Nema":
                    degree = 7.2
                    print(
                        "Size of turn in degrees = {}".format(
                            round(self.target * degree, 2)
                        )
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
                    for pin_print in self.gpiopins:
                        if pin_print in enabled_pins:
                            print("GPIO pin on {}".format(pin_print))
                        else:
                            print("GPIO pin off {}".format(pin_print))

            steps_remaining = self.steps_to_go() / 4
            if steps_remaining < 0:
                ccwise = True
                #  To run motor in reverse we flip the sequence order.
                self.step_sequence.reverse()

            # Iterate through the pins turning them on and off.

            # if there are steps remaining we move
            if (steps_remaining) >= 0:  # and self.is_step_due():
                print(
                    # self.name,
                    # self.current_pos,
                    # steps_remaining,
                    # self.target,
                    self.current_step_sequence,
                    self.step_sequence_len,
                    self.step_sequence[self.current_step_sequence],
                )
                for pin in self.gpiopins:
                    if self.stop_motor:
                        raise StopMotorInterrupt
                    else:
                        if (
                            pin in self.step_sequence[self.current_step_sequence]
                            and not self.simulation
                        ):
                            GPIO.output(pin, True)
                        else:
                            GPIO.output(pin, False)
                        time.sleep(0.0001)
                # print_status(pin_list)
                # TODO:Need to change the mapping below to control speed, acceleration etc
                self.current_step_sequence += 1
                if self.current_step_sequence > self.step_sequence_len - 1:
                    self.current_step_sequence = 0

                if ccwise:
                    self.current_pos -= self.step_size
                else:
                    self.current_pos += self.step_size
                self.lastStepTime = time.time()
            else:
                return False

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib: ")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
            self.stop_motor = False
            return

        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")

        finally:
            # switch off pins at end
            for pin in self.gpiopins:
                GPIO.output(pin, False)

            if verbose and steps_remaining == 1:
                print("\nMotor Run finished, Details:.\n")
                print("Motor name = {}".format(self.name))

                print("Motor type = {}".format(self.motor_type))
                print("Initial delay = {}".format(initdelay))
                print("GPIO pins = {}".format(self.gpiopins))
                print(
                    "Speed: "
                    + str(self.speed)
                    + " ("
                    + str(round(self.stepInterval, 7))
                    + ")"
                )
                print("Number of step sequences = {}".format(self.target))
                print("Size of step sequence = {}".format(len(self.step_sequence)))
                print(
                    "Number of steps = {}".format(self.target * len(self.step_sequence))
                )
                display_degree()
                print("Counter clockwise = {}".format(ccwise))
                print("Verbose  = {}".format(verbose))
                print("Steptype = {}".format(steptype))
        self.m_run_time = time.time() - motor_run_start


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
        "full": 1.8,  #
        "half": 0.9,
        "1/4": 0.45,
        "1/8": 0.225,
        "1/16": 0.1125,
        "1/32": 0.05625,
        "1/64": 0.028125,
        "1/128": 0.0140625,
    }
    steps = degree / degree_value[steptype]
    return steps


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
