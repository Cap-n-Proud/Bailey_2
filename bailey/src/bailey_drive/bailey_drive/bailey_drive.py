# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

# import board

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int16
from sensor_msgs.msg import Joy, Imu

import time
import argparse
import threading


import math

euler = [0.0, 0.0, 0.0]


class Robot(Node):
    def __init__(
        self,
        simulation=False,
        debug=False,
        steer_gain=0.8,
        speed_limit=0.3,
        left_trim=-0,
        right_trim=0,
        *args,
        **kwargs
    ):
        super().__init__("robot")

        self.steer_gain = steer_gain
        self.speed_limit = speed_limit
        self.left_trim = left_trim
        self.right_trim = right_trim
        self.simulation = simulation
        self.debug = debug
        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)
        self.joy_web = self.create_subscription(String, "joy_web", self.joy_web, 10)
        self.imu_topic = self.create_subscription(Imu, "/imu", self.imu_topic, 10)

        from simple_pid import PID

        self.max_speed = 0.002
        self.GpioPins_MA = [13, 11, 15, 12]
        self.GpioPins_MB = [37, 33, 35, 16]
        # Max spees is [-0.002,0.002], will need to divide teh PID value accorgingly
        self.pid = PID(1, 0, 0, output_limits=(-15, 15), setpoint=41, sample_time=0.08)

        # Initialize motors
        if not self.simulation:
            from .stepper_lib import BYJMotor

            # Declare an named instance of class pass a name and type of motor
            # type of motor(Nema) is case sensitive
            self.motor_A = BYJMotor("motor_A", "Nema", max_speed)
            self.motor_B = BYJMotor("motor_B", "Nema", max_speed)
            self.get_logger().info("Motors initialized successfully")

    def set_steer_gain(self, steer_gain):
        self.steer_gain = steer_gain

    def set_speed_limit(self, speed_limit):
        self.speed_limit = speed_limit

    def set_left_trim(self, left_trim):
        self.left_trim = left_trim

    def set_right_trim(self, right_trim):
        self.right_trim = right_trim

    def move(self, speed, steer):
        speed_l = speed
        speed_r = speed
        if not self.simulation:
            time.sleep(0.1)
            t1 = threading.Thread(
                target=self.motor_A.motor_run,
                args=(self.GpioPins_MA, speed_l / 10000, 1, True, False, "full", 1),
            )
            # motor_A.motor_run(GpioPins_MA, s, 200, True, True, "full", 1)
            t1.start()
            t2 = threading.Thread(
                target=self.motor_B.motor_run,
                args=(self.GpioPins_MB, -speed_r / 10000, 1, True, False, "full", 1),
            )
            t2.start()

        else:
            self.get_logger().info(
                "Simulation mode: "
                + str(round(speed_l / 10000, 7))
                + ","
                + str(round(speed_r, 2))
                + "angle: "
                + str(round(euler[0], 2))
                + ")"
            )

    def balance(self, angle):
        angle_l = speed
        angle_r = speed
        speed = 0.8 * 255
        if not self.simulation:
            time.sleep(0.01)
            t1 = threading.Thread(
                target=self.motor_A.motor_run,
                args=(self.GpioPins_MA, speed, angle, "d", True, False, "full", 1),
            )
            # motor_A.motor_run(GpioPins_MA, s, 200, True, True, "full", 1)
            t1.start()
            t2 = threading.Thread(
                target=self.motor_B.motor_run,
                args=(self.GpioPins_MB, speed, -angle, "d", True, False, "full", 1),
            )
            t2.start()

        else:
            self.get_logger().info(
                "Simulation mode: "
                + str(round(speed_l / 10000, 7))
                + ","
                + str(round(speed_r, 2))
                + "angle: "
                + str(round(euler[0], 2))
                + ")"
            )

    def imu_topic(self, msg):
        global euler
        euler = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            False,
            1,
        )
        print(
            "PID: "
            + str(round(self.pid(euler[0] / 10000), 7))
            + "\t: "
            + str(euler[0])
            + "\t: "
            + str(euler[1])
            + "\t: "
            + str(euler[2])
        )
        self.move(self.pid(euler[0]), 0)

    def joy_topic(self, msg):
        if self.debug:
            self.get_logger().info(
                "joy X: " + str(msg.axes[0]) + " Y: " + str(msg.axes[1])
            )
        self.move(msg.axes[1], -msg.axes[0])

        # These are the buttons to do fine adjustments to the position of the robot
        if msg.axes[7] != 0:
            self.move(msg.axes[7] * self.speed_limit - self.left_trim, 0)

        if msg.axes[6] != 0:
            self.move(0, -(msg.axes[6] * self.speed_limit - self.left_trim))

        if msg.buttons[2] == 1:
            self.move(0, 0)

    def joy_web(self, msg):
        speed = msg.data.split(",")
        self.move(speed[1], speed[0])
        # print(msg.data)


def euler_from_quaternion(x, y, z, w, rad=False, approx=1):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    if not rad:
        roll_x = round(math.degrees(roll_x), approx)
        pitch_y = round(math.degrees(pitch_y), approx)
        yaw_z = round(math.degrees(yaw_z), approx)
    return roll_x, pitch_y, yaw_z  # in radians


def main(args=None):
    rclpy.init(args=args)
    # Construct an argument parser
    parser = argparse.ArgumentParser()

    # Add arguments to the parser
    parser.add_argument(
        "-s",
        "--simulation",
        required=False,
        action="store_true",
        default=False,
        help="Simulation mode: motors are not imported, a message is displayed to simuate the signal",
    )
    parser.add_argument(
        "-d",
        "--debug",
        required=False,
        action="store_true",
        default=False,
        help="Debug mode: robot will print extra log info in the console",
    )

    args = parser.parse_args()
    robot = Robot(simulation=args.simulation, debug=args.debug)
    print("Robot ready")
    if args.debug:
        print("Debug mode")
    if args.simulation:
        print("Simulation mode")

    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
