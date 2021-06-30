import sys, getopt

sys.path.append(".")
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

#  computeHeight() - the conversion uses the formula:
#
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperatur elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = mloecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#
#  Given the constants, this works out to:
#
#  h = 44330.8 * (1 - (p / P0)**0.190263)


def computeHeight(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263))


print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
pressure = RTIMU.RTPressure(s)

print("IMU Name: " + imu.IMUName())
print("Pressure Name: " + pressure.pressureName())

if not imu.IMUInit():
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

if not pressure.pressureInit():
    print("Pressure sensor Init Failed")
else:
    print("Pressure sensor Init Succeeded")

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

from simple_pid import PID

pid = PID(1, 0, 0, output_limits=(-300, 300), setpoint=110, sample_time=0.08)

import time

import pi_stepper as stepper
import threading

# from stepper_lib import RpiMotorLib
max_speed = 0.002
min_speed = 0.06
GpioPins_MA = [13, 11, 15, 12]
GpioPins_MB = [37, 33, 35, 16]
# GpioPins_MA = [10, 10, 10, 10]
# Declare an named instance of class pass a name and type of motor
# type of motor(Nema) is case sensitive

motor_A = stepper.Stepper(
    "motor_A", "Nema", GpioPins_MA, max_speed, min_speed, "d", "full", False
)
motor_B = stepper.Stepper(
    "motor_B", "Nema", GpioPins_MB, max_speed, min_speed, "d", "full", False
)


def thread_motor(thread_name, motor):
    while 1:
        motor.motor_run()


# ====== tests for motor L298STepTest ====
t1 = threading.Thread(name="MA", target=thread_motor, args=("MA", motor_A))
# Started the threads
t1.start()
t2 = threading.Thread(name="MB", target=thread_motor, args=("MB", motor_B))
# Started the threads
t2.start()
motor_A.set_speed(200)
motor_B.set_speed(200)

while True:
    if imu.IMURead():
        # x, y, z = imu.getFusionData()
        # print("%f %f %f" % (x,y,z))
        data = imu.getIMUData()
        (
            data["pressureValid"],
            data["pressure"],
            data["temperatureValid"],
            data["temperature"],
        ) = pressure.pressureRead()
        fusionPose = data["fusionPose"]
        print(
            "r: %f pid: %i"
            % (math.degrees(fusionPose[0]), pid(math.degrees(fusionPose[0])))
        )
        motor_A.set_target(pid(math.degrees(fusionPose[0])))
        time.sleep(0.08)
