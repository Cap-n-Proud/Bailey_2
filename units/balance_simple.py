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

P = 3
I = 0
D = 0

pid = PID(P, I, D, output_limits=(-255, 255), setpoint=110, sample_time=0.08)


import time

import Easy_Driver as es
import threading


en = 33
step = 29
dir = 31

motor_A = es.ES("motor_A", False, step, dir, en, 0.00001, 0.02)
motor_A.set_speed(200)
motor_A.disable()
for i in range(1, 100):
    motor_A.step()


def sign(x):
    return 1 if x >= 0 else -1


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
        # print(
        #     sign(pid(math.degrees(fusionPose[0]))),
        #     int(abs(pid(math.degrees(fusionPose[0])))),
        # )
        motor_A.set_direction(sign(pid(math.degrees(fusionPose[0]))))
        motor_A.set_speed(int(abs(pid(math.degrees(fusionPose[0])))))
        # motor_A.step()
        # time.sleep(0.1)
