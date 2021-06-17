import RPi.GPIO as GPIO

GpioPins = [37, 33, 35, 31]


GPIO.output(GpioPins[0], GPIO.LOW)
GPIO.output(GpioPins[1], GPIO.LOW)
GPIO.output(GpioPins[2], GPIO.LOW)
GPIO.output(GpioPins[3], GPIO.LOW)
