def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def maprange(a, b, s):
    # a = [from_lower, from_upper]
    # b = [to_lower, to_upper]
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))


# speed = [0.002, 0.003, 0.005]


speed = 0.002
lib_range = [0, 255]
motor_range = [0.006, 0.002]
for s in range(0, 10):
    print(s * 25, maprange(lib_range, motor_range, s * 25))
    # if maprange(lib_range, motor_range, s * 25) < motor_range[0]:
    #     print("ERROR")
