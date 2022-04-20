import math


def normalize_angle_pos(angle):
    return ((angle % 2.0 * math.pi) + 2.0 * math.pi) % 2.0 * math.pi


def normalize_angle(angle):
    a = normalize_angle_pos(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def sqr(val):
    return val * val


def sign(x):
    if x > 0:
        return 1
    else:
        return -1


def toRad(degVal):
    return degVal * (math.pi / 180.0)


def poseDifferenceLargerThan(pose1, pose2, distanceDiffThresh, angleDiffThresh):
    # TODO
    angleDiff = pose1.z() - pose2.z()
    if angleDiff > math.pi:
        angleDiff -= math.pi * 2.0
    elif angleDiff < -math.pi:
        angleDiff += math.pi * 2.0
    if abs(angleDiff) < angleDiffThresh:
        return True
    return False
