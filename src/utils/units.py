import numpy as np


def angle_range(a: float) -> float:
    """Return an angle within the range [-pi, pi]."""
    while a < -np.pi:
        a += 2 * np.pi
    while a > np.pi:
        a -= 2 * np.pi
    return a


def angle_diff(a: float, b: float) -> float:
    """Get the shortest distance between 2 angles."""
    a = angle_range(a)
    b = angle_range(b)
    diff = angle_range(a - b)
    return diff


cm_per_meter = 100
meters_per_cm = 0.01

inches_per_foot = 12
feet_per_inch = 0.083333

inches_per_meter = 39.3701
meters_per_inch = 0.0254

feet_per_meter = 3.28084
meters_per_foot = 0.3048

radians_per_degree = np.pi / 180
degrees_per_radian = 180 / np.pi

pounds_per_kilogram = 2.20462
kilograms_per_pound = 0.453592

grams_per_kilogram = 1000
kilograms_per_gram = 0.001
