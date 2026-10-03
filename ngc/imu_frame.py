"""IMU frame conversions for Switch 2 controllers.

Raw Switch 2 IMU frame, from gravity: +1 g on Z lying flat face up (Z out of
the face), +1 g on Y held upright (Y toward the top edge - a Joy-Con's stick
end, a Pro's shoulder buttons). Emulators want SDL's controller frame (X
right, Y up out of the face, Z toward you) over SDL, and the cemuhook
convention over DSU; these convert between them.
"""

from __future__ import annotations

from . import protocol as P

LEFT, RIGHT = "left", "right"


def upright(report: P.InputReport) -> P.InputReport:
    """IMU of a controller held normally - a Pro, or one half of a Joy-Con
    pair - face up, top edge away from you, in SDL's frame: X right, Y up out
    of the face, Z toward you.

    Up = raw Z and toward-you = -raw Y (the top edge points away); X is
    unchanged. Passing raw axes through put an upward flick on the
    toward-you axis, which Eden read as the opposite throw. Gyro rates rotate
    the same way."""
    ax, ay, az = report.accel
    gx, gy, gz = report.gyro
    return P.InputReport(**{**report.__dict__,
                            "accel": (ax, az, -ay),
                            "gyro": (gx, gz, -gy)})


def sideways(side: str, report: P.InputReport) -> P.InputReport:
    """A lone Joy-Con held sideways (rail away from you, face up) in the same
    emulator frame as upright. The same quarter turn as
    joycon._rotate_sideways: the Right is turned clockwise (its button end points
    right, its right edge toward you), the Left anticlockwise (stick end
    left, rail edge away)."""
    ax, ay, az = report.accel
    gx, gy, gz = report.gyro
    if side == RIGHT:
        accel, gyro = (ay, az, ax), (gy, gz, gx)
    else:
        accel, gyro = (-ay, az, -ax), (-gy, gz, -gx)
    return P.InputReport(**{**report.__dict__, "accel": accel, "gyro": gyro})


def to_dsu(report: P.InputReport) -> P.InputReport:
    """SDL-frame IMU (see upright) -> what DSU/cemuhook clients expect.

    The two conventions differ in sign. Eden (input_common, eden-emulator
    mirror) maps SDL as accel (-x, z, -y), gyro (x, -z, y) and DSU as accel
    (x, -z, y), gyro (pitch, roll, -yaw); for one motion to reach it the same
    way both ways, DSU needs every accel axis negated and gyro Y / Z negated."""
    ax, ay, az = report.accel
    gx, gy, gz = report.gyro
    return P.InputReport(**{**report.__dict__,
                            "accel": (-ax, -ay, -az),
                            "gyro": (gx, -gy, -gz)})
