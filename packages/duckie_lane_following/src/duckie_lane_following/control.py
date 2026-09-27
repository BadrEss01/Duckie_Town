"""HSV centroid steering; concept adapted from the attributed upstream example."""

import math
import cv2
import numpy as np


class LaneController:
    def __init__(
        self,
        speed=0.15,
        gain=0.8,
        max_turn=1.0,
        lower=(20, 80, 80),
        upper=(40, 255, 255),
        roi_start=0.65,
        min_pixels=20,
    ):
        if not all(math.isfinite(v) for v in (speed, gain, max_turn, roi_start)):
            raise ValueError("Control parameters must be finite")
        if (
            speed < 0
            or gain < 0
            or max_turn <= 0
            or not 0 <= roi_start < 1
            or min_pixels < 1
        ):
            raise ValueError("Invalid control limits")
        if (
            len(lower) != 3
            or len(upper) != 3
            or any(
                not 0 <= lo <= hi <= lim
                for lo, hi, lim in zip(lower, upper, (179, 255, 255))
            )
        ):
            raise ValueError("Invalid OpenCV HSV bounds")
        self.speed, self.gain, self.max_turn = speed, gain, max_turn
        self.lower, self.upper = np.array(lower, np.uint8), np.array(upper, np.uint8)
        self.roi_start, self.min_pixels = roi_start, min_pixels

    def command(self, image):
        if (
            image is None
            or image.dtype != np.uint8
            or image.ndim != 3
            or image.shape[2] != 3
            or not image.size
        ):
            raise ValueError("Expected uint8 BGR image")
        mask = cv2.inRange(
            cv2.cvtColor(image, cv2.COLOR_BGR2HSV), self.lower, self.upper
        )
        mask[: int(image.shape[0] * self.roi_start)] = 0
        m = cv2.moments(mask, binaryImage=True)
        if m["m00"] < self.min_pixels:
            return 0.0, 0.0
        error = (m["m10"] / m["m00"] - image.shape[1] / 2) / (image.shape[1] / 2)
        return self.speed, float(
            np.clip(-self.gain * error, -self.max_turn, self.max_turn)
        )


class CommandState:
    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Timeout must be positive")
        self.timeout = timeout
        self.last_frame = None
        self.command = (0.0, 0.0)

    def update(self, command, now):
        self.command, self.last_frame = command, now

    def current(self, now):
        if self.last_frame is None or now - self.last_frame > self.timeout:
            return 0.0, 0.0
        return self.command
