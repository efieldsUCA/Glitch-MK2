"""ROS-free helpers shared by chronos_recorder and chronos_pilot.

Labels follow the Deep-Learning-Chronos convention: steering and throttle
normalized to [-1, 1]. On Glitch (diff drive) steering is angular.z and
throttle is linear.x, each divided by the max speed from gamepad.yaml.
"""
import cv2 as cv
import numpy as np


def clip_unit(value):
    """Clamp value to [-1, 1]."""
    return max(-1.0, min(1.0, value))


def twist_to_labels(linear_x, angular_z, max_linear_vel, max_angular_vel):
    """Convert a cmd_vel command to normalized (steering, throttle)."""
    steering = clip_unit(angular_z / max_angular_vel)
    throttle = clip_unit(linear_x / max_linear_vel)
    return steering, throttle


def labels_to_twist(steering, throttle, max_linear_vel, max_angular_vel):
    """Convert normalized (steering, throttle) back to (linear.x, angular.z)."""
    return (
        clip_unit(throttle) * max_linear_vel,
        clip_unit(steering) * max_angular_vel,
    )


def resize_frame(bgr_frame, width, height):
    """Resize a BGR frame to the network's input size."""
    return cv.resize(bgr_frame, (width, height), interpolation=cv.INTER_AREA)


def frame_to_input(bgr_frame, width, height):
    """Turn a BGR frame into a 1x3xHxW float32 array in [0, 1], RGB order.

    Recorded jpgs are decoded as RGB by torchvision.io.decode_image in
    chronoslearn.py, so the live frame must be RGB too.
    """
    rgb = cv.cvtColor(resize_frame(bgr_frame, width, height), cv.COLOR_BGR2RGB)
    chw = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
    return chw[np.newaxis, ...]
