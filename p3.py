"""
Laser Navigation System
==============================

This module provides classes and functions to manage obstacle detection and
auto-parking management. It utilizes data from laser sensors to determine the
availability of parking spaces, as well as the proximity to various obstacles.
"""

import HAL  # noqa
import GUI  # noqa
import time
import numpy as np
from colorama import Fore, Style
from dataclasses import dataclass
from enum import Enum, auto
import math


# ================== CONSTANTS =====================

SPACE_AVAILABLE_Y = 70  # Space for the car in laser rows
SPACE_AVAILABLE_X = 6  # Space for the car in metters

SPACE_AVAILABLE_WIDTH = 6
SPACE_AVAILABLE_DEPTH = 6

DEFAULT_VEL = 2  # Searching velocity
PARKING_VEL = 1.5  # Parking velocity
TURNING_VEL = 6.5  # Angular velocity when parking
INITIAL_YAW_ERROR = 0.1
YAW_ERROR = 0.01  # Yaw umbral when navigating
YAW_ERROR_TURNING = 0.03  # Yaw umbral when parking
MIN_LATERAL_DISTANCE = 1  # Minimum distance (in metters) to obstacles
MIN_BACK_DISTANCE = 1.5  # Minimum distance (in metters) to obstacles
YAW_SEARCHING_P = 0.7
ERROR_CONSIDERED_INF = 10
LASER_N_VALUES = 180
LASER_RADIANS = np.pi


# ================== Util Classes =====================
@dataclass
class Vector:
    """
    This class simplifies the process of working with 2d vectors.
    It will be used instead of 2-element array for improving readability.
    """

    x: float
    y: float


# ================== Detection =====================
class Laser:

    def parking_space_available_v2(self) -> bool:
        laser: np.ndarray = np.array(HAL.getRightLaserData().values)

        vectors: list[Vector] = []
        val: float
        for idx, val in enumerate(laser):
            if abs(val) > ERROR_CONSIDERED_INF:
                continue

            alpha = (idx / LASER_N_VALUES) * LASER_RADIANS
            vectors.append(
                Vector(
                    x=val * np.cos(alpha),
                    y=val * np.sin(alpha),
                )
            )

        lateral_size = SPACE_AVAILABLE_WIDTH / 2

        for vec in vectors:
            if abs(vec.x) < lateral_size and abs(vec.y) < SPACE_AVAILABLE_DEPTH:
                return False
        return True

    def parking_space_available(self) -> bool:
        """Checks if there is an available parking space by analyzing
        laser data from the right side."""
        laser: np.ndarray = np.array(HAL.getRightLaserData().values)
        center_idx = int(len(laser) / 2)
        start = int(center_idx - SPACE_AVAILABLE_Y / 2)
        end = int(center_idx + SPACE_AVAILABLE_Y / 2)

        test_values = laser[start:end]
        return all(test_values > SPACE_AVAILABLE_X)

    def is_close_to_right_obstacle(self) -> bool:
        laser: np.ndarray = np.array(HAL.getRightLaserData().values)
        return min(laser) < MIN_LATERAL_DISTANCE

    def is_close_to_back_obstacle(self) -> bool:
        laser: np.ndarray = np.array(HAL.getBackLaserData().values)
        return min(laser) < MIN_BACK_DISTANCE

    def is_close_to_front_obstacle(self) -> bool:
        laser: np.ndarray = np.array(HAL.getFrontLaserData().values)
        return min(laser) < MIN_LATERAL_DISTANCE


# ================== FSM =====================
class StateNames(Enum):
    LOCATING: int = auto()
    SEARCHING: int = auto()
    BRAKING: int = auto()
    FORWARDING: int = auto()
    TURNING: int = auto()
    OPPOSITE_TURNING: int = auto()
    STOP_TURNING: int = auto()
    RELOCATE: int = auto()
    FINISH: int = auto()


class State:
    def __init__(
        self, initial_state: StateNames, deadlines: dict[StateNames, float]
    ) -> None:
        self.current_state = initial_state
        self.start = time.perf_counter()
        self.deadlines = deadlines

    def change_state(self, next_state: StateNames) -> None:
        """Changes the current state of the fsm to the specified next state"""
        self.current_state = next_state
        self.start = time.perf_counter()

    def deadline_reached(self):
        """Checks if the time elapsed since the current state was set has
        exceeded its deadline"""
        deadline = self.deadlines[self.current_state]
        return state.elapsed_time > deadline

    @property
    def elapsed_time(self) -> float:
        """Elapsed time since the current state was set."""
        return time.perf_counter() - self.start


# ================== Utils =====================
def displace_angle(initial, displacement):
    """Adjusts an initial angle by a specified displacement, ensuring
    the result remains within the range of -π to π."""
    result = initial + displacement
    return (result + math.pi) % (2 * math.pi) - math.pi


def shortest_angle_distance_radians(a, b):
    """Calculates the shortest angular distance between two angles in radians.
    This function ensures that the distance is minimized by accounting for the
    circular nature of angular measurements."""
    a = a % (2 * math.pi)
    b = b % (2 * math.pi)

    diff = b - a
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi

    return diff


# ============= Debugging ================
def log(text):
    print(f"{Fore.YELLOW}{Style.BRIGHT} -> {text}{Style.RESET_ALL}")


# ============= Initializing ================

deadlines = {
    StateNames.SEARCHING: None,
    StateNames.FORWARDING: 8.5,
    StateNames.BRAKING: 1,
    StateNames.TURNING: None,
    StateNames.OPPOSITE_TURNING: None,
    StateNames.STOP_TURNING: 1,
    StateNames.RELOCATE: 1,
}

state = State(StateNames.SEARCHING, deadlines=deadlines)
print("\n" * 30)
print("=" * 30 + " Starting " + "=" * 30)
laser = Laser()

# ================== MAIN =====================
initial_yaw = None
err = 0

while True:
    yaw = HAL.getPose3d().yaw
    match state.current_state:

        case StateNames.SEARCHING:
            front = HAL.getFrontLaserData().values[0]
            back = HAL.getBackLaserData().values[-1]

            if initial_yaw is None:
                err = (front - back) * YAW_SEARCHING_P
                if (
                    abs(back) > ERROR_CONSIDERED_INF
                    or abs(front) > ERROR_CONSIDERED_INF
                ):
                    err = 0
            else:
                err = yaw - initial_yaw

            if initial_yaw is None and abs(front - back) < INITIAL_YAW_ERROR:
                initial_yaw = yaw
                log(f"Detected yaw orientation of the street: [{initial_yaw:.3f}]")

            if abs(err) > YAW_ERROR:
                HAL.setW(-3 * err)
            else:
                HAL.setW(0)

            if laser.parking_space_available_v2():
                state.change_state(StateNames.FORWARDING)
                log("Free space found. Starting parking...")
                continue

            HAL.setV(DEFAULT_VEL)

        case StateNames.FORWARDING:
            if state.deadline_reached():
                state.change_state(StateNames.BRAKING)
            HAL.setV(PARKING_VEL)

        case StateNames.BRAKING:
            if state.deadline_reached():
                state.change_state(StateNames.TURNING)
            HAL.setV(0)

        case StateNames.TURNING:

            target_yaw = displace_angle(initial_yaw, math.radians(55))
            err = shortest_angle_distance_radians(yaw, target_yaw)

            if laser.is_close_to_right_obstacle() or abs(err) < YAW_ERROR_TURNING:
                state.change_state(StateNames.OPPOSITE_TURNING)
                continue

            HAL.setV(-PARKING_VEL)
            HAL.setW(np.sign(err) * TURNING_VEL)

        case StateNames.OPPOSITE_TURNING:
            target_yaw = initial_yaw
            err = shortest_angle_distance_radians(yaw, target_yaw)

            if laser.is_close_to_back_obstacle() or abs(err) < YAW_ERROR_TURNING:
                state.change_state(StateNames.STOP_TURNING)
                continue

            ang_vel = np.sign(err) * TURNING_VEL

            if laser.is_close_to_right_obstacle():
                ang_vel = 0

            HAL.setV(-PARKING_VEL)
            HAL.setW(ang_vel)

        case StateNames.STOP_TURNING:
            if state.deadline_reached():
                state.change_state(StateNames.RELOCATE)
            HAL.setV(0)
            HAL.setW(0)

        case StateNames.RELOCATE:
            if laser.is_close_to_front_obstacle() or state.deadline_reached():
                state.change_state(StateNames.FINISH)
                log("Parking Completed, exit.")

            HAL.setV(PARKING_VEL)

        case StateNames.FINISH:
            HAL.setV(0)
            HAL.setW(0)


# Version 004
