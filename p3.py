import HAL  # noqa
import GUI  # noqa
import time
import numpy as np
from colorama import Fore, Style
from enum import Enum, auto
import math


# ============= CONSTANTS ================
SPACE_AVAILABLE_Y = 70
SPACE_AVAILABLE_X = 6
DEFAULT_VEL = 2
PARKING_VEL = 1.5

TURNING_VEL = 2.5
INITIAL_YAW = -np.pi / 2
YAW_ERROR = 0.01
YAW_ERROR_TURNING = 0.03


class Laser:

    def parking_space_available(self) -> bool:
        laser: np.ndarray = np.array(HAL.getRightLaserData().values)
        center_idx = int(len(laser) / 2)
        start = int(center_idx - SPACE_AVAILABLE_Y / 2)
        end = int(center_idx + SPACE_AVAILABLE_Y / 2)

        test_values = laser[start:end]

        # ===== Debugging ====
        # spaces = test_values > SPACE_AVAILABLE_X
        # q = 0
        # for s in spaces:
        #     if s:
        #         q += 1
        # print(f"{Fore.YELLOW}{laser[center_idx-1: center_idx+1]}{Fore.RESET} -> Y: {q} / {SPACE_AVAILABLE_Y}")
        # ==================

        return all(test_values > SPACE_AVAILABLE_X)


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
        self.current_state = next_state
        self.start = time.perf_counter()

    def deadline_reached(self):
        deadline = self.deadlines[self.current_state]
        return state.elapsed_time > deadline

    @property
    def elapsed_time(self) -> float:
        return time.perf_counter() - self.start


def displace_angle(initial, displacement):
    result = initial + displacement
    return (result + math.pi) % (2 * math.pi) - math.pi


def shortest_angle_distance_radians(a, b):
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


# ============= MAIN ================

deadlines = {
    StateNames.SEARCHING: None,
    StateNames.FORWARDING: 8,
    StateNames.BRAKING: 1,
    StateNames.TURNING: 9.5,
    StateNames.OPPOSITE_TURNING: 7.5,
    StateNames.STOP_TURNING: 1,
    StateNames.RELOCATE: 1,
}

state = State(StateNames.SEARCHING, deadlines=deadlines)
print("\n" * 30)
print("=" * 30 + " Starting " + "=" * 30)
laser = Laser()

while True:
    yaw = HAL.getPose3d().yaw
    match state.current_state:

        case StateNames.SEARCHING:

            err = yaw - INITIAL_YAW
            if abs(err) > YAW_ERROR:
                HAL.setW(-3 * err)
            else:
                HAL.setW(0)

            if laser.parking_space_available():
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

            target_yaw = displace_angle(INITIAL_YAW, math.radians(55))
            err = shortest_angle_distance_radians(yaw, target_yaw)

            if abs(err) < YAW_ERROR_TURNING:
                state.change_state(StateNames.OPPOSITE_TURNING)
                continue

            HAL.setV(-PARKING_VEL)
            HAL.setW(np.sign(err)*TURNING_VEL)

            print(f"{yaw:.2f} -> {target_yaw:.2f}")

        case StateNames.OPPOSITE_TURNING:
            target_yaw = INITIAL_YAW
            err = shortest_angle_distance_radians(yaw, target_yaw)

            if abs(err) < YAW_ERROR_TURNING:
                state.change_state(StateNames.STOP_TURNING)
                continue

            HAL.setV(-PARKING_VEL)
            HAL.setW(np.sign(err)*TURNING_VEL)

        case StateNames.STOP_TURNING:
            if state.deadline_reached():
                state.change_state(StateNames.RELOCATE)
            HAL.setV(0)
            HAL.setW(0)

        case StateNames.RELOCATE:
            if state.deadline_reached():
                state.change_state(StateNames.FINISH)

            HAL.setV(PARKING_VEL)

        case StateNames.FINISH:
            HAL.setV(0)
            HAL.setW(0)


# Version 004
