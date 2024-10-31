import HAL  # noqa
import GUI  # noqa
import numpy as np
from colorama import Fore
from enum import Enum, auto

# ============= CONSTANTS ================
SPACE_AVAILABLE_Y = 80
SPACE_AVAILABLE_X = 6
SEARCHING_VEL = 2


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


class States(Enum):
    SEARCHING: int = auto()
    LOCATING: int = auto()


# ============= MAIN ================

state = States.SEARCHING
print("\n" * 30)
print("=" * 30 + " Starting " + "=" * 30)
laser = Laser()

while True:
    match state:
        case States.SEARCHING:
            if laser.parking_space_available():
                HAL.setV(0)
                state = States.LOCATING
                continue

            HAL.setV(SEARCHING_VEL)

# Version 003
