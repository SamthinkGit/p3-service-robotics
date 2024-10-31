# flake8: noqa
# pylint: skip-file
import rclpy
import sys
import threading
import time
from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.bumper import BumperNode
from typing import Any, Union

freq: float

motor_node: MotorsNode
odometry_node: OdometryNode
laser_node: LaserNode
bumper_node: BumperNode

executor: rclpy.executors.MultiThreadedExecutor
executor_thread: threading.Thread

def __auto_spin() -> None: ...

### GETTERS ###

def getLaserData() -> Any: ...

def getPose3d() -> Any: ...

def getBumperData() -> Any: ...

### SETTERS ###

def setV(v: Union[int, float]) -> None: ...

def setW(w: Union[int, float]) -> None: ...
