# flake8: noqa
# pylint: skip-file
import json
import cv2
import base64
import numpy as np
from shared.image import SharedImage
from PIL import Image
from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from map import Map
from HAL import getPose3d
from typing import Dict, Tuple, Union

# Matrix colors
red: list[int]
orange: list[int]
yellow: list[int]
green: list[int]
blue: list[int]
indigo: list[int]
violet: list[int]

class GUI(MeasuringThreadingGUI):
    def __init__(self, host: str = "ws://127.0.0.1:2303") -> None: ...
    
    def update_gui(self) -> None: ...

    def payloadImage(self) -> Dict[str, Union[str, Tuple[int, int, int]]]: ...

    def process_colors(self, image: np.ndarray) -> np.ndarray: ...

    def showNumpy(self, image: np.ndarray) -> None: ...

    def getMap(self, url: str) -> Union[np.ndarray, None]: ...

    def reset_gui(self) -> None: ...

host: str
gui: GUI

# Redirect the console
def start_console() -> None: ...

# Expose to the user
def showNumpy(image: np.ndarray) -> None: ...

def getMap(url: str) -> Union[np.ndarray, None]: ...
