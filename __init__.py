# external/pydobotplus/__init__.py

from .dobotplus import Dobot, auto_connect_dobot
from .message import SomeOtherHelper  # if needed

__all__ = ["Dobot", "auto_connect_dobot"]  # optional, for explicitness