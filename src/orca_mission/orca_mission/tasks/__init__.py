"""執行器登記表：行為樹 YAML 的 type 名稱 → Task 類別。新增執行器時加在這裡。"""
from .base import Status, Task, params_of
from .basic import Cruise, Dive, Hold, TurnTo
from .gate import AlignTo, ApproachGate, CenterGate

TASKS = {cls.__name__: cls for cls in (Hold, Dive, TurnTo, Cruise, AlignTo, CenterGate, ApproachGate)}

__all__ = ['TASKS', 'Status', 'Task', 'params_of']
