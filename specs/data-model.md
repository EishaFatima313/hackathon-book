# Data Model

This document defines the core data structures used across modules.

---

# 1. RobotState

Represents the current state of the humanoid robot.

```python
class RobotState:
    id: str
    position: dict  # {x, y, z}
    orientation: dict  # {roll, pitch, yaw}
    battery_level: float
    current_task: str
    status: str