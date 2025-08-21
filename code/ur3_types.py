#!/usr/bin/env python3
"""
Simple UR3 joint position class
"""


class JointPositions:
    """Simple class for UR3 joint positions with ±360° validation"""

    def __init__(
        self,
        joints=None,
        base=0.0,
        shoulder=0.0,
        elbow=0.0,
        wrist1=0.0,
        wrist2=0.0,
        wrist3=0.0,
    ):
        if joints is not None:
            # Initialize from list: JointPositions([0,0,0,0,0,0])
            if len(joints) != 6:
                raise ValueError("Joint list must have exactly 6 values")
            self._set_joint_values(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5])
        else:
            # Initialize with individual parameters
            self._set_joint_values(base, shoulder, elbow, wrist1, wrist2, wrist3)

    def _validate_angle(self, angle, joint_name):
        """Validate that angle is within ±360 degrees"""
        if not isinstance(angle, (int, float)):
            raise TypeError(f"{joint_name} must be a number, got {type(angle).__name__}")
        if abs(angle) > 360:
            raise ValueError(f"{joint_name} angle {angle}° exceeds ±360° limit")
        return float(angle)

    def _set_joint_values(self, base, shoulder, elbow, wrist1, wrist2, wrist3):
        """Set all joint values with validation"""
        self._base = self._validate_angle(base, "base")
        self._shoulder = self._validate_angle(shoulder, "shoulder")
        self._elbow = self._validate_angle(elbow, "elbow")
        self._wrist1 = self._validate_angle(wrist1, "wrist1")
        self._wrist2 = self._validate_angle(wrist2, "wrist2")
        self._wrist3 = self._validate_angle(wrist3, "wrist3")

    # Property getters and setters for validation
    @property
    def base(self):
        return self._base

    @base.setter
    def base(self, value):
        self._base = self._validate_angle(value, "base")

    @property
    def shoulder(self):
        return self._shoulder

    @shoulder.setter
    def shoulder(self, value):
        self._shoulder = self._validate_angle(value, "shoulder")

    @property
    def elbow(self):
        return self._elbow

    @elbow.setter
    def elbow(self, value):
        self._elbow = self._validate_angle(value, "elbow")

    @property
    def wrist1(self):
        return self._wrist1

    @wrist1.setter
    def wrist1(self, value):
        self._wrist1 = self._validate_angle(value, "wrist1")

    @property
    def wrist2(self):
        return self._wrist2

    @wrist2.setter
    def wrist2(self, value):
        self._wrist2 = self._validate_angle(value, "wrist2")

    @property
    def wrist3(self):
        return self._wrist3

    @wrist3.setter
    def wrist3(self, value):
        self._wrist3 = self._validate_angle(value, "wrist3")

    def to_list(self):
        """Convert to list format for ROS commands"""
        return [
            self.base,
            self.shoulder,
            self.elbow,
            self.wrist1,
            self.wrist2,
            self.wrist3,
        ]

    def __getitem__(self, index):
        """Enable indexed access like joint_pos[0]"""
        joint_list = self.to_list()
        return joint_list[index]

    def __setitem__(self, index, value):
        """Enable indexed assignment like joint_pos[0] = value"""
        joint_names = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        if not 0 <= index < 6:
            raise IndexError("Joint index must be between 0 and 5")
        setattr(self, joint_names[index], value)

    def copy(self):
        """Create a copy of this JointPositions object"""
        return JointPositions(self.to_list())

    def __str__(self):
        return f"[{self.base}, {self.shoulder}, {self.elbow}, {self.wrist1}, {self.wrist2}, {self.wrist3}]"
