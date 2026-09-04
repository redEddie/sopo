"""sopo: controller for our mid-size Feetech-based arm (hopejr-derived)."""

from .bus import FeetechBus
from .dynamics import GravityCal, GravityModel
from .joints import (
    ContinuousJoint,
    DualMotorJoint,
    Joint,
    SingleMotorJoint,
    build_joints,
)
from .reflex import Event, Mode, Reflex, ReflexConfig, Trip
from .safety import (
    MODEL_STALL_TORQUE_KGCM,
    SafetyLimits,
    apply_safety,
    clamp_goal,
    freeze,
    persist_torque_limit,
    read_effort,
    torque_limit_from_kgcm,
    torque_limit_to_kgcm,
)

__all__ = [
    "FeetechBus",
    "Joint",
    "SingleMotorJoint",
    "DualMotorJoint",
    "ContinuousJoint",
    "build_joints",
    "SafetyLimits",
    "apply_safety",
    "clamp_goal",
    "freeze",
    "persist_torque_limit",
    "read_effort",
    "MODEL_STALL_TORQUE_KGCM",
    "torque_limit_from_kgcm",
    "torque_limit_to_kgcm",
    "GravityModel",
    "GravityCal",
    "Mode",
    "Event",
    "ReflexConfig",
    "Trip",
    "Reflex",
]
