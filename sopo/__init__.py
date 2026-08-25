"""sopo: controller for our mid-size Feetech-based arm (hopejr-derived)."""

from .bus import FeetechBus
from .safety import (
    MODEL_STALL_TORQUE_KGCM,
    SafetyLimits,
    apply_safety,
    clamp_goal,
    persist_torque_limit,
    read_effort,
    torque_limit_from_kgcm,
    torque_limit_to_kgcm,
)

__all__ = [
    "FeetechBus",
    "SafetyLimits",
    "apply_safety",
    "clamp_goal",
    "persist_torque_limit",
    "read_effort",
    "MODEL_STALL_TORQUE_KGCM",
    "torque_limit_from_kgcm",
    "torque_limit_to_kgcm",
]
