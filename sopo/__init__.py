"""sopo: controller for our mid-size Feetech-based arm (hopejr-derived).

공개 API는 여기서 재수출한다. 실제 구현은 층별 서브패키지에 있다:
hal(버스/레지스터) → motion(관절/제어 루프) → safety(제한/리플렉스) →
model(중력 모델) → runtime(데몬/클ライ언트/CLI). 설정 로더(config)와 키 입력(keys)은 루트 유지.
"""

from .hal.bus import FeetechBus
from .model.dynamics import GravityCal, GravityModel
from .motion.joints import (
    ContinuousJoint,
    DualMotorJoint,
    Joint,
    SingleMotorJoint,
    build_joints,
)
from .safety.reflex import Event, Mode, Reflex, ReflexConfig, Trip
from .safety.limits import (
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
