"""Feetech STS/SMS/SCS control table definitions.

Register map adapted from HuggingFace lerobot (Apache-2.0):
https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
Official docs: http://doc.feetech.cn/
"""

# data_name: (address, byte_size)
STS_SMS_CONTROL_TABLE = {
    # EPROM (persistent, requires Lock=0 to write)
    "Firmware_Major_Version": (0, 1),  # read-only
    "Firmware_Minor_Version": (1, 1),  # read-only
    "Model_Number": (3, 2),  # read-only
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),  # 0-1000 = 0-100% of stall torque, persistent
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),  # units of 6.5mA
    "Angular_Resolution": (30, 1),
    "Homing_Offset": (31, 2),  # sign bit 11
    "Operating_Mode": (33, 1),  # 0: position, 1: velocity, 2: PWM, 3: step
    "Protective_Torque": (34, 1),  # torque % after overload protection kicks in
    "Protection_Time": (35, 1),  # units of 10ms
    "Overload_Torque": (36, 1),  # load % that starts the overload timer
    "Velocity_P_Coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),  # units of 10ms
    "Velocity_I_Coefficient": (39, 1),
    # SRAM (volatile, reset on power cycle)
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),  # units of 8.7 deg/s^2
    "Goal_Position": (42, 2),  # sign bit 15
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),  # sign bit 15
    "Torque_Limit": (48, 2),  # 0-1000, runtime cap, initialized from Max_Torque_Limit
    "Lock": (55, 1),  # 0: EPROM writable, 1: locked
    "Present_Position": (56, 2),  # read-only, sign bit 15
    "Present_Velocity": (58, 2),  # read-only, sign bit 15
    "Present_Load": (60, 2),  # read-only, sign bit 10, 0-1000 = 0-100%
    "Present_Voltage": (62, 1),  # read-only, units of 0.1V
    "Present_Temperature": (63, 1),  # read-only, deg C
    "Status": (65, 1),  # read-only, error flags
    "Moving": (66, 1),  # read-only
    "Present_Current": (69, 2),  # read-only, units of 6.5mA
}

# SCS series (e.g. SCS0009, protocol 1) has a different layout for some registers.
SCS_CONTROL_TABLE = {
    **{k: v for k, v in STS_SMS_CONTROL_TABLE.items() if v[0] < 26},
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protective_Torque": (37, 1),
    "Protection_Time": (38, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Running_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Lock": (48, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
}

# Sign-magnitude encoded registers: data_name -> sign bit index
STS_SMS_SIGN_BITS = {
    "Homing_Offset": 11,
    "Goal_Position": 15,
    "Goal_Velocity": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
    "Present_Load": 10,
}

# Baud_Rate register value -> baudrate
BAUDRATE_TABLE = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 57_600,
    6: 38_400,
    7: 19_200,
}

MODEL_RESOLUTION = {
    "sts3215": 4096,
    "sts3250": 4096,
    "sm8512bl": 4096,
    "scs0009": 1024,
}

MODEL_NUMBER_TABLE = {
    777: "sts3215",
    2825: "sts3250",
    11272: "sm8512bl",
    1284: "scs0009",
}

# scservo_sdk protocol_end: 0 for STS/SMS, 1 for SCS
MODEL_PROTOCOL = {
    "sts3215": 0,
    "sts3250": 0,
    "sm8512bl": 0,
    "scs0009": 1,
}


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    max_magnitude = (1 << sign_bit) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(f"Magnitude {magnitude} exceeds {max_magnitude} (sign bit {sign_bit})")
    return (1 << sign_bit) | magnitude if value < 0 else magnitude


def decode_sign_magnitude(encoded: int, sign_bit: int) -> int:
    magnitude = encoded & ((1 << sign_bit) - 1)
    return -magnitude if (encoded >> sign_bit) & 1 else magnitude
