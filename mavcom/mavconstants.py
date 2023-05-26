from collections import defaultdict

MODE_MAP = defaultdict(
    lambda: None,
    {
        "FIXED_WING": {
            0: 'MANUAL',
            1: 'CIRCLE',
            2: 'STABILIZE',
            3: 'TRAINING',
            4: 'ACRO',
            5: 'FBWA',
            6: 'FBWB',
            7: 'CRUISE',
            8: 'AUTOTUNE',
            10: 'AUTO',
            11: 'RTL',
            12: 'LOITER',
            13: 'TAKEOFF',
            14: 'AVOID_ADSB',
            15: 'GUIDED',
            16: 'INITIALISING',
            17: 'QSTABILIZE',
            18: 'QHOVER',
            19: 'QLOITER',
            20: 'QLAND',
            21: 'QRTL',
            22: 'QAUTOTUNE',
            23: 'QACRO',
            24: 'THERMAL',
            25: 'LOITERALTQLAND',
        },
        "QUADROTOR": {
            0: 'STABILIZE',
            1: 'ACRO',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            5: 'LOITER',
            6: 'RTL',
            7: 'CIRCLE',
            8: 'POSITION',
            9: 'LAND',
            10: 'OF_LOITER',
            11: 'DRIFT',
            13: 'SPORT',
            14: 'FLIP',
            15: 'AUTOTUNE',
            16: 'POSHOLD',
            17: 'BRAKE',
            18: 'THROW',
            19: 'AVOID_ADSB',
            20: 'GUIDED_NOGPS',
            21: 'SMART_RTL',
            22: 'FLOWHOLD',
            23: 'FOLLOW',
            24: 'ZIGZAG',
            25: 'SYSTEMID',
            26: 'AUTOROTATE',
            27: 'AUTO_RTL',
        }
    }
)

AIRFRAME_TYPES = defaultdict(
    lambda: None,
    {
        1: "FIXED_WING",
        2: "QUADROTOR"
    }
)
