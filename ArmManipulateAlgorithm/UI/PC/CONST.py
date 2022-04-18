from enum import IntEnum, auto

class ARM_STATUS(IntEnum):
    ActionExecute = auto()
    ReachTarget = auto()
    AlgorithmDebugDataBufferFull = auto()
    RunParamBufferFull = auto()
    TargetBufferFull = auto()
    PointsBufferFull = auto()
    SettingDataBufferFull = auto()
    STATUSNUMBER = auto()
    ReceiveRunParams = auto()
    ReceiveTarget = auto()
    ReceivePoints = auto()
    SettingData = auto()

class MODE(IntEnum):
    TestAlgorithm = auto()
    NormalRun = auto()
