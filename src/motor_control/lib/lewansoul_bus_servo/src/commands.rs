#[derive(Clone, Copy)]
pub enum ServoWriteCommand {
    MoveTime = 1,
    MoveTimeWait = 7,
    MoveStart = 11,
    MoveStop = 12,
    Id = 13,
    AngleOffset = 17,
    AngleLimit = 20,
    VinLimit = 22,
    MaxTempLimit = 24,
    IsMotor = 29,
    LoadUnload = 31,
    LedCtrl = 33,
    LedError = 35,
}

#[derive(Clone, Copy)]
pub enum ServoReadCommand {
    MoveTime = 2,
    MoveTimeWait = 8,
    Id = 13,
    AngleOffset = 18,
    AngleLimit = 21,
    VinLimit = 23,
    MaxTempLimit = 25,
    Temp = 26,
    Vin = 27,
    Pos = 28,
    IsMotor = 30,
    LoadUnload = 32,
    LedCtrl = 34,
    LedError = 36,
}
