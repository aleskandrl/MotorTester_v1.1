#pragma once

#include <cstdint>

namespace motion_core {

enum class ParameterDomain {
    Common = 0,
    Ethercat,
    Mks
};

enum class CommonParameter : std::uint16_t {
    HardwareGearRatio = 1,
    HardwareEncoderResolutionBits,
    HardwareInvertDirection,
    LimitsMaxVelocityDegPerSec,
    LimitsMaxAccelerationDegPerSec2,
    LimitsSoftwareMinDeg,
    LimitsSoftwareMaxDeg,
    HomingMethod,
    HomingSpeedSwitchDegPerSec,
    HomingSpeedZeroDegPerSec,
    HomingOffsetDeg,
    LimitsCurrentLimitPositivePct,  // = 12
    LimitsCurrentLimitNegativePct,  // = 13
    HomingAccelerationDegPerSec2,   // = 14
    MotorSelectionCode              // = 15
};

enum class EthercatParameter : std::uint16_t {
    // CiA402 cyclic (PDO) — managed by process_cycle(), not in SDO dictionary
    Controlword = 1,
    Statusword,
    OperationMode,
    ActualPositionCounts,
    ActualVelocityCountsPerSec,
    TargetPositionCounts,

    // CiA402 SDO — motion profile & limits
    MaxProfileVelocityCountsPerSec = 7,  // 0x607F
    MaxTorqueTenthsPercent,              // = 8
    SoftLimitMinCounts,                  // = 9  (0x607D:01)
    SoftLimitMaxCounts,                  // = 10 (0x607D:02)
    ProfileVelocity,                     // = 11 (0x6081)
    ProfileAcceleration,                 // = 12 (0x6083)
    ProfileDeceleration,                 // = 13 (0x6084)
    QuickStopDecel,                      // = 14 (0x6085)
    PositionArrivalThreshold,            // = 15 (0x6067)
    TorqueLimitForward,                  // = 16 (0x60E0)
    TorqueLimitReverse,                  // = 17 (0x60E1)
    HomingAcceleration,                  // = 18 (0x609A)
    AxialResolution,                     // = 19 (0x6091:02) instruction units/revolution

    // Vendor specific PID (0x2001 "PID Adjust Parameter", DT2001)
    PidPosKp,                            // = 20 (0x2001:01, PA_9)
    PidVelKp,                            // = 21 (0x2001:02, PA_5)
    PidVelKi,                            // = 22 (0x2001:03, PA_6)
    PidPosFF,                            // = 23 (0x2001:04, PA_19 position smooth filter)
    PidTorqueFilter,                     // = 24 (0x2001:05, PA_7)
    PidSpdDetectionFilter,               // = 25 (0x2001:06, PA_8)
    PidAccelTime,                        // = 26 (0x2001:07, PA_40)
    PidDecelTime,                        // = 27 (0x2001:08, PA_41)

    // Vendor specific current control (0x2008 "Step Mode Parameter I", DT2008)
    PidCurrentKp,                        // = 28 (0x2008:04)
    PidCurrentKi,                        // = 29 (0x2008:05)

    // Vendor specific hardware config (0x2000 "Basic Control Parameter", DT2000)
    EncoderSelection,                    // = 30 (0x2000:10, PA_62)
    EncoderResolutionBits,               // = 31 (0x2000:11, PA_95, default 17)

    PidModelKp = 32,                     // kept for ABI — no real SDO mapping for AC servo

    // CiA402 SDO — additional profile/status objects from ESI/PDF
    ModeDisplay,                         // = 33 (0x6061)
    TargetVelocity,                      // = 34 (0x60FF)
    ActualVelocity,                      // = 35 (0x606C)
    TargetTorque,                        // = 36 (0x6071)
    HomeOffset,                          // = 37 (0x607C)
    TorqueSlope,                         // = 38 (0x6087)
    QuickStopOptionCode,                 // = 39 (0x605A)
    FollowingErrorActualValue,           // = 40 (0x60F4)
    DigitalInputs,                       // = 41 (0x60FD)
    PhysicalOutputs,                     // = 42 (0x60FE:01)

    // Probe function objects
    ProbeFunction,                       // = 43 (0x60B8)
    ProbeStatus,                         // = 44 (0x60B9)
    Probe1RisePosition,                  // = 45 (0x60BA)
    Probe1FallPosition,                  // = 46 (0x60BB)
    Probe2RisePosition,                  // = 47 (0x60BC)
    Probe2FallPosition,                  // = 48 (0x60BD)

    // Vendor-specific communication / basic control objects
    CommunicationFixedAddress,           // = 49 (0x2002:03)
    BasicInitialStatusDisplay,           // = 50 (0x2000:01)
    BasicActionOnStop,                   // = 51 (0x2000:02)
    BasicActuatorAction,                 // = 52 (0x2000:03)
    BasicRunningBrakeSpeed,              // = 53 (0x2000:04)
    BasicTorqueControlSpeedLimit,        // = 54 (0x2000:05)
    BasicServoOnDelayMs,                 // = 55 (0x2000:06)
    BasicIoInputActiveLevelWord,         // = 56 (0x2000:07)
    BasicIoOutputActiveLevelWord,        // = 57 (0x2000:08)
    BasicIoDebounceMs,                   // = 58 (0x2000:09)
    BasicMotorPoles,                     // = 59 (0x2000:12)
    BasicPwmDutyCycle,                   // = 60 (0x2000:13)

    // Vendor step-mode objects
    StepLockCurrent,                     // = 61 (0x2008:01)
    StepRunCurrent,                      // = 62 (0x2008:02)
};

enum class MksParameter : std::uint16_t {
    WorkMode = 1,
    WorkingCurrentMilliAmp,
    Subdivision,
    EnPinActiveLevel,
    MotorDirection,
    AutoScreenOff,
    LockedRotorProtection,
    SubdivisionInterpolation,
    CanBitrateIndex,
    CanId,
    SlaveRespondMode,
    SlaveActiveReport,
    GroupId,
    KeyLock,
    HoldingCurrentIndex,
    LimitPortRemap,
    AxisPositionRaw,
    MotorSpeedRpm,
    ProtectionState,
    MotorStatus,
    EnableMotor
};

struct ParameterId {
    ParameterDomain domain{ParameterDomain::Common};
    std::uint32_t value{0};

    [[nodiscard]] bool valid() const noexcept { return value != 0; }
};

constexpr ParameterId make_parameter_id(CommonParameter parameter) {
    return ParameterId{ParameterDomain::Common, static_cast<std::uint32_t>(parameter)};
}

constexpr ParameterId make_parameter_id(EthercatParameter parameter) {
    return ParameterId{ParameterDomain::Ethercat, static_cast<std::uint32_t>(parameter)};
}

constexpr ParameterId make_parameter_id(MksParameter parameter) {
    return ParameterId{ParameterDomain::Mks, static_cast<std::uint32_t>(parameter)};
}

} // namespace motion_core
