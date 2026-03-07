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
    Controlword = 1,
    Statusword,
    OperationMode,
    ActualPositionCounts,
    ActualVelocityCountsPerSec,
    TargetPositionCounts,
    MaxProfileVelocityCountsPerSec,
    MaxTorqueTenthsPercent,
    SoftLimitMinCounts,   // = 9
    SoftLimitMaxCounts,   // = 10
    PidPosKp,             // = 11
    PidVelKp,             // = 12
    PidVelKi,             // = 13
    PidPosFF,             // = 14
    PidCurrentKp,         // = 15
    PidCurrentKi,         // = 16
    PidModelKp,           // = 17
    PidModelKi            // = 18
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
