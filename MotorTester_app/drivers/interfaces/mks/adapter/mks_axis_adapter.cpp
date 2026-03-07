#include "mks/adapter/mks_axis_adapter.h"
#include "mks/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

using motion_core::MksParameter;
using motion_core::ParameterDescriptor;
using motion_core::ParameterDomain;
using motion_core::ParameterValue;

constexpr double kDegreesPerRevolution = 360.0;
constexpr std::uint32_t kDefaultEncoderResolutionBits = 14;

[[nodiscard]] double compute_axis_units_per_degree(const std::uint32_t encoder_bits, const double gear_ratio) {
    const auto bits = std::min<std::uint32_t>(encoder_bits, 31U);
    const double ticks_per_rev = std::ldexp(1.0, static_cast<int>(bits));
    return (ticks_per_rev * gear_ratio) / kDegreesPerRevolution;
}

const ParameterDescriptor kMksParameterDescriptors[] = {
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio),
     "Gear Ratio", "Common/Hardware", "motor_turns_per_output_turn", false, true, true, true,
     ParameterValue::from_floating(0.001), ParameterValue::from_floating(1000.0)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareEncoderResolutionBits),
     "Encoder Resolution (bits)", "Common/Hardware", "bits", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(31)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::HardwareInvertDirection),
     "Invert Direction", "Common/Hardware", "", false, true, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxVelocityDegPerSec),
     "Max Velocity", "Common/Limits", "deg/s", false, true, true, true,
     ParameterValue::from_floating(0.1), ParameterValue::from_floating(18000.0)},
    {motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2),
     "Max Acceleration", "Common/Limits", "%", false, true, true, true,
     ParameterValue::from_floating(0.0), ParameterValue::from_floating(100.0)},
    {motion_core::make_parameter_id(MksParameter::WorkMode),
     "Work Mode", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(5)},
    {motion_core::make_parameter_id(MksParameter::WorkingCurrentMilliAmp),
     "Working Current", "Drive/MKS/Config", "mA", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(5200)},
    {motion_core::make_parameter_id(MksParameter::Subdivision),
     "Subdivision", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(255)},
    {motion_core::make_parameter_id(MksParameter::EnPinActiveLevel),
     "EN Pin Active Level", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(2)},
    {motion_core::make_parameter_id(MksParameter::MotorDirection),
     "Motor Direction", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::AutoScreenOff),
     "Auto Screen Off", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::LockedRotorProtection),
     "Locked Rotor Protection", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::SubdivisionInterpolation),
     "Subdivision Interpolation", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::CanBitrateIndex),
     "CAN Bitrate Index", "Drive/MKS/Config", "", true, false, true, true, // Not persistable in axis config (bootstrap)
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(3)},
    {motion_core::make_parameter_id(MksParameter::CanId),
     "CAN ID", "Drive/MKS/Config", "", true, false, true, true, // Not persistable in axis config (bootstrap)
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(2047)},
    {motion_core::make_parameter_id(MksParameter::SlaveRespondMode),
     "Slave Respond Mode", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::SlaveActiveReport),
     "Slave Active Report", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::GroupId),
     "Group ID", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(1), ParameterValue::from_unsigned(2047)},
    {motion_core::make_parameter_id(MksParameter::KeyLock),
     "Key Lock", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::HoldingCurrentIndex),
     "Holding Current Index", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(8)},
    {motion_core::make_parameter_id(MksParameter::LimitPortRemap),
     "Limit Port Remap", "Drive/MKS/Config", "", false, true, true, true,
     ParameterValue::from_unsigned(0), ParameterValue::from_unsigned(1)},
    {motion_core::make_parameter_id(MksParameter::AxisPositionRaw),
     "Axis Position Raw", "Drive/MKS/Telemetry", "axis", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::MotorSpeedRpm),
     "Motor Speed", "Drive/MKS/Telemetry", "rpm", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::ProtectionState),
     "Protection State", "Drive/MKS/Telemetry", "", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::MotorStatus),
     "Motor Status", "Drive/MKS/Telemetry", "", true, false, false, false,
     ParameterValue{}, ParameterValue{}},
    {motion_core::make_parameter_id(MksParameter::EnableMotor),
     "Enable Motor", "Drive/MKS", "", false, false, true, true, // Runtime-only (don't save to config file)
     ParameterValue::from_bool(false), ParameterValue::from_bool(true)},
};

[[nodiscard]] const ParameterDescriptor* find_mks_descriptor(const motion_core::ParameterId id) {
    for (const auto& descriptor : kMksParameterDescriptors) {
        if (descriptor.id.domain == id.domain && descriptor.id.value == id.value) {
            return &descriptor;
        }
    }
    return nullptr;
}

[[nodiscard]] motion_core::Result<std::uint64_t> require_unsigned_value(const motion_core::ParameterValue& value,
                                                                        const char* field_name) {
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<std::uint64_t>::success(value.unsigned_value);
    if (value.type == motion_core::ParameterValueType::SignedInteger) {
        if (value.signed_value < 0) return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
        return motion_core::Result<std::uint64_t>::success(static_cast<std::uint64_t>(value.signed_value));
    }
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<std::uint64_t>::success(value.bool_value ? 1U : 0U);
    if (value.type == motion_core::ParameterValueType::FloatingPoint) {
        if (value.floating_value < 0.0) return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
        return motion_core::Result<std::uint64_t>::success(static_cast<std::uint64_t>(std::llround(value.floating_value)));
    }
    return motion_core::Result<std::uint64_t>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

[[nodiscard]] motion_core::Result<double> require_floating_value(const motion_core::ParameterValue& value,
                                                                 const char* field_name) {
    if (value.type == motion_core::ParameterValueType::FloatingPoint) return motion_core::Result<double>::success(value.floating_value);
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<double>::success(static_cast<double>(value.unsigned_value));
    if (value.type == motion_core::ParameterValueType::SignedInteger) return motion_core::Result<double>::success(static_cast<double>(value.signed_value));
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<double>::success(value.bool_value ? 1.0 : 0.0);
    return motion_core::Result<double>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

[[nodiscard]] motion_core::Result<bool> require_bool_value(const motion_core::ParameterValue& value,
                                                           const char* field_name) {
    if (value.type == motion_core::ParameterValueType::Boolean) return motion_core::Result<bool>::success(value.bool_value);
    if (value.type == motion_core::ParameterValueType::UnsignedInteger) return motion_core::Result<bool>::success(value.unsigned_value != 0U);
    if (value.type == motion_core::ParameterValueType::SignedInteger) return motion_core::Result<bool>::success(value.signed_value != 0);
    if (value.type == motion_core::ParameterValueType::FloatingPoint) return motion_core::Result<bool>::success(std::abs(value.floating_value) >= 0.5);
    return motion_core::Result<bool>::failure({motion_core::ErrorCode::InvalidArgument, field_name});
}

} // namespace

namespace mks {

MksAxisAdapter::MksAxisAdapter(MksAxisAdapterConfig config)
    : config_(std::move(config)) {
    software_gear_ratio_ = config_.axis_units_per_degree / compute_axis_units_per_degree(kDefaultEncoderResolutionBits, 1.0);
    if (!(software_gear_ratio_ > 0.0) || std::isnan(software_gear_ratio_) || std::isinf(software_gear_ratio_)) {
        software_gear_ratio_ = 1.0;
    }
    software_encoder_resolution_bits_ = kDefaultEncoderResolutionBits;
    recalculate_axis_units_per_degree_from_software_params();

    telemetry_.state = motion_core::AxisState::Disabled;
    telemetry_.mode = mode_;
}

MksAxisAdapter::~MksAxisAdapter() {
    (void)stop();
}

motion_core::AxisInfo MksAxisAdapter::info() const {
    motion_core::AxisInfo info{};
    info.id = config_.axis_id;
    info.name = config_.axis_name;
    info.transport = motion_core::AxisTransportKind::CanBus;
    info.capabilities = {
        motion_core::Capability::ReadTelemetry,
        motion_core::Capability::SetTargetPosition,
        motion_core::Capability::SetTargetVelocity,
        motion_core::Capability::EnableDisable,
        motion_core::Capability::EmergencyStop,
        motion_core::Capability::ReadParameters,
        motion_core::Capability::WriteParameters,
        motion_core::Capability::Homing,
    };
    return info;
}

motion_core::Result<void> MksAxisAdapter::start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (started_) return motion_core::Result<void>::success();
    if (!config_.bus_manager) return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "bus_manager is null"});
    if (std::abs(config_.axis_units_per_degree) < 1e-9) return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "axis_units_per_degree must be non-zero"});

    if (config_.auto_start_bus_manager) {
        const auto bus_start = config_.bus_manager->start();
        if (!bus_start.ok()) return bus_start;
    }

    const auto reg_result = config_.bus_manager->register_axis(config_.can_id);
    if (!reg_result.ok() && reg_result.error().code != motion_core::ErrorCode::AlreadyExists) {
        return reg_result;
    }

    // Ensure CAN response is enabled and motor is in serial (SR_*) work mode for motion commands.
    std::vector<uint8_t> sr_payload = {1, 1}; // respond=1, active=1
    const auto sr_result = config_.bus_manager->execute_raw_command_sync(
        config_.can_id,
        static_cast<uint8_t>(MksCommand::SetSlaveRespondActive),
        sr_payload);
    if (!sr_result.ok()) {
        (void)config_.bus_manager->unregister_axis(config_.can_id);
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "failed to set slave respond/active mode"});
    }

    std::vector<uint8_t> mode_payload = {4}; // work_mode = 4
    const auto mode_result = config_.bus_manager->execute_raw_command_sync(
        config_.can_id,
        static_cast<uint8_t>(MksCommand::SetWorkMode),
        mode_payload);
    if (!mode_result.ok()) {
        (void)config_.bus_manager->unregister_axis(config_.can_id);
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "failed to set work mode"});
    }

    if (const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id); raw_result.ok()) {
        if (raw_result.value().has_axis_position) {
            target_axis_units_ = static_cast<std::int32_t>(raw_result.value().axis_position);
            telemetry_.actual_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
            telemetry_.target_position_deg = telemetry_.actual_position_deg;
        }
    }

    started_ = true;
    telemetry_.state = enabled_ ? motion_core::AxisState::OperationEnabled : motion_core::AxisState::Ready;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!started_) return motion_core::Result<void>::success();

    if (config_.bus_manager) {
        (void)config_.bus_manager->unregister_axis(config_.can_id);
    }

    started_ = false;
    enabled_ = false;
    telemetry_.state = motion_core::AxisState::Disabled;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::set_enabled(const bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    return set_enabled_locked(enabled);
}

motion_core::Result<void> MksAxisAdapter::set_enabled_locked(const bool enabled) {
    const auto status = ensure_started();
    if (!status.ok()) return status;

    std::vector<uint8_t> payload = { static_cast<uint8_t>(enabled ? 1 : 0) };
    const auto result = config_.bus_manager->send_raw_command(config_.can_id, static_cast<uint8_t>(MksCommand::EnableMotor), payload);
    if (!result.ok()) return result;

    if (enabled && !enabled_) {
        // Safety: sync target with actual on enable
        const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id);
        if (raw_result.ok() && raw_result.value().has_axis_position) {
            target_axis_units_ = static_cast<std::int32_t>(raw_result.value().axis_position);
            telemetry_.actual_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
        } else {
             const double raw_actual = telemetry_.actual_position_deg * config_.axis_units_per_degree;
             target_axis_units_ = static_cast<std::int32_t>(std::llround(raw_actual));
        }
        telemetry_.target_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
    }

    enabled_ = enabled;
    telemetry_.state = enabled ? motion_core::AxisState::OperationEnabled : motion_core::AxisState::Ready;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::set_mode(const motion_core::AxisMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto status = ensure_started();
    if (!status.ok()) return status;

    const int work_mode = mode_to_work_mode(mode);
    if (work_mode < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "axis mode not supported by MKS adapter"});
    }

    std::vector<uint8_t> payload = { static_cast<uint8_t>(work_mode) };
    const auto result = config_.bus_manager->send_raw_command(config_.can_id, static_cast<uint8_t>(MksCommand::SetWorkMode), payload);
    if (!result.ok()) return result;

    mode_ = mode;
    telemetry_.mode = mode;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::apply_command(const motion_core::AxisCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto status = ensure_started();
    if (!status.ok()) return status;

    if (command.emergency_stop) {
        const auto result = config_.bus_manager->send_raw_command(config_.can_id, static_cast<uint8_t>(MksCommand::EmergencyStop), {});
        if (!result.ok()) return result;
        enabled_ = false;
        telemetry_.state = motion_core::AxisState::Fault;
        // Reset target to current on emergency stop
        const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id);
        if (raw_result.ok() && raw_result.value().has_axis_position) {
            target_axis_units_ = static_cast<std::int32_t>(raw_result.value().axis_position);
        }
        return motion_core::Result<void>::success();
    }

    if (command.clear_errors) {
        const auto result = config_.bus_manager->send_raw_command(config_.can_id, static_cast<uint8_t>(MksCommand::ReleaseStallProtection), {});
        if (!result.ok()) return result;
        // Safety sync target with actual on error clear
        const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id);
        if (raw_result.ok() && raw_result.value().has_axis_position) {
            target_axis_units_ = static_cast<std::int32_t>(raw_result.value().axis_position);
            telemetry_.target_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
        }
    }

    if (command.set_zero) {
        const auto result = config_.bus_manager->send_raw_command(config_.can_id, 0x92, {});
        if (!result.ok()) return result;
        target_axis_units_ = 0;
        telemetry_.target_position_deg = 0.0;
        telemetry_.actual_position_deg = 0.0;
    }

    if (command.go_home) {
        const auto result = config_.bus_manager->send_raw_command(config_.can_id, 0x91, {});
        if (!result.ok()) return result;
    }

    if (command.has_target_position) {
        const double raw_axis = command.target_position_deg * config_.axis_units_per_degree;
        const auto axis_value = static_cast<std::int32_t>(std::llround(raw_axis));
        const auto clamped_axis = std::clamp(axis_value, -8'388'608, 8'388'607);

        const std::uint16_t speed_rpm = command.has_profile_speed_rpm
            ? static_cast<std::uint16_t>(std::clamp<std::uint32_t>(command.profile_speed_rpm, 0U, 3000U))
            : config_.default_speed;

        std::uint8_t accel_raw = config_.default_accel;
        if (command.has_profile_accel_percent) {
            const double accel_percent = std::clamp(command.profile_accel_percent, 0.0, 100.0);
            accel_raw = static_cast<std::uint8_t>(std::llround((accel_percent / 100.0) * 255.0));
        }

        std::vector<uint8_t> payload;
        MksProtocol::appendBe16(payload, speed_rpm);
        payload.push_back(accel_raw);
        MksProtocol::appendBe24(payload, clamped_axis);

        const uint8_t cmd_code = command.is_relative ? 0xF4 : 0xF5;
        const auto result = config_.bus_manager->send_raw_command(config_.can_id, cmd_code, payload);
        if (!result.ok()) return result;

        if (command.is_relative) {
            target_axis_units_ += clamped_axis;
        } else {
            target_axis_units_ = clamped_axis;
        }
        telemetry_.target_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
    }

    if (command.has_target_velocity) {
        // speed (RPM) = (deg/sec / 6) * gear_ratio
        const double motor_deg_per_sec = command.target_velocity_deg_per_sec * software_gear_ratio_;
        const double rpm = std::abs(motor_deg_per_sec) / 6.0;
        const bool clockwise = motor_deg_per_sec < 0.0; // CW is negative in our Deg/Sec convention, CCW is positive

        const auto speed = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(static_cast<std::uint32_t>(std::llround(rpm)), 0U, 3000U));

        std::vector<uint8_t> payload;
        payload.push_back(clockwise ? 1 : 0);
        MksProtocol::appendBe16(payload, speed);
        payload.push_back(config_.default_accel);

        const auto result = config_.bus_manager->send_raw_command(config_.can_id, static_cast<uint8_t>(MksCommand::RunSpeedMode), payload);
        if (!result.ok()) return result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::AxisTelemetry> MksAxisAdapter::read_telemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::AxisTelemetry>::failure(status.error());

    const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id);
    if (!raw_result.ok()) return motion_core::Result<motion_core::AxisTelemetry>::failure(raw_result.error());

    auto telemetry_snapshot = telemetry_;
    const auto& raw = raw_result.value();
    
    if (raw.has_axis_position) telemetry_snapshot.actual_position_deg = static_cast<double>(raw.axis_position) / config_.axis_units_per_degree;
    if (raw.has_speed_rpm) {
        // deg/sec_out = (RPM * 6) / gear_ratio
        telemetry_snapshot.actual_velocity_deg_per_sec = (static_cast<double>(raw.speed_rpm) * 6.0) / software_gear_ratio_;
    }
    if (raw.has_protection_state) telemetry_snapshot.status_word = raw.protection_state;

    telemetry_snapshot.target_position_deg = static_cast<double>(target_axis_units_) / config_.axis_units_per_degree;
    telemetry_snapshot.mode = mode_;
    
    if (raw.has_protection_state && raw.protection_state != 0) {
        telemetry_snapshot.state = motion_core::AxisState::Fault;
    } else if (raw.has_motor_status) {
        switch (raw.motor_status) {
            case 0: telemetry_snapshot.state = motion_core::AxisState::Fault; break;
            case 1: telemetry_snapshot.state = motion_core::AxisState::Ready; break;
            case 2: case 3: case 4: case 5: case 6:
                telemetry_snapshot.state = motion_core::AxisState::OperationEnabled; break;
            default: telemetry_snapshot.state = motion_core::AxisState::Unknown; break;
        }
    } else {
         telemetry_snapshot.state = enabled_ ? motion_core::AxisState::OperationEnabled : motion_core::AxisState::Ready;
    }

    telemetry_snapshot.timestamp_ns = raw.timestamp_ns;
    return motion_core::Result<motion_core::AxisTelemetry>::success(std::move(telemetry_snapshot));
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> MksAxisAdapter::list_parameters() const {
    std::vector<motion_core::ParameterDescriptor> out;
    out.assign(std::begin(kMksParameterDescriptors), std::end(kMksParameterDescriptors));
    return motion_core::Result<std::vector<motion_core::ParameterDescriptor>>::success(std::move(out));
}

motion_core::Result<motion_core::ParameterSet> MksAxisAdapter::read_parameters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::ParameterSet>::failure(status.error());

    motion_core::ParameterSet set{};
    
    auto add_if_persistable = [&set](const motion_core::ParameterId id, ParameterValue val) {
        const auto* desc = find_mks_descriptor(id);
        if (desc && desc->persistable) {
            set.entries.push_back({id, std::move(val)});
        }
    };

    add_if_persistable(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio), ParameterValue::from_floating(software_gear_ratio_));
    add_if_persistable(motion_core::make_parameter_id(motion_core::CommonParameter::HardwareEncoderResolutionBits), ParameterValue::from_unsigned(software_encoder_resolution_bits_));
    add_if_persistable(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxVelocityDegPerSec), ParameterValue::from_floating(static_cast<double>(config_.default_speed) * 6.0 / software_gear_ratio_));
    add_if_persistable(motion_core::make_parameter_id(motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2), ParameterValue::from_floating((static_cast<double>(config_.default_accel) / 255.0) * 100.0));

    auto read_prm = [this](std::uint8_t cmd) {
        return config_.bus_manager->read_system_parameter(config_.can_id, cmd);
    };

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetWorkMode)); res.ok() && res.value().size() >= 1) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::WorkMode), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetWorkingCurrent)); res.ok() && res.value().size() >= 2) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::WorkingCurrentMilliAmp), ParameterValue::from_unsigned(MksProtocol::readBe16(res.value().data() + res.value().size() - 2)));
    }
    
    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetSubdivision)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::Subdivision), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetEnPinActiveLevel)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::EnPinActiveLevel), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetMotorDirection)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::MotorDirection), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetAutoTurnOffScreen)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::AutoScreenOff), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetLockedRotorProtection)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::LockedRotorProtection), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetSubdivisionInterpolation)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::SubdivisionInterpolation), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetCanBitrate)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::CanBitrateIndex), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetCanId)); res.ok() && res.value().size() >= 2) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::CanId), ParameterValue::from_unsigned(MksProtocol::readBe16(res.value().data() + res.value().size() - 2)));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetGroupId)); res.ok() && res.value().size() >= 2) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::GroupId), ParameterValue::from_unsigned(MksProtocol::readBe16(res.value().data() + res.value().size() - 2)));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetKeyLock)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::KeyLock), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetHoldingCurrent)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::HoldingCurrentIndex), ParameterValue::from_unsigned(res.value().back()));
    }

    if (auto res = read_prm(static_cast<uint8_t>(MksCommand::SetLimitPortRemap)); res.ok() && !res.value().empty()) {
        add_if_persistable(motion_core::make_parameter_id(MksParameter::LimitPortRemap), ParameterValue::from_unsigned(res.value().back()));
    }

    const auto raw_result = config_.bus_manager->read_axis_telemetry(config_.can_id);
    if (raw_result.ok()) {
        const auto& raw = raw_result.value();
        if (raw.has_axis_position) add_if_persistable(motion_core::make_parameter_id(MksParameter::AxisPositionRaw), ParameterValue::from_signed(raw.axis_position));
        if (raw.has_speed_rpm) add_if_persistable(motion_core::make_parameter_id(MksParameter::MotorSpeedRpm), ParameterValue::from_signed(raw.speed_rpm));
        if (raw.has_protection_state) add_if_persistable(motion_core::make_parameter_id(MksParameter::ProtectionState), ParameterValue::from_unsigned(raw.protection_state));
        if (raw.has_motor_status) add_if_persistable(motion_core::make_parameter_id(MksParameter::MotorStatus), ParameterValue::from_unsigned(raw.motor_status));
    }

    add_if_persistable(motion_core::make_parameter_id(MksParameter::EnableMotor), ParameterValue::from_bool(enabled_));

    return motion_core::Result<motion_core::ParameterSet>::success(std::move(set));
}

motion_core::Result<void> MksAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto status = ensure_started();
    if (!status.ok()) return status;

    for (const auto& entry : patch.entries) {
        const auto* descriptor = find_mks_descriptor(entry.id);
        if (!descriptor || descriptor->read_only) continue;

        if (entry.id.domain == ParameterDomain::Common) {
            const auto common_parameter = static_cast<motion_core::CommonParameter>(entry.id.value);
            if (common_parameter == motion_core::CommonParameter::HardwareGearRatio) {
                const auto ratio_result = require_floating_value(entry.value, "HardwareGearRatio requires floating point");
                if (ratio_result.ok() && ratio_result.value() > 0.0) {
                    software_gear_ratio_ = ratio_result.value();
                    recalculate_axis_units_per_degree_from_software_params();
                }
            } else if (common_parameter == motion_core::CommonParameter::HardwareEncoderResolutionBits) {
                const auto bits_result = require_unsigned_value(entry.value, "HardwareEncoderResolutionBits requires unsigned integer");
                if (bits_result.ok() && bits_result.value() >= 1 && bits_result.value() <= 31) {
                    software_encoder_resolution_bits_ = static_cast<std::uint32_t>(bits_result.value());
                    recalculate_axis_units_per_degree_from_software_params();
                }
            } else if (common_parameter == motion_core::CommonParameter::LimitsMaxVelocityDegPerSec) {
                if (auto r = require_floating_value(entry.value, "double"); r.ok()) {
                    // RPM = (deg/sec / 6) * gear_ratio
                    const double rpm = (r.value() * software_gear_ratio_) / 6.0;
                    config_.default_speed = static_cast<std::uint16_t>(std::clamp(rpm, 1.0, 3000.0));
                }
            } else if (common_parameter == motion_core::CommonParameter::LimitsMaxAccelerationDegPerSec2) {
                if (auto r = require_floating_value(entry.value, "double"); r.ok()) {
                    // UI semantic: acceleration in percent, no gear-ratio scaling.
                    const double accel_percent = std::clamp(r.value(), 0.0, 100.0);
                    config_.default_accel = static_cast<std::uint8_t>(std::llround((accel_percent / 100.0) * 255.0));
                }
            }
            continue;
        }

        if (entry.id.domain != ParameterDomain::Mks) continue;

        const auto parameter = static_cast<MksParameter>(entry.id.value);
        std::vector<uint8_t> payload;

        // Centralized packing directly to raw commands
        switch (parameter) {
            case MksParameter::EnableMotor:
                if (auto b = require_bool_value(entry.value, "bool"); b.ok()) {
                    const auto enable_result = set_enabled_locked(b.value());
                    if (!enable_result.ok()) {
                        return enable_result;
                    }
                }
                break;
            case MksParameter::WorkMode:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetWorkMode), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::WorkingCurrentMilliAmp:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    MksProtocol::appendBe16(payload, static_cast<uint16_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetWorkingCurrent), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::Subdivision:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetSubdivision), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::EnPinActiveLevel:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetEnPinActiveLevel), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::MotorDirection:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetMotorDirection), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::AutoScreenOff:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetAutoTurnOffScreen), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::LockedRotorProtection:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetLockedRotorProtection), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::SubdivisionInterpolation:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetSubdivisionInterpolation), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::CanBitrateIndex:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetCanBitrate), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::CanId:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    MksProtocol::appendBe16(payload, static_cast<uint16_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetCanId), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::SlaveRespondMode:
                 // Deprecated / Complex param handling, ignored in strict mode
                 break;
            case MksParameter::SlaveActiveReport:
                 break;
            case MksParameter::GroupId:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    MksProtocol::appendBe16(payload, static_cast<uint16_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetGroupId), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::KeyLock:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetKeyLock), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::HoldingCurrentIndex:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetHoldingCurrent), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            case MksParameter::LimitPortRemap:
                if (auto r = require_unsigned_value(entry.value, "uint"); r.ok()) {
                    payload.push_back(static_cast<uint8_t>(r.value()));
                    const auto write_result = config_.bus_manager->execute_raw_command_sync(
                        config_.can_id, static_cast<uint8_t>(MksCommand::SetLimitPortRemap), payload);
                    if (!write_result.ok()) return motion_core::Result<void>::failure(write_result.error());
                }
                break;
            default:
                break;
        }
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisAdapter::ensure_started() const {
    if (!started_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "axis adapter is not started"});
    }
    return motion_core::Result<void>::success();
}

void MksAxisAdapter::recalculate_axis_units_per_degree_from_software_params() {
    config_.axis_units_per_degree = compute_axis_units_per_degree(
        software_encoder_resolution_bits_, software_gear_ratio_);
}

int MksAxisAdapter::mode_to_work_mode(const motion_core::AxisMode mode) const {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return 4; // SR_vFOC
        default:
            return -1;
    }
}

std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config) {
    return std::make_shared<MksAxisAdapter>(std::move(config));
}

} // namespace mks
