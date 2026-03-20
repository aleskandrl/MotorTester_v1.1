#pragma once

#include "motion_core/parameter_id.h"
#include "motion_core/parameter_types.h"

#include <array>
#include <cstdint>

namespace ethercat_driver {

struct ParameterDefinition {
    // Technical data
    motion_core::ParameterId id;
    const char* rdt_key;
    uint16_t index;
    uint8_t sub_index;
    uint8_t data_size;
    motion_core::ParameterValueType type;
    bool is_read_only;
    bool persistable_runtime;

    // UI and validation data
    const char* name;
    const char* group;
    const char* unit;
    const char* description;
    motion_core::ParameterValue min_value;
    motion_core::ParameterValue max_value;
};

constexpr std::array<ParameterDefinition, 58> p100e_dictionary = {{

    // ── Common/Homing ──────────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingMethod),
     "homing.method", 0x6098, 0, 1, motion_core::ParameterValueType::SignedInteger, false, true,
     "Homing method", "Common/Homing", "", "Homing method (17=fwd limit, 18=rev limit).",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(35)},

    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedSwitchDegPerSec),
     "homing.speed_switch", 0x6099, 1, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing speed (switch)", "Common/Homing", "instr/s", "Speed during search for switch.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedZeroDegPerSec),
     "homing.speed_zero", 0x6099, 2, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing speed (zero)", "Common/Homing", "instr/s", "Speed during search for zero.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::HomingAcceleration),
     "homing.accel", 0x609A, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Homing acceleration", "Common/Homing", "instr/s2", "Acceleration/deceleration for homing.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::HomeOffset),
     "homing.offset", 0x607C, 0, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Home offset", "Common/Homing", "instr", "Home offset applied after homing sequence.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    // ── Common/Limits ──────────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::SoftLimitMinCounts),
     "limits.soft_min", 0x607D, 1, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Software position limit (min)", "Common/Limits", "instr", "Minimum software position limit in instruction units.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::SoftLimitMaxCounts),
     "limits.soft_max", 0x607D, 2, 4, motion_core::ParameterValueType::SignedInteger, false, true,
     "Software position limit (max)", "Common/Limits", "instr", "Maximum software position limit in instruction units.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::MaxTorqueTenthsPercent),
     "limits.max_torque", 0x6072, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Max torque", "Common/Limits", "0.1%", "Maximum commanded torque in torque modes.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::TorqueLimitForward),
     "limits.torque_fwd", 0x60E0, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Forward torque limit", "Common/Limits", "0.1%", "Forward torque limit (3000 = 300%).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::TorqueLimitReverse),
     "limits.torque_rev", 0x60E1, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Reverse torque limit", "Common/Limits", "0.1%", "Reverse torque limit (3000 = 300%).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(3000)},

    // ── Common/Motion ──────────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::MaxProfileVelocityCountsPerSec),
     "motion.max_speed", 0x607F, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Max speed", "Common/Motion", "instr/s", "Maximum running speed.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ProfileVelocity),
     "motion.profile_vel", 0x6081, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile velocity", "Common/Motion", "instr/s", "Speed in profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ProfileAcceleration),
     "motion.profile_accel", 0x6083, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile acceleration", "Common/Motion", "instr/s2", "Acceleration for profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ProfileDeceleration),
     "motion.profile_decel", 0x6084, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Profile deceleration", "Common/Motion", "instr/s2", "Deceleration for profile position mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::QuickStopDecel),
     "motion.quickstop_decel", 0x6085, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Quick stop deceleration", "Common/Motion", "instr/s2", "Deceleration for emergency stop.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::QuickStopOptionCode),
     "motion.quickstop_option", 0x605A, 0, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Quick stop option code", "Common/Motion", "", "Quick stop behavior option code.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(6)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PositionArrivalThreshold),
     "motion.arrival_threshold", 0x6067, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Position arrival threshold", "Common/Motion", "instr", "Window for target-reached flag (status bit 10).",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(65535)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::TargetVelocity),
     "motion.target_velocity", 0x60FF, 0, 4, motion_core::ParameterValueType::SignedInteger, false, false,
     "Target velocity", "Common/Motion", "instr/s", "Runtime velocity target in PV/CSV modes.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ActualVelocity),
     "motion.actual_velocity", 0x606C, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Actual velocity", "Common/Motion", "instr/s", "Actual velocity feedback.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::TargetTorque),
     "motion.target_torque", 0x6071, 0, 2, motion_core::ParameterValueType::SignedInteger, false, false,
     "Target torque", "Common/Motion", "0.1%", "Runtime torque target in PT/CST modes.",
     motion_core::ParameterValue::from_signed(-3000), motion_core::ParameterValue::from_signed(3000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::TorqueSlope),
     "motion.torque_slope", 0x6087, 0, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Torque slope", "Common/Motion", "0.1%/ms", "Torque slope in profile torque mode.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ModeDisplay),
     "motion.mode_display", 0x6061, 0, 1, motion_core::ParameterValueType::SignedInteger, true, false,
     "Mode display", "Common/Motion", "", "Current active operation mode reported by drive.",
     motion_core::ParameterValue::from_signed(-128), motion_core::ParameterValue::from_signed(127)},

    // ── Common/Mechanics ───────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::AxialResolution),
     "mechanics.axial_resolution", 0x6091, 2, 4, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Axial resolution", "Common/Mechanics", "instr/rev",
     "Instruction units per motor revolution. All PDO position values are in these units. Default 10000.",
     motion_core::ParameterValue::from_unsigned(1), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::FollowingErrorActualValue),
     "mechanics.following_error", 0x60F4, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Following error actual value", "Common/Mechanics", "instr", "Actual following error.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::DigitalInputs),
     "io.digital_inputs", 0x60FD, 0, 4, motion_core::ParameterValueType::UnsignedInteger, true, false,
     "Digital inputs", "Common/IO", "bitmask", "Digital input status bitmask.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PhysicalOutputs),
     "io.physical_outputs", 0x60FE, 1, 4, motion_core::ParameterValueType::UnsignedInteger, false, false,
     "Physical outputs", "Common/IO", "bitmask", "Digital output command bitmask.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    // ── Vendor Specific/PID ────────────────────────────────────────────────
    // 0x2001 "PID Adjust Parameter" (DT2001 from ESI XML)
    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidPosKp),
     "pid.pos_kp", 0x2001, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Position Kp (PA9)", "Vendor Specific/PID", "", "Position proportional gain.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidVelKp),
     "pid.vel_kp", 0x2001, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed Kp (PA5)", "Vendor Specific/PID", "Hz", "Speed loop proportional gain.",
     motion_core::ParameterValue::from_signed(5), motion_core::ParameterValue::from_signed(2000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidVelKi),
     "pid.vel_ki", 0x2001, 3, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed Ki (PA6)", "Vendor Specific/PID", "ms", "Speed loop integral time constant.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    // ── Vendor Specific/Filters ────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidPosFF),
     "pid.pos_smooth", 0x2001, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Position smooth filter (PA19)", "Vendor Specific/Filters", "x0.1ms",
     "Position command exponential smoothing filter time constant.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(1000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidTorqueFilter),
     "pid.torque_filter", 0x2001, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Torque filter (PA7)", "Vendor Specific/Filters", "%", "Torque command low-pass filter cutoff.",
     motion_core::ParameterValue::from_signed(20), motion_core::ParameterValue::from_signed(500)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidSpdDetectionFilter),
     "pid.spd_filter", 0x2001, 6, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Speed detection filter (PA8)", "Vendor Specific/Filters", "%", "Speed feedback low-pass filter cutoff.",
     motion_core::ParameterValue::from_signed(20), motion_core::ParameterValue::from_signed(500)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidAccelTime),
     "pid.accel_time", 0x2001, 7, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Acceleration time (PA40)", "Vendor Specific/Filters", "ms", "Time to ramp from 0 to 1000 rpm.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(10000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidDecelTime),
     "pid.decel_time", 0x2001, 8, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Deceleration time (PA41)", "Vendor Specific/Filters", "ms", "Time to ramp from 1000 rpm to 0.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(10000)},

    // ── Vendor Specific/Current Control ────────────────────────────────────
    // 0x2008 "Step Mode Parameter I" (DT2008 from ESI XML)
    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidCurrentKp),
     "pid.cur_kp", 0x2008, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Current KP", "Vendor Specific/Current Control", "", "Current loop proportional gain.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::PidCurrentKi),
     "pid.cur_ki", 0x2008, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Current KI", "Vendor Specific/Current Control", "", "Current loop integral gain.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::StepLockCurrent),
     "step.lock_current", 0x2008, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Locking current", "Vendor Specific/Current Control", "", "Step mode locking current.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::StepRunCurrent),
     "step.run_current", 0x2008, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Running current", "Vendor Specific/Current Control", "", "Step mode running current.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    // ── Vendor Specific/Hardware ───────────────────────────────────────────
    // 0x2000 "Basic Control Parameter" (DT2000 from ESI XML)
    {motion_core::make_parameter_id(motion_core::EthercatParameter::EncoderSelection),
     "hw.encoder_sel", 0x2000, 10, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Encoder selection (PA62)", "Vendor Specific/Hardware", "",
     "4 = single-turn absolute encoder, 5 = multi-turn absolute encoder.",
     motion_core::ParameterValue::from_signed(4), motion_core::ParameterValue::from_signed(5)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::EncoderResolutionBits),
     "hw.encoder_bits", 0x2000, 11, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Encoder resolution (PA95)", "Vendor Specific/Hardware", "bits",
     "Motor encoder resolution in bits. Default 17 (2^17=131072 counts/rev). Modify carefully.",
     motion_core::ParameterValue::from_signed(10), motion_core::ParameterValue::from_signed(32)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicInitialStatusDisplay),
     "basic.initial_status_display", 0x2000, 1, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Initial status display (PA3)", "Vendor Specific/Basic", "", "Initial panel status display code.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(23)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicActionOnStop),
     "basic.action_on_stop", 0x2000, 2, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake delay on stop (PA47)", "Vendor Specific/Basic", "x10ms", "Delay to release brake after motor stops.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(200)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicActuatorAction),
     "basic.actuator_action", 0x2000, 3, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake delay on run (PA48)", "Vendor Specific/Basic", "x10ms", "Delay to engage brake after motor runs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(200)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicRunningBrakeSpeed),
     "basic.running_brake_speed", 0x2000, 4, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Brake speed (PA49)", "Vendor Specific/Basic", "rpm", "Speed threshold for brake logic.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(3000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicTorqueControlSpeedLimit),
     "basic.torque_speed_limit", 0x2000, 5, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Torque mode speed (PA50)", "Vendor Specific/Basic", "rpm", "Speed limit in torque control mode.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(5000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicServoOnDelayMs),
     "basic.servo_on_delay", 0x2000, 6, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Servo OFF delay (PA54)", "Vendor Specific/Basic", "ms", "Delay to cut current after servo off signal.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(30000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicIoInputActiveLevelWord),
     "basic.io_input_active_level", 0x2000, 7, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input logic word (PA55)", "Vendor Specific/Basic", "", "Bitmask to invert digital inputs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(31)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicIoOutputActiveLevelWord),
     "basic.io_output_active_level", 0x2000, 8, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Output logic word (PA57)", "Vendor Specific/Basic", "", "Bitmask to invert digital outputs.",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(31)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicIoDebounceMs),
     "basic.io_debounce", 0x2000, 9, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Input filter / debounce (PA58)", "Vendor Specific/Basic", "ms", "IO jitter suppression time constant.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(1000)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicMotorPoles),
     "basic.motor_poles", 0x2000, 12, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "Motor poles (PA96)", "Vendor Specific/Basic", "", "Configured motor pole count.",
     motion_core::ParameterValue::from_signed(1), motion_core::ParameterValue::from_signed(360)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::BasicPwmDutyCycle),
     "basic.pwm_duty", 0x2000, 13, 2, motion_core::ParameterValueType::SignedInteger, false, true,
     "PWM duty cycle (PA99)", "Vendor Specific/Basic", "%", "PWM duty cycle limit.",
     motion_core::ParameterValue::from_signed(5), motion_core::ParameterValue::from_signed(90)},

    // ── Vendor Specific/Communication ──────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::CommunicationFixedAddress),
     "comm.fixed_address", 0x2002, 3, 2, motion_core::ParameterValueType::SignedInteger, false, false,
     "Fixed address", "Vendor Specific/Communication", "", "Communication fixed address (bootstrap).",
     motion_core::ParameterValue::from_signed(0), motion_core::ParameterValue::from_signed(32767)},

    // ── Probe ───────────────────────────────────────────────────────────────
    {motion_core::make_parameter_id(motion_core::EthercatParameter::ProbeFunction),
     "probe.function", 0x60B8, 0, 2, motion_core::ParameterValueType::UnsignedInteger, false, true,
     "Probe function", "Common/Probe", "", "Probe function configuration.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(65535)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::ProbeStatus),
     "probe.status", 0x60B9, 0, 4, motion_core::ParameterValueType::UnsignedInteger, true, false,
     "Probe status", "Common/Probe", "", "Probe status bitfield.",
     motion_core::ParameterValue::from_unsigned(0), motion_core::ParameterValue::from_unsigned(4294967295u)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::Probe1RisePosition),
     "probe.p1_rise", 0x60BA, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 1 rise position", "Common/Probe", "instr", "Probe 1 rising edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::Probe1FallPosition),
     "probe.p1_fall", 0x60BB, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 1 fall position", "Common/Probe", "instr", "Probe 1 falling edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::Probe2RisePosition),
     "probe.p2_rise", 0x60BC, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 2 rise position", "Common/Probe", "instr", "Probe 2 rising edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},

    {motion_core::make_parameter_id(motion_core::EthercatParameter::Probe2FallPosition),
     "probe.p2_fall", 0x60BD, 0, 4, motion_core::ParameterValueType::SignedInteger, true, false,
     "Probe 2 fall position", "Common/Probe", "instr", "Probe 2 falling edge latched position.",
     motion_core::ParameterValue::from_signed(-2147483647 - 1), motion_core::ParameterValue::from_signed(2147483647)},
}};

} // namespace ethercat_driver
