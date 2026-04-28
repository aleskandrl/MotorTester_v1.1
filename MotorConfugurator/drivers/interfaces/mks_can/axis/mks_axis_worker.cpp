#include "mks_can/axis/mks_axis_worker.h"

#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <cmath>

namespace mks {

namespace {

[[nodiscard]] MksBusCommand make_simple_command(const std::uint16_t can_id,
                                                const std::uint8_t command,
                                                const std::vector<std::uint8_t>& payload) {
    MksBusCommand out{};
    out.can_id = can_id;
    out.command = command;
    for (auto b : payload) {
        out.push_back(b);
    }
    return out;
}

[[nodiscard]] int axis_mode_to_mks_work_mode(const motion_core::AxisMode mode) {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
            return 5; // SR_vFOC
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return 3; // SR_OPEN
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return -1;
    }
    return -1;
}

} // namespace

MksAxisWorker::MksAxisWorker(Config config)
    : config_(std::move(config)) {
    runtime_can_id_.store(config_.can_id, std::memory_order_release);
    axis_units_per_degree_runtime_.store(config_.axis_units_per_degree, std::memory_order_release);
    software_gear_ratio_runtime_.store(config_.software_gear_ratio, std::memory_order_release);
    invert_direction_runtime_.store(config_.invert_direction, std::memory_order_release);
    telemetry_invert_position_sign_runtime_.store(config_.telemetry_invert_position_sign, std::memory_order_release);
    default_speed_rpm_runtime_.store(config_.default_speed_rpm, std::memory_order_release);
    default_accel_byte_runtime_.store(config_.default_accel_byte, std::memory_order_release);

    motion_core::TelemetrySnapshot initial{};
    initial.mode = motion_core::AxisMode::ProfilePosition;
    initial.state = motion_core::AxisState::Disabled;
    latest_telemetry_.store(initial, std::memory_order_release);

    pending_work_mode_.store(axis_mode_to_mks_work_mode(motion_core::AxisMode::ProfilePosition),
                             std::memory_order_release);
    active_mode_controller_ = &absolute_mode_;
}

MksAxisWorker::~MksAxisWorker() {
}


void MksAxisWorker::request_emergency_stop() {
    MksBusCommand ignored;
    while (tx_queue_.try_pop(ignored)) {}
    estop_latched_.store(true, std::memory_order_release);
}

void MksAxisWorker::set_runtime_can_id(const std::uint16_t can_id) {
    if (can_id == 0U || can_id > 0x07FFU) {
        return;
    }
    runtime_can_id_.store(can_id, std::memory_order_release);
}

motion_core::Result<void> MksAxisWorker::set_axis_units_per_degree(const double axis_units_per_degree) {
    if (!(std::isfinite(axis_units_per_degree) && std::abs(axis_units_per_degree) > 1e-9)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "axis_units_per_degree must be finite and non-zero"});
    }
    axis_units_per_degree_runtime_.store(axis_units_per_degree, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_software_gear_ratio(const double software_gear_ratio) {
    if (!(std::isfinite(software_gear_ratio) && software_gear_ratio > 0.0)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "software_gear_ratio must be finite and > 0"});
    }
    software_gear_ratio_runtime_.store(software_gear_ratio, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_invert_direction(const bool invert_direction) {
    invert_direction_runtime_.store(invert_direction, std::memory_order_release);
    return motion_core::Result<void>::success();
}

void MksAxisWorker::set_telemetry_invert_position_sign(const bool invert) {
    telemetry_invert_position_sign_runtime_.store(invert, std::memory_order_release);
}

motion_core::Result<void> MksAxisWorker::set_default_speed_rpm(const std::uint16_t default_speed_rpm) {
    if (default_speed_rpm == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "default_speed_rpm must be > 0"});
    }
    default_speed_rpm_runtime_.store(default_speed_rpm, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_default_accel_byte(const std::uint8_t default_accel_byte) {
    default_accel_byte_runtime_.store(default_accel_byte, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksAxisWorker::set_mode(const motion_core::AxisMode mode) {
    const auto previous_mode = mode_.load(std::memory_order_acquire);
    if (previous_mode == mode) {
        return motion_core::Result<void>::success();
    }

    const auto work_mode = axis_mode_to_mks_work_mode(mode);
    if (work_mode >= 0) {
        pending_work_mode_.store(work_mode, std::memory_order_release);
    }

    mode_.store(mode, std::memory_order_release);

    auto current = latest_telemetry_.load(std::memory_order_acquire);
    current.mode = mode;
    latest_telemetry_.store(current, std::memory_order_release);

    return motion_core::Result<void>::success();
}


bool MksAxisWorker::enqueue_service_point(const motion_core::ServiceCommandPoint& point) {
    return service_queue_.try_push(point);
}

bool MksAxisWorker::enqueue_command_batch(const std::vector<motion_core::MotionCommandPoint>& points) {
    auto* mode_controller = active_motion_mode();
    if (!mode_controller) {
        return false;
    }
    // We assume mode_controller->enqueue_motion_batch takes the updated type
    const auto res = mode_controller->enqueue_motion_batch(points);
    return res.ok();
}

void MksAxisWorker::clear_motion_queue() {
    (void)absolute_mode_.clear_motion_queue();
    (void)velocity_mode_.clear_motion_queue();
}

motion_core::Result<motion_core::MotionQueueStats> MksAxisWorker::query_motion_queue_stats() const {
    const auto mode = mode_.load(std::memory_order_acquire);
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return absolute_mode_.query_motion_queue_stats();
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return velocity_mode_.query_motion_queue_stats();
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return motion_core::Result<motion_core::MotionQueueStats>::failure(
                {motion_core::ErrorCode::Unsupported, "motion mode does not support queued setpoints"});
    }
    return motion_core::Result<motion_core::MotionQueueStats>::failure(
        {motion_core::ErrorCode::Unsupported, "unknown motion mode"});
}

motion_core::Result<motion_core::TelemetrySnapshot> MksAxisWorker::read_telemetry() const {
    return motion_core::Result<motion_core::TelemetrySnapshot>::success(
        latest_telemetry_.load(std::memory_order_acquire));
}



bool MksAxisWorker::consume_tx_command(MksBusCommand& command) {
    return tx_queue_.try_pop(command);
}

void MksAxisWorker::publish_telemetry(const motion_core::TelemetrySnapshot& telemetry) {
    auto updated = telemetry;
    updated.mode = mode_.load(std::memory_order_acquire);
    latest_telemetry_.store(updated, std::memory_order_release);
}

void MksAxisWorker::step(const std::chrono::steady_clock::time_point now) {
    if (estop_latched_.load(std::memory_order_acquire)) {
        // Send estop frame aggressively
        auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire), 
                                           static_cast<std::uint8_t>(MksCommand::EmergencyStop), {});
        (void)tx_queue_.try_push(bus_cmd);
        return;
    }

    if (handle_service_request()) {
        return;
    }
    
    motion_core::ServiceCommandPoint cmd{};
    bool handled_svc = false;
    while (service_queue_.try_pop(cmd)) {
        handle_service_queue_command(cmd);
        handled_svc = true;
    }
    if (handled_svc) {
        return; // Service has priority
    }

    handle_motion_request(now);
}

bool MksAxisWorker::handle_service_request() {
    bool pushed_any = false;
    const auto work_mode = pending_work_mode_.exchange(-1, std::memory_order_acq_rel);
    if (work_mode >= 0) {
        auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire),
                                           static_cast<std::uint8_t>(MksCommand::SetWorkMode),
                                           {static_cast<std::uint8_t>(work_mode)});
        if (tx_queue_.try_push(bus_cmd)) {
            pushed_any = true;
        } else {
            pending_work_mode_.store(work_mode, std::memory_order_release);
        }
    }
    return pushed_any;
}

void MksAxisWorker::handle_service_queue_command(const motion_core::ServiceCommandPoint& command) {
    auto try_push = [&](const std::uint8_t cmd, const std::vector<std::uint8_t>& payload) {
        auto bus_cmd = make_simple_command(runtime_can_id_.load(std::memory_order_acquire), cmd, payload);
        (void)tx_queue_.try_push(bus_cmd);
    };

    switch (command.type) {
        case motion_core::ServiceCommandType::Enable:
            try_push(static_cast<std::uint8_t>(MksCommand::EnableMotor), {1U});
            break;
        case motion_core::ServiceCommandType::Disable:
            try_push(static_cast<std::uint8_t>(MksCommand::EmergencyStop), {});
            try_push(static_cast<std::uint8_t>(MksCommand::EnableMotor), {0U});
            break;
        case motion_core::ServiceCommandType::ClearErrors:
            try_push(static_cast<std::uint8_t>(MksCommand::ReleaseStallProtection), {});
            break;
        case motion_core::ServiceCommandType::Home:
            try_push(static_cast<std::uint8_t>(MksCommand::GoHome), {});
            break;
        case motion_core::ServiceCommandType::SetZero:
            try_push(static_cast<std::uint8_t>(MksCommand::SetCurrentAxisToZero), {});
            break;
        case motion_core::ServiceCommandType::ResetDrive:
            // Custom or not implemented in MKS
            break;
        case motion_core::ServiceCommandType::ClearMotionQueue:
            clear_motion_queue();
            break;
        case motion_core::ServiceCommandType::SetOperatingMode:
            (void)set_mode(command.requested_mode);
            break;
    }
}

void MksAxisWorker::handle_motion_request(const std::chrono::steady_clock::time_point now) {
    auto* mode_controller = active_motion_mode();
    if (!mode_controller) {
        if (active_mode_controller_) {
            active_mode_controller_->on_mode_exit();
            active_mode_controller_ = nullptr;
        }
        return;
    }

    if (active_mode_controller_ != mode_controller) {
        if (active_mode_controller_) {
            active_mode_controller_->on_mode_exit();
        }
        mode_controller->on_mode_enter(now);
        active_mode_controller_ = mode_controller;
    }

    const auto context = build_motion_context();
    const auto step_result = mode_controller->step(now, context);
    if (!step_result.ok()) {
        return;
    }

    if (!step_result.value().has_command) {
        return;
    }

    push_tx_command(step_result.value().command);
}

void MksAxisWorker::push_tx_command(const MksBusCommand& command) {
    (void)tx_queue_.try_push(command);
}



MksMotionModeBase* MksAxisWorker::active_motion_mode() {
    const auto mode = mode_.load(std::memory_order_acquire);
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition:
            return &absolute_mode_;
        case motion_core::AxisMode::ProfileVelocity:
        case motion_core::AxisMode::CyclicSyncVelocity:
            return &velocity_mode_;
        case motion_core::AxisMode::CyclicSyncPosition:
        case motion_core::AxisMode::Homing:
        case motion_core::AxisMode::ManualHoming:
        case motion_core::AxisMode::CyclicSyncTorque:
        case motion_core::AxisMode::VendorSpecific:
            return nullptr;
    }
    return nullptr;
}

MksMotionBuildContext MksAxisWorker::build_motion_context() const {
    MksMotionBuildContext context{};
    context.can_id = runtime_can_id_.load(std::memory_order_acquire);
    context.axis_units_per_degree = axis_units_per_degree_runtime_.load(std::memory_order_acquire);
    context.software_gear_ratio = software_gear_ratio_runtime_.load(std::memory_order_acquire);
    context.bus_cycle_period_sec =
        static_cast<double>(config_.cycle_period.count()) / 1'000'000.0;
    context.invert_direction = invert_direction_runtime_.load(std::memory_order_acquire);
    context.telemetry_invert_position_sign = telemetry_invert_position_sign_runtime_.load(std::memory_order_acquire);
    context.fallback_speed_rpm = default_speed_rpm_runtime_.load(std::memory_order_acquire);
    context.fallback_accel_byte = default_accel_byte_runtime_.load(std::memory_order_acquire);
    return context;
}

} // namespace mks
