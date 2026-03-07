#include "motion_core/axis_control_service.h"
#include "motion_core/config/hal_runtime_config.h"

#ifdef MOTION_CORE_WITH_DRIVERS
#include "mks/adapter/mks_runtime_factory.h"
#include "ethercat/manager/ethercat_runtime_factory.h"
#include "motion_core/config/axis_config_json.h"
#endif

#include <deque>
#include <utility>

namespace motion_core {

namespace {

#ifdef MOTION_CORE_WITH_DRIVERS
mks::MksRuntimeConfig convert_to_mks_config(const HalRuntimeConfig& hal_cfg) {
    mks::MksRuntimeConfig mks_cfg;
    for (const auto& bus : hal_cfg.mks_buses) {
        mks::MksBusRuntimeConfig b;
        b.interface_id = bus.interface_id;
        b.device_path = bus.device_path;
        b.baud_rate = bus.baud_rate;
        mks_cfg.buses.push_back(std::move(b));
    }
    for (const auto& axis : hal_cfg.axes) {
        if (axis.transport == AxisTransportKind::CanBus) {
            mks::MksAxisRuntimeConfig a;
            a.axis_id = axis.axis_id;
            a.axis_name = axis.axis_name;
            a.can_id = axis.transport_address;
            
            // Find appropriate bus and add axis to it
            for (auto& bus : mks_cfg.buses) {
                if (bus.interface_id == axis.bus_ref) {
                    bus.axes.push_back(a);
                    break;
                }
            }
        }
    }
    return mks_cfg;
}

ethercat_driver::EthercatRuntimeConfig convert_to_ecat_config(const HalRuntimeConfig& hal_cfg) {
    ethercat_driver::EthercatRuntimeConfig ecat_cfg;
    // Берем первую шину EtherCAT (упрощение для Phase 4)
    if (!hal_cfg.ethercat_buses.empty()) {
        ecat_cfg.bus.interface_name = hal_cfg.ethercat_buses[0].interface_name;
    }
    for (const auto& axis : hal_cfg.axes) {
        if (axis.transport == AxisTransportKind::Ethercat) {
            ethercat_driver::EthercatAxisRuntimeConfig a;
            a.axis_id = axis.axis_id;
            a.axis_name = axis.axis_name;
            ecat_cfg.axes.push_back(a);
        }
    }
    return ecat_cfg;
}
#endif

} // namespace


Result<void> AxisControlService::open_runtime(const HalRuntimeConfig& config) {
#ifdef MOTION_CORE_WITH_DRIVERS
    // 1. Создание MKS шин через MksRuntimeFactory
    const auto mks_cfg = convert_to_mks_config(config);
    if (!mks_cfg.buses.empty()) {
        const auto mks_res = mks::build_mks_runtime(mks_cfg);
        if (!mks_res.ok()) return Result<void>::failure(mks_res.error());
        for (const auto& axis : mks_res.value().axes) {
            const auto add_res = add_axis(axis);
            if (!add_res.ok()) return add_res;
        }
    }

    // 2. Создание EtherCAT шин через EthercatRuntimeFactory
    const auto ecat_cfg = convert_to_ecat_config(config);
    if (!ecat_cfg.axes.empty()) {
        const auto ecat_res = ethercat_driver::build_ethercat_runtime(ecat_cfg);
        if (!ecat_res.ok()) return Result<void>::failure(ecat_res.error());
        for (const auto& axis : ecat_res.value().axes) {
            const auto add_res = add_axis(axis);
            if (!add_res.ok()) return add_res;
        }
    }
#else
    (void)config;
#endif

    return Result<void>::success();
}

Result<void> AxisControlService::close_runtime() {
    stop_runtime();
    std::lock_guard<std::mutex> lock(mutex_);
    axes_.clear();
    return Result<void>::success();
}

Result<void> AxisControlService::start_runtime() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [_, slot] : axes_) {
        const auto res = slot.axis->start();
        if (!res.ok()) return res;
    }
    return start_dispatch_loop();
}

Result<void> AxisControlService::stop_runtime() {
    stop_dispatch_loop();
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [_, slot] : axes_) {
        slot.axis->stop();
    }
    return Result<void>::success();
}

bool AxisControlService::is_runtime_open() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return !axes_.empty();
}

bool AxisControlService::is_runtime_active() const noexcept {
    return is_dispatch_loop_running();
}


Result<void> AxisControlService::add_axis(const std::shared_ptr<IAxis>& axis) {
    if (!axis) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis is null"});
    }

    const AxisInfo info = axis->info();
    if (!info.id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (axes_.find(info.id.value) != axes_.end()) {
        return Result<void>::failure({ErrorCode::AlreadyExists, "axis already registered"});
    }

    AxisSlot slot{};
    slot.axis = axis;
    slot.stats.capacity = slot.queue_options.capacity;
    axes_.emplace(info.id.value, std::move(slot));
    return Result<void>::success();
}

Result<void> AxisControlService::remove_axis(const AxisId axis_id) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = axes_.find(axis_id.value);
    if (it == axes_.end()) {
        return Result<void>::failure({ErrorCode::NotFound, "axis not found"});
    }
    axes_.erase(it);
    return Result<void>::success();
}

Result<std::vector<AxisInfo>> AxisControlService::list_axes() const {
    std::vector<AxisInfo> out;
    std::lock_guard<std::mutex> lock(mutex_);
    out.reserve(axes_.size());
    for (const auto& [_, slot] : axes_) {
        out.push_back(slot.axis->info());
    }
    return Result<std::vector<AxisInfo>>::success(std::move(out));
}

Result<void> AxisControlService::start_axis(const AxisId axis_id) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->start();
}

Result<void> AxisControlService::stop_axis(const AxisId axis_id) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->stop();
}

Result<void> AxisControlService::enable_axis(const AxisId axis_id, const bool enabled) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->set_enabled(enabled);
}

Result<void> AxisControlService::set_axis_mode(const AxisId axis_id, const AxisMode mode) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->set_mode(mode);
}

Result<void> AxisControlService::submit_command(const AxisId axis_id, const AxisCommand& command) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->apply_command(command);
}

Result<void> AxisControlService::move_absolute(const AxisId axis_id,
                                               const double position_deg,
                                               const bool has_speed_rpm,
                                               const std::uint16_t speed_rpm,
                                               const bool has_accel_percent,
                                               const double accel_percent) {
    AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.target_position_deg = position_deg;
    cmd.has_profile_speed_rpm = has_speed_rpm;
    cmd.profile_speed_rpm = speed_rpm;
    cmd.has_profile_accel_percent = has_accel_percent;
    cmd.profile_accel_percent = accel_percent;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::move_relative(const AxisId axis_id,
                                               const double delta_deg,
                                               const bool has_speed_rpm,
                                               const std::uint16_t speed_rpm,
                                               const bool has_accel_percent,
                                               const double accel_percent) {
    AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.is_relative = true;
    cmd.target_position_deg = delta_deg;
    cmd.has_profile_speed_rpm = has_speed_rpm;
    cmd.profile_speed_rpm = speed_rpm;
    cmd.has_profile_accel_percent = has_accel_percent;
    cmd.profile_accel_percent = accel_percent;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::emergency_stop(const AxisId axis_id) {
    AxisCommand cmd{};
    cmd.emergency_stop = true;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::clear_errors(const AxisId axis_id) {
    AxisCommand cmd{};
    cmd.clear_errors = true;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::go_home(const AxisId axis_id) {
    AxisCommand cmd{};
    cmd.go_home = true;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::set_zero(const AxisId axis_id) {
    AxisCommand cmd{};
    cmd.set_zero = true;
    return submit_command(axis_id, cmd);
}

Result<void> AxisControlService::apply_safe_baseline(const AxisId axis_id,
                                                     const SafetyBaselineOptions& options) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    if (options.force_disable) {
        const auto disable_result = enable_axis(axis_id, false);
        if (!disable_result.ok()) {
            return disable_result;
        }
    }

    if (options.sync_target_to_actual) {
        const auto telemetry_result = read_telemetry(axis_id);
        if (!telemetry_result.ok()) {
            return Result<void>::failure(telemetry_result.error());
        }

        AxisCommand cmd{};
        cmd.has_target_position = true;
        cmd.target_position_deg = telemetry_result.value().actual_position_deg;
        cmd.has_profile_speed_rpm = true;
        cmd.profile_speed_rpm = 0;
        cmd.has_profile_accel_percent = true;
        cmd.profile_accel_percent = 0.0;
        const auto cmd_result = submit_command(axis_id, cmd);
        if (!cmd_result.ok()) {
            return cmd_result;
        }
    }

    return Result<void>::success();
}

Result<void> AxisControlService::set_motion_queue_options(const AxisId axis_id, const MotionQueueOptions& options) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }
    if (options.capacity == 0) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "queue capacity must be > 0"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto* slot = find_slot_ptr_locked(axis_id);
    if (!slot) {
        return Result<void>::failure({ErrorCode::NotFound, "axis not found"});
    }

    slot->queue_options = options;
    while (slot->queue.size() > options.capacity) {
        slot->queue.pop_front();
        ++slot->stats.dropped;
    }
    slot->stats.capacity = options.capacity;
    slot->stats.size = slot->queue.size();
    return Result<void>::success();
}

Result<void> AxisControlService::enqueue_motion_point(const AxisId axis_id, const MotionPoint& point) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto* slot = find_slot_ptr_locked(axis_id);
    if (!slot) {
        return Result<void>::failure({ErrorCode::NotFound, "axis not found"});
    }

    return enqueue_motion_point_locked(*slot, point);
}

Result<void> AxisControlService::enqueue_motion_point_locked(AxisSlot& slot, const MotionPoint& point) {
    const std::size_t capacity = slot.queue_options.capacity;
    const bool full = slot.queue.size() >= capacity;

    if (slot.queue_options.policy == QueuePolicy::ReplaceLatest) {
        slot.queue.clear();
        slot.queue.push_back(point);
        ++slot.stats.pushed;
        slot.stats.size = slot.queue.size();
        return Result<void>::success();
    }

    if (full) {
        if (slot.queue_options.policy == QueuePolicy::DropNewest) {
            ++slot.stats.dropped;
            return Result<void>::success();
        }
        if (slot.queue_options.policy == QueuePolicy::DropOldest) {
            slot.queue.pop_front();
            ++slot.stats.dropped;
        } else {
            return Result<void>::failure({ErrorCode::Busy, "motion queue is full"});
        }
    }

    slot.queue.push_back(point);
    ++slot.stats.pushed;
    slot.stats.size = slot.queue.size();
    return Result<void>::success();
}

Result<void> AxisControlService::enqueue_motion_batch(const AxisId axis_id, const std::vector<MotionPoint>& points) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto* slot = find_slot_ptr_locked(axis_id);
    if (!slot) {
        return Result<void>::failure({ErrorCode::NotFound, "axis not found"});
    }

    for (const auto& p : points) {
        const auto r = enqueue_motion_point_locked(*slot, p);
        if (!r.ok()) {
            return r;
        }
    }
    return Result<void>::success();
}

Result<void> AxisControlService::clear_motion_queue(const AxisId axis_id) {
    if (!axis_id.valid()) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto* slot = find_slot_ptr_locked(axis_id);
    if (!slot) {
        return Result<void>::failure({ErrorCode::NotFound, "axis not found"});
    }

    slot->queue.clear();
    slot->stats.size = 0;
    return Result<void>::success();
}

Result<MotionQueueStats> AxisControlService::get_motion_queue_stats(const AxisId axis_id) const {
    if (!axis_id.valid()) {
        return Result<MotionQueueStats>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto* slot = find_slot_ptr_locked(axis_id);
    if (!slot) {
        return Result<MotionQueueStats>::failure({ErrorCode::NotFound, "axis not found"});
    }

    auto stats = slot->stats;
    stats.size = slot->queue.size();
    stats.capacity = slot->queue_options.capacity;
    return Result<MotionQueueStats>::success(stats);
}

Result<void> AxisControlService::start_dispatch_loop(const std::chrono::milliseconds period) {
    return dispatch_loop_.start(period, [this]() { dispatch_tick(); });
}

Result<void> AxisControlService::stop_dispatch_loop() {
    return dispatch_loop_.stop();
}

bool AxisControlService::is_dispatch_loop_running() const noexcept {
    return dispatch_loop_.is_running();
}

Result<AxisTelemetry> AxisControlService::read_telemetry(const AxisId axis_id) const {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<AxisTelemetry>::failure(found.error());
        axis = found.value();
    }
    return axis->read_telemetry();
}

Result<std::vector<ParameterDescriptor>> AxisControlService::list_parameters(const AxisId axis_id) const {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<std::vector<ParameterDescriptor>>::failure(found.error());
        axis = found.value();
    }
    return axis->list_parameters();
}

Result<ParameterSet> AxisControlService::read_parameters(const AxisId axis_id) const {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<ParameterSet>::failure(found.error());
        axis = found.value();
    }
    return axis->read_parameters();
}

Result<void> AxisControlService::apply_parameter_patch(const AxisId axis_id, const ParameterPatch& patch) {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<void>::failure(found.error());
        axis = found.value();
    }
    return axis->apply_parameter_patch(patch);
}

Result<AxisConfig> AxisControlService::export_axis_config(const AxisId axis_id) const {
    std::shared_ptr<IAxis> axis;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = find_axis_locked(axis_id);
        if (!found.ok()) return Result<AxisConfig>::failure(found.error());
        axis = found.value();
    }

    const auto info = axis->info();
    const auto params_res = axis->read_parameters();
    if (!params_res.ok()) return Result<AxisConfig>::failure(params_res.error());

    AxisConfig cfg;
    cfg.axis_id = info.id;
    cfg.axis_name = info.name;
    cfg.transport = info.transport;
    cfg.parameters = params_res.value();

    // Пытаемся вытащить gear_ratio из параметров, если он там есть
    for (const auto& entry : cfg.parameters.entries) {
        if (entry.id.domain == ParameterDomain::Common) {
            if (static_cast<CommonParameter>(entry.id.value) == CommonParameter::HardwareGearRatio) {
                cfg.gear_ratio = entry.value.floating_value;
            } else if (static_cast<CommonParameter>(entry.id.value) == CommonParameter::HardwareEncoderResolutionBits) {
                cfg.encoder_resolution_bits = static_cast<uint32_t>(entry.value.unsigned_value);
            }
        }
    }

    return Result<AxisConfig>::success(std::move(cfg));
}

Result<void> AxisControlService::apply_axis_config(const AxisId axis_id, const AxisConfig& config) {
    ParameterPatch patch;
    patch.entries = config.parameters.entries;
    return apply_parameter_patch(axis_id, patch);
}

Result<std::shared_ptr<IAxis>> AxisControlService::find_axis_locked(const AxisId axis_id) const {
    if (!axis_id.valid()) {
        return Result<std::shared_ptr<IAxis>>::failure({ErrorCode::InvalidArgument, "axis id is invalid"});
    }

    const auto it = axes_.find(axis_id.value);
    if (it == axes_.end()) {
        return Result<std::shared_ptr<IAxis>>::failure({ErrorCode::NotFound, "axis not found"});
    }
    return Result<std::shared_ptr<IAxis>>::success(it->second.axis);
}

AxisControlService::AxisSlot* AxisControlService::find_slot_ptr_locked(const AxisId axis_id) {
    if (!axis_id.valid()) {
        return nullptr;
    }
    const auto it = axes_.find(axis_id.value);
    if (it == axes_.end()) {
        return nullptr;
    }
    return &it->second;
}

const AxisControlService::AxisSlot* AxisControlService::find_slot_ptr_locked(const AxisId axis_id) const {
    if (!axis_id.valid()) {
        return nullptr;
    }
    const auto it = axes_.find(axis_id.value);
    if (it == axes_.end()) {
        return nullptr;
    }
    return &it->second;
}

void AxisControlService::dispatch_tick() {
    struct PendingDispatch {
        AxisId axis_id{};
        std::shared_ptr<IAxis> axis;
        MotionPoint point{};
        AxisCommand command{};
    };

    std::vector<PendingDispatch> commands;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        commands.reserve(axes_.size());

        for (auto& [axis_key, slot] : axes_) {
            if (slot.queue.empty()) {
                continue;
            }

            const MotionPoint point = slot.queue.front();
            slot.queue.pop_front();
            slot.stats.size = slot.queue.size();

            AxisCommand cmd{};
            cmd.has_target_position = true;
            cmd.target_position_deg = point.position_deg;
            cmd.is_relative = point.relative;
            cmd.has_profile_speed_rpm = point.has_speed_rpm;
            cmd.profile_speed_rpm = point.speed_rpm;
            cmd.has_profile_accel_percent = point.has_accel_percent;
            cmd.profile_accel_percent = point.accel_percent;

            PendingDispatch pending{};
            pending.axis_id = AxisId{static_cast<std::uint16_t>(axis_key)};
            pending.axis = slot.axis;
            pending.point = point;
            pending.command = cmd;
            commands.push_back(std::move(pending));
        }
    }

    std::vector<std::pair<AxisId, MotionPoint>> failed_points;
    failed_points.reserve(commands.size());

    for (auto& pending : commands) {
        if (!pending.axis) {
            continue;
        }

        const auto result = pending.axis->apply_command(pending.command);
        if (!result.ok()) {
            failed_points.emplace_back(pending.axis_id, pending.point);
        }
    }

    if (failed_points.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [axis_id, point] : failed_points) {
        auto* slot = find_slot_ptr_locked(axis_id);
        if (!slot) {
            continue;
        }

        if (slot->queue.size() >= slot->queue_options.capacity) {
            if (slot->queue_options.policy == QueuePolicy::DropNewest) {
                ++slot->stats.dropped;
                slot->stats.size = slot->queue.size();
                continue;
            }

            if (slot->queue_options.policy == QueuePolicy::ReplaceLatest) {
                slot->queue.clear();
            } else if (!slot->queue.empty()) {
                // Preserve the failed in-flight point and evict a newer queued point if needed.
                slot->queue.pop_back();
                ++slot->stats.dropped;
            }
        }

        slot->queue.push_front(point);
        slot->stats.size = slot->queue.size();
        slot->stats.capacity = slot->queue_options.capacity;
    }
}

} // namespace motion_core
