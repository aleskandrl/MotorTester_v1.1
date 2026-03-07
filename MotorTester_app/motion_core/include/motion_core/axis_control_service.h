#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/axis_interface.h"
#include "motion_core/parameter_types.h"
#include "motion_core/config/axis_config.h"
#include "motion_core/result.h"
#include "motion_core/runtime_loop.h"
#include "motion_core/config/hal_runtime_config.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace motion_core {

enum class QueuePolicy {
    Fifo = 0,
    ReplaceLatest,
    DropNewest,
    DropOldest
};

struct MotionQueueOptions {
    std::size_t capacity{256};
    QueuePolicy policy{QueuePolicy::Fifo};
};

struct MotionPoint {
    double position_deg{0.0};
    bool relative{false};
    bool has_speed_rpm{false};
    std::uint16_t speed_rpm{0};
    bool has_accel_percent{false};
    double accel_percent{0.0};
};

struct MotionQueueStats {
    std::size_t size{0};
    std::size_t capacity{0};
    std::uint64_t pushed{0};
    std::uint64_t dropped{0};
};

struct SafetyBaselineOptions {
    bool force_disable{true};
    bool sync_target_to_actual{true};
};

class AxisControlService {
public:
    Result<void> add_axis(const std::shared_ptr<IAxis>& axis);
    Result<void> remove_axis(AxisId axis_id);
    [[nodiscard]] Result<std::vector<AxisInfo>> list_axes() const;

    Result<void> start_axis(AxisId axis_id);
    Result<void> stop_axis(AxisId axis_id);

    Result<void> enable_axis(AxisId axis_id, bool enabled);
    Result<void> set_axis_mode(AxisId axis_id, AxisMode mode);
    Result<void> submit_command(AxisId axis_id, const AxisCommand& command);

    Result<void> move_absolute(AxisId axis_id,
                               double position_deg,
                               bool has_speed_rpm = false,
                               std::uint16_t speed_rpm = 0,
                               bool has_accel_percent = false,
                               double accel_percent = 0.0);

    Result<void> move_relative(AxisId axis_id,
                               double delta_deg,
                               bool has_speed_rpm = false,
                               std::uint16_t speed_rpm = 0,
                               bool has_accel_percent = false,
                               double accel_percent = 0.0);

    Result<void> emergency_stop(AxisId axis_id);
    Result<void> clear_errors(AxisId axis_id);
    Result<void> go_home(AxisId axis_id);
    Result<void> set_zero(AxisId axis_id);
    Result<void> apply_safe_baseline(AxisId axis_id, const SafetyBaselineOptions& options = {});

    Result<void> set_motion_queue_options(AxisId axis_id, const MotionQueueOptions& options);
    Result<void> enqueue_motion_point(AxisId axis_id, const MotionPoint& point);
    Result<void> enqueue_motion_batch(AxisId axis_id, const std::vector<MotionPoint>& points);
    Result<void> clear_motion_queue(AxisId axis_id);
    [[nodiscard]] Result<MotionQueueStats> get_motion_queue_stats(AxisId axis_id) const;

    Result<void> start_dispatch_loop(std::chrono::milliseconds period = std::chrono::milliseconds{4});
    Result<void> stop_dispatch_loop();
    [[nodiscard]] bool is_dispatch_loop_running() const noexcept;

    [[nodiscard]] Result<AxisTelemetry> read_telemetry(AxisId axis_id) const;
    [[nodiscard]] Result<std::vector<ParameterDescriptor>> list_parameters(AxisId axis_id) const;
    [[nodiscard]] Result<ParameterSet> read_parameters(AxisId axis_id) const;
    Result<void> apply_parameter_patch(AxisId axis_id, const ParameterPatch& patch);

    // Новые методы управления конфигурацией (Phase 4-5)
    [[nodiscard]] Result<AxisConfig> export_axis_config(AxisId axis_id) const;
    Result<void> apply_axis_config(AxisId axis_id, const AxisConfig& config);

    // Runtime Lifecycle API
    Result<void> open_runtime(const HalRuntimeConfig& config);
    Result<void> close_runtime();
    Result<void> start_runtime();
    Result<void> stop_runtime();
    [[nodiscard]] bool is_runtime_open() const noexcept;
    [[nodiscard]] bool is_runtime_active() const noexcept;

private:
    struct AxisSlot {
        std::shared_ptr<IAxis> axis;
        MotionQueueOptions queue_options{};
        std::deque<MotionPoint> queue{};
        MotionQueueStats stats{};
    };

    Result<void> enqueue_motion_point_locked(AxisSlot& slot, const MotionPoint& point);
    [[nodiscard]] Result<std::shared_ptr<IAxis>> find_axis_locked(AxisId axis_id) const;
    [[nodiscard]] AxisSlot* find_slot_ptr_locked(AxisId axis_id);
    [[nodiscard]] const AxisSlot* find_slot_ptr_locked(AxisId axis_id) const;

    void dispatch_tick();

    mutable std::mutex mutex_;
    std::unordered_map<std::uint16_t, AxisSlot> axes_;
    RuntimeLoop dispatch_loop_;
};

} // namespace motion_core
