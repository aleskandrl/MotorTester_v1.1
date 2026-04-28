#pragma once

#include "motion_core/hal_runtime.h"

#include <cstdint>
#include <functional>

namespace hal_host_service {

enum class MotionControlSource {
    Ui = 0,
    HexaMotion = 1,
};

struct RuntimeQueueIngressState {
    MotionControlSource control_source{MotionControlSource::Ui};
    bool hexamotion_connected{false};
    bool estop_active{false};
};

class RuntimeQueueIngress final {
public:
    using StateProvider = std::function<RuntimeQueueIngressState()>;

    RuntimeQueueIngress(motion_core::HalRuntime& runtime, StateProvider state_provider)
        : runtime_(runtime), state_provider_(std::move(state_provider)) {
    }

    [[nodiscard]] motion_core::Result<void> enqueueCommandPoint(
        const MotionControlSource source,
        const std::uint16_t axis_id,
        const motion_core::MotionCommandPoint& point) const {
        const auto access_result = authorize_motion(source);
        if (!access_result.ok()) {
            return motion_core::Result<void>::failure(access_result.error());
        }

        const auto axis_result = runtime_.find_axis(axis_id);
        if (!axis_result.ok()) {
            return motion_core::Result<void>::failure(axis_result.error());
        }

        // Fail-safe policy:
        // Do not hard-reject by telemetry mode here. Mode switching is asynchronous for EtherCAT,
        // and telemetry mode may lag behind requested mode for multiple cycles.
        // Rejection at ingress caused UI-side command floods and pseudo-freezes under CSP/PP transitions.
        // Axis adapter keeps strict execution gating at RT cycle and processes queued points when mode is ready.
        if (!axis_result.value()->enqueueCommandPoint(point)) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::Busy, "failed to enqueue motion command point"});
        }
        return motion_core::Result<void>::success();
    }

    [[nodiscard]] motion_core::Result<void> enqueueServicePoint(
        const MotionControlSource source,
        const std::uint16_t axis_id,
        const motion_core::ServiceCommandPoint& point) const {
        (void)source;

        const auto axis_result = runtime_.find_axis(axis_id);
        if (!axis_result.ok()) {
            return motion_core::Result<void>::failure(axis_result.error());
        }
        if (!axis_result.value()->enqueueServicePoint(point)) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::Busy, "failed to enqueue service command point"});
        }
        return motion_core::Result<void>::success();
    }

private:
    [[nodiscard]] motion_core::Result<void> authorize_motion(const MotionControlSource source) const {
        if (!state_provider_) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::InternalError, "runtime queue ingress state provider is not configured"});
        }

        const RuntimeQueueIngressState state = state_provider_();
        if (state.estop_active) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "global estop is active"});
        }

        if (source == MotionControlSource::Ui) {
            if (state.control_source != MotionControlSource::Ui) {
                return motion_core::Result<void>::failure(
                    {motion_core::ErrorCode::Busy, "ui motion source is not active"});
            }
            return motion_core::Result<void>::success();
        }

        if (state.control_source != MotionControlSource::HexaMotion) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Busy, "hexamotion motion source is not active"});
        }
        if (!state.hexamotion_connected) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::NotConnected, "hexamotion is not connected"});
        }
        return motion_core::Result<void>::success();
    }

    motion_core::HalRuntime& runtime_;
    StateProvider state_provider_{};
};

} // namespace hal_host_service