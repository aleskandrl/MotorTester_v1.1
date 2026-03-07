#pragma once

#include "mks/manager/mks_can_bus_manager.h"
#include "motion_core/axis_interface.h"

#include <memory>
#include <mutex>
#include <string>

namespace mks {

struct MksAxisAdapterConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    std::uint16_t can_id{1};
    std::shared_ptr<MksCanBusManager> bus_manager{};
    bool auto_start_bus_manager{true};

    // Conversion between human angle and motor axis units.
    double axis_units_per_degree{16384.0 / 360.0};

    // Default motion profile used for absolute position commands.
    std::uint16_t default_speed{300};
    std::uint8_t default_accel{10};
};

class MksAxisAdapter final : public motion_core::IAxis {
public:
    explicit MksAxisAdapter(MksAxisAdapterConfig config);
    ~MksAxisAdapter() override;

    [[nodiscard]] motion_core::AxisInfo info() const override;

    motion_core::Result<void> start() override;
    motion_core::Result<void> stop() override;

    motion_core::Result<void> set_enabled(bool enabled) override;
    motion_core::Result<void> set_mode(motion_core::AxisMode mode) override;
    motion_core::Result<void> apply_command(const motion_core::AxisCommand& command) override;

    [[nodiscard]] motion_core::Result<motion_core::AxisTelemetry> read_telemetry() const override;

    [[nodiscard]] motion_core::Result<std::vector<motion_core::ParameterDescriptor>> list_parameters() const override;
    [[nodiscard]] motion_core::Result<motion_core::ParameterSet> read_parameters() const override;
    motion_core::Result<void> apply_parameter_patch(const motion_core::ParameterPatch& patch) override;

private:
    motion_core::Result<void> ensure_started() const;
    motion_core::Result<void> set_enabled_locked(bool enabled);
    void recalculate_axis_units_per_degree_from_software_params();

    [[nodiscard]] int mode_to_work_mode(motion_core::AxisMode mode) const;

    MksAxisAdapterConfig config_{};

    mutable std::mutex mutex_;
    bool started_{false};
    bool enabled_{false};
    motion_core::AxisMode mode_{motion_core::AxisMode::ProfilePosition};
    motion_core::AxisTelemetry telemetry_{};
    std::int32_t target_axis_units_{0};
    double software_gear_ratio_{1.0};
    std::uint32_t software_encoder_resolution_bits_{14};
};

[[nodiscard]] std::shared_ptr<motion_core::IAxis> make_mks_axis_adapter(MksAxisAdapterConfig config);

} // namespace mks
