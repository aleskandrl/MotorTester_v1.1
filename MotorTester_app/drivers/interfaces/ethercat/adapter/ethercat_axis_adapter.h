#pragma once

#include "motion_core/axis_interface.h"
#include "ethercat/manager/ethercat_bus_manager.h"

#include <memory>
#include <string>
#include <vector>
#include <atomic>

namespace ethercat_driver {

struct EthercatAxisAdapterConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
    int ecat_axis_index{0};
    uint16_t ecat_bus_position{0};
    std::shared_ptr<EthercatBusManager> bus_manager{};
};

class EthercatAxisAdapter final : public motion_core::IAxis {
public:
    explicit EthercatAxisAdapter(EthercatAxisAdapterConfig config);
    ~EthercatAxisAdapter() override;

    [[nodiscard]] motion_core::AxisInfo info() const override;

    motion_core::Result<void> configure_hardware();

    motion_core::Result<void> start() override;
    motion_core::Result<void> stop() override;

    motion_core::Result<void> set_enabled(bool enabled) override;
    motion_core::Result<void> set_mode(motion_core::AxisMode mode) override;
    motion_core::Result<void> apply_command(const motion_core::AxisCommand& command) override;

    [[nodiscard]] motion_core::Result<motion_core::AxisTelemetry> read_telemetry() const override;

    [[nodiscard]] motion_core::Result<std::vector<motion_core::ParameterDescriptor>> list_parameters() const override;
    [[nodiscard]] motion_core::Result<motion_core::ParameterSet> read_parameters() const override;
    motion_core::Result<void> apply_parameter_patch(const motion_core::ParameterPatch& patch) override;

    // Called by EthercatBusManager's RT loop
    void process_cycle(uint8_t* domain_pd);

private:
    motion_core::Result<void> ensure_started() const;
    void recalculate_counts_per_radian();
    [[nodiscard]] int8_t mode_to_work_mode(motion_core::AxisMode mode) const;
    [[nodiscard]] static std::vector<motion_core::ParameterDescriptor> make_parameter_descriptors();
    
    // Command atomic state from NRT to RT
    struct AtomicCommand {
        std::atomic<bool> enable_req{false};
        std::atomic<bool> reset_req{false};
        std::atomic<int8_t> mode_req{8}; // CSP
        std::atomic<bool> emergency_stop_req{false};
        std::atomic<double> target_pos_deg{0.0};
        std::atomic<bool> quick_stop_req{false};
    };
    
    // Telemetry atomic state from RT to NRT
    struct AtomicTelemetry {
        std::atomic<uint64_t> timestamp_ns{0};
        std::atomic<std::int32_t> actual_position_counts{0};
        std::atomic<double> actual_position_deg{0.0};
        std::atomic<double> actual_velocity_deg_s{0.0};
        std::atomic<double> actual_torque_pct{0.0};
        std::atomic<uint16_t> statusword{0};
        std::atomic<uint16_t> error_code{0};
        std::atomic<bool> is_ready{false};
        std::atomic<bool> is_faulted{false};
        std::atomic<bool> is_enabled{false};
    };

    EthercatAxisAdapterConfig config_{};
    
    AtomicCommand cmd_;
    AtomicTelemetry telem_;

    bool started_{false};
    motion_core::AxisMode mode_{motion_core::AxisMode::ProfilePosition};
    
    // PDO offsets assigned by ecrt
    unsigned int off_ctrl_{0}, off_target_pos_{0}, off_modes_op_{0}, off_max_vel_{0};
    unsigned int off_status_{0}, off_act_pos_{0}, off_error_{0}, off_act_torque_{0};
    
    // Hardware params
    std::atomic<double> gear_ratio_{1.0};
    std::atomic<std::uint32_t> encoder_resolution_bits_{17};
    std::atomic<double> counts_per_radian_{1.0};
    std::atomic<std::int32_t> zero_offset_counts_{0};
    double last_position_deg_{0.0};
    std::uint64_t last_timestamp_ns_{0};

    // Control state
    uint16_t current_state_word_{0};
};

[[nodiscard]] std::shared_ptr<EthercatAxisAdapter> make_ethercat_axis_adapter(EthercatAxisAdapterConfig config);

} // namespace ethercat_driver
