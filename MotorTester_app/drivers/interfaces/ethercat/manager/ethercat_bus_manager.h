#pragma once

#include "motion_core/result.h"
#include "motion_core/runtime_loop.h"
#include "motion_core/bus_manager_interface.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>

#include "motion_core/config/hal_runtime_config.h"

#include <ecrt.h>

namespace ethercat_driver {

class EthercatAxisAdapter;

struct SlaveBusInfo {
    int runtime_index{0};
    uint16_t bus_position{0};
};

struct EthercatBusConfig {
    std::string interface_name{};
    std::chrono::milliseconds cycle_time{4}; // 250 Hz default
};

class EthercatBusManager : public motion_core::IBusManager {
public:
    explicit EthercatBusManager(EthercatBusConfig config);
    ~EthercatBusManager();

    [[nodiscard]] std::string get_name() const override { return "EtherCAT " + config_.interface_name; }

    motion_core::Result<void> open();
    motion_core::Result<void> close();

    [[nodiscard]] bool is_open() const noexcept;

    [[nodiscard]] motion_core::Result<std::vector<std::uint16_t>> scan_axes();

    [[nodiscard]] motion_core::Result<motion_core::BusStatistics> get_statistics() const override;

    motion_core::Result<void> start() override;
    motion_core::Result<void> stop() override;

    [[nodiscard]] motion_core::Result<SlaveBusInfo> get_slave_bus_info(std::uint16_t axis_id) const;

    [[nodiscard]] ec_master_t* master() const { return master_; }
    [[nodiscard]] ec_domain_t* domain() const { return domain_; }

    void register_adapter(int axis_index, EthercatAxisAdapter* adapter);

    // Headless discovery that does not hold the master open
    static motion_core::Result<motion_core::HalBusConfigEthercat> discover_ethercat_topology(
        const std::string& interface_name);

private:
    void poll_cycle();

    EthercatBusConfig config_{};
    
    ec_master_t* master_{nullptr};
    ec_domain_t* domain_{nullptr};
    uint8_t* domain_pd_{nullptr};
    
    struct SlaveInfo {
        uint16_t axis_id{0};
        uint16_t bus_position{0};
        uint32_t vendor_id{0};
        uint32_t product_code{0};
    };
    std::vector<SlaveInfo> discovered_slaves_{};
    std::unordered_map<std::uint16_t, SlaveBusInfo> axis_map_{};

    mutable std::mutex state_mutex_;
    std::atomic<bool> started_{false};
    
    motion_core::RuntimeLoop runtime_loop_;
    
    std::vector<EthercatAxisAdapter*> registered_adapters_{};

    motion_core::BusStatistics current_stats_{};
    mutable std::mutex stats_mutex_;
    std::chrono::steady_clock::time_point last_stats_time_{};
    uint64_t cycles_since_last_stats_{0};
    uint64_t accrued_cycle_time_us_{0};
    std::chrono::steady_clock::time_point last_cycle_time_{};
};

[[nodiscard]] std::shared_ptr<EthercatBusManager> make_ethercat_bus_manager(EthercatBusConfig config);

} // namespace ethercat_driver
