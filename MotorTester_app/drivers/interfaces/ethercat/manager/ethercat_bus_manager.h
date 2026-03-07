#pragma once

#include "motion_core/result.h"
#include "motion_core/runtime_loop.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>

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

class EthercatBusManager {
public:
    explicit EthercatBusManager(EthercatBusConfig config);
    ~EthercatBusManager();

    motion_core::Result<void> open();
    motion_core::Result<void> close();

    [[nodiscard]] bool is_open() const noexcept;

    [[nodiscard]] motion_core::Result<std::vector<std::uint16_t>> scan_axes();

    struct BusStatistics {
        double cycle_rate_hz{0.0};
        double bus_load_percent{0.0};
    };
    [[nodiscard]] BusStatistics get_bus_statistics() const;

    motion_core::Result<void> start_runtime();
    motion_core::Result<void> stop_runtime();

    [[nodiscard]] motion_core::Result<SlaveBusInfo> get_slave_bus_info(std::uint16_t axis_id) const;

    [[nodiscard]] ec_master_t* master() const { return master_; }
    [[nodiscard]] ec_domain_t* domain() const { return domain_; }

    void register_adapter(int axis_index, EthercatAxisAdapter* adapter);

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

    BusStatistics current_stats_{};
    mutable std::mutex stats_mutex_;
    std::chrono::steady_clock::time_point last_stats_time_{};
    uint64_t cycles_since_last_stats_{0};
    uint64_t accrued_cycle_time_us_{0};
};

[[nodiscard]] std::shared_ptr<EthercatBusManager> make_ethercat_bus_manager(EthercatBusConfig config);

} // namespace ethercat_driver
