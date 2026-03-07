#pragma once

#include "motion_core/result.h"
#include "motion_core/runtime_loop.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>
#include <string>

namespace mks {

class ICanPort;
class MksProtocol;

struct MksCanBusConfig {
    std::string device_path{};
    unsigned int baud_rate{1'000'000U};
    std::chrono::milliseconds cycle_time{4};
    bool simulated{false};
};

struct MksAxisTelemetryRaw {
    bool has_axis_position{false};
    std::int64_t axis_position{0};

    bool has_speed_rpm{false};
    std::int16_t speed_rpm{0};

    bool has_protection_state{false};
    std::uint8_t protection_state{0};

    bool has_motor_status{false};
    std::uint8_t motor_status{0};

    std::uint64_t timestamp_ns{0};
};

struct CmdRawData {
    std::uint16_t can_id;
    std::uint8_t cmd_byte;
    std::vector<std::uint8_t> payload;
};

class MksCanBusManager {
public:
    static constexpr std::size_t kMaxCommandQueueDepth = 512;
    static constexpr std::size_t kMaxCommandsPerCycle = 16;

    explicit MksCanBusManager(MksCanBusConfig config);
    ~MksCanBusManager();

    MksCanBusManager(const MksCanBusManager&) = delete;
    MksCanBusManager& operator=(const MksCanBusManager&) = delete;

    motion_core::Result<void> start();
    motion_core::Result<void> stop();

    motion_core::Result<void> register_axis(std::uint16_t can_id);
    motion_core::Result<void> unregister_axis(std::uint16_t can_id);

    // Completely generalized dispatch (Asynchronous)
    motion_core::Result<void> send_raw_command(std::uint16_t can_id,
                                               std::uint8_t cmd_byte,
                                               const std::vector<std::uint8_t>& data);

    // Synchronous execute, used for safe parameter writing where we must wait for a response
    motion_core::Result<std::vector<std::uint8_t>> execute_raw_command_sync(
        std::uint16_t can_id, std::uint8_t cmd_byte, const std::vector<std::uint8_t>& data);

    // Synchronous read for parameters without specific motor structs
    [[nodiscard]] motion_core::Result<std::vector<std::uint8_t>> read_system_parameter(std::uint16_t can_id, std::uint8_t parameter_cmd);

    // Read async telemetry state
    [[nodiscard]] motion_core::Result<MksAxisTelemetryRaw> read_axis_telemetry(std::uint16_t can_id) const;

    struct BusStatistics {
        double cycle_rate_hz{0.0};
        double bus_load_percent{0.0};
    };

    [[nodiscard]] BusStatistics get_bus_statistics() const;

private:
    struct AxisRuntimeData {
        MksAxisTelemetryRaw telemetry{};
    };

    [[nodiscard]] bool is_started_locked() const noexcept;
    [[nodiscard]] motion_core::Result<std::vector<std::uint8_t>> execute_sync_transaction(
        std::uint16_t can_id,
        std::uint8_t request_cmd,
        const std::vector<std::uint8_t>& payload,
        std::uint8_t expected_response_cmd,
        unsigned int timeout_ms);
    void poll_cycle();

    MksCanBusConfig config_{};

    mutable std::mutex state_mutex_;
    mutable std::mutex io_mutex_;
    mutable std::mutex sync_mutex_;

    bool started_{false};
    std::unordered_map<std::uint16_t, AxisRuntimeData> axes_;
    std::vector<std::uint16_t> axis_order_;
    std::size_t rr_index_{0};
    std::uint32_t telemetry_tick_{0};
    std::atomic<bool> sync_transaction_active_{false};

    mutable std::mutex queue_mutex_;
    std::queue<CmdRawData> high_priority_command_queue_;
    std::queue<CmdRawData> command_queue_;

    std::unique_ptr<ICanPort> can_port_;
    std::unique_ptr<MksProtocol> protocol_;
    motion_core::RuntimeLoop runtime_loop_;

    // Statistics
    std::chrono::steady_clock::time_point last_stats_time_;
    std::uint64_t cycles_since_last_stats_{0};
    std::uint64_t tx_bits_last_{0};
    std::uint64_t rx_bits_last_{0};
    
    mutable std::mutex stats_mutex_;
    BusStatistics current_stats_{};
};

[[nodiscard]] std::shared_ptr<MksCanBusManager> make_mks_can_bus_manager(MksCanBusConfig config);

} // namespace mks
