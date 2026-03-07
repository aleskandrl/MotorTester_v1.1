#include "mks/manager/mks_can_bus_manager.h"

#include "mks/port/can_port_interface.h"
#include "mks/port/gs_usb_can_port.h"
#include "mks/protocol/mks_protocol.h"
#include "mks/port/sim_can_port.h"

#include <algorithm>
#include <chrono>

namespace {

class SyncTransactionGuard {
public:
    explicit SyncTransactionGuard(std::atomic<bool>& flag) : flag_(flag) {
        flag_.store(true, std::memory_order_release);
    }

    ~SyncTransactionGuard() {
        flag_.store(false, std::memory_order_release);
    }

    SyncTransactionGuard(const SyncTransactionGuard&) = delete;
    SyncTransactionGuard& operator=(const SyncTransactionGuard&) = delete;

private:
    std::atomic<bool>& flag_;
};

bool validate_response_crc(const std::uint16_t can_id, const mks::CanFrame& frame) {
    std::vector<std::uint8_t> bytes_without_crc;
    bytes_without_crc.reserve(frame.dlc - 1U);
    for (std::uint8_t i = 0; i + 1U < frame.dlc; ++i) {
        bytes_without_crc.push_back(frame.data[i]);
    }

    const auto expected_crc = mks::MksProtocol::computeCrc(can_id, bytes_without_crc);
    const auto actual_crc = frame.data[frame.dlc - 1U];
    return expected_crc == actual_crc;
}

} // namespace

namespace mks {

MksCanBusManager::MksCanBusManager(MksCanBusConfig config)
    : config_(std::move(config)) {}

MksCanBusManager::~MksCanBusManager() {
    (void)stop();
}

motion_core::Result<void> MksCanBusManager::start() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (started_) return motion_core::Result<void>::success();
    }

    last_stats_time_ = std::chrono::steady_clock::now();
    cycles_since_last_stats_ = 0;
    tx_bits_last_ = 0;
    rx_bits_last_ = 0;
    current_stats_ = BusStatistics{};

    if (config_.device_path.empty()) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "device_path is empty"});
    }

    std::unique_ptr<ICanPort> can_port;
    if (config_.simulated || config_.device_path.rfind("sim", 0) == 0) {
        can_port = std::make_unique<SimCanPort>();
    } else {
        can_port = std::make_unique<GsUsbCanPort>();
    }
    
    if (!can_port->open(config_.device_path.c_str(), config_.baud_rate)) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "failed to open CAN port"});
    }

    auto protocol = std::make_unique<MksProtocol>(*can_port);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        can_port_ = std::move(can_port);
        protocol_ = std::move(protocol);
        started_ = true;
    }

    const auto loop_result = runtime_loop_.start(config_.cycle_time, [this]() { poll_cycle(); });
    if (!loop_result.ok()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        started_ = false;
        protocol_.reset();
        if (can_port_) { can_port_->close(); can_port_.reset(); }
        return loop_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::stop() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_) return motion_core::Result<void>::success();
        started_ = false;
    }

    (void)runtime_loop_.stop();

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    protocol_.reset();
    if (can_port_) {
        can_port_->close();
        can_port_.reset();
    }
    rr_index_ = 0;

    {
        std::lock_guard<std::mutex> q_lock(queue_mutex_);
        high_priority_command_queue_ = {};
        command_queue_ = {};
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::register_axis(const std::uint16_t can_id) {
    if (can_id == 0) return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "can_id > 0"});

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_started_locked()) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "not started"});
    if (axes_.find(can_id) != axes_.end()) return motion_core::Result<void>::failure({motion_core::ErrorCode::AlreadyExists, "exists"});

    axes_.emplace(can_id, AxisRuntimeData{});
    axis_order_.push_back(can_id);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::unregister_axis(const std::uint16_t can_id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto axis_it = axes_.find(can_id);
    if (axis_it == axes_.end()) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotFound, "not registered"});

    axes_.erase(axis_it);
    axis_order_.erase(std::remove(axis_order_.begin(), axis_order_.end(), can_id), axis_order_.end());
    if (rr_index_ >= axis_order_.size()) rr_index_ = 0;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::send_raw_command(const std::uint16_t can_id,
                                                             const std::uint8_t cmd_byte,
                                                             const std::vector<std::uint8_t>& data) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!is_started_locked()) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "not started"});
    if (axes_.find(can_id) == axes_.end()) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotFound, "not registered"});

    std::lock_guard<std::mutex> q_lock(queue_mutex_);
    const bool is_emergency_stop = (cmd_byte == static_cast<std::uint8_t>(MksCommand::EmergencyStop));
    const std::size_t total_queue_depth = command_queue_.size() + high_priority_command_queue_.size();
    if (total_queue_depth >= kMaxCommandQueueDepth) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::Busy, "command queue is full"});
    }

    if (is_emergency_stop) {
        // Drop pending commands for that axis to guarantee E-STOP priority on the bus.
        std::queue<CmdRawData> filtered;
        while (!command_queue_.empty()) {
            auto cmd = std::move(command_queue_.front());
            command_queue_.pop();
            if (cmd.can_id != can_id) {
                filtered.push(std::move(cmd));
            }
        }
        command_queue_ = std::move(filtered);
        high_priority_command_queue_.push(CmdRawData{can_id, cmd_byte, data});
    } else {
        command_queue_.push(CmdRawData{can_id, cmd_byte, data});
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::execute_raw_command_sync(const std::uint16_t can_id,
                                                                                            const std::uint8_t cmd_byte,
                                                                                            const std::vector<std::uint8_t>& data) {
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!is_started_locked()) return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::NotConnected, "not started"});
        if (axes_.find(can_id) == axes_.end()) return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::NotFound, "not registered"});
    }

    return execute_sync_transaction(can_id, cmd_byte, data, cmd_byte, 100U);
}

motion_core::Result<MksAxisTelemetryRaw> MksCanBusManager::read_axis_telemetry(const std::uint16_t can_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto axis_it = axes_.find(can_id);
    if (axis_it == axes_.end()) return motion_core::Result<MksAxisTelemetryRaw>::failure({motion_core::ErrorCode::NotFound, "not registered"});
    return motion_core::Result<MksAxisTelemetryRaw>::success(axis_it->second.telemetry);
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::read_system_parameter(std::uint16_t can_id, std::uint8_t parameter_cmd) {
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!is_started_locked()) return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::NotConnected, "not started"});
        if (axes_.find(can_id) == axes_.end()) return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::NotFound, "not registered"});
    }

    return execute_sync_transaction(can_id, 0x00U, {parameter_cmd}, parameter_cmd, 100U);
}

bool MksCanBusManager::is_started_locked() const noexcept {
    return started_ && can_port_ && protocol_;
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::execute_sync_transaction(
    const std::uint16_t can_id,
    const std::uint8_t request_cmd,
    const std::vector<std::uint8_t>& payload,
    const std::uint8_t expected_response_cmd,
    const unsigned int timeout_ms) {
    std::lock_guard<std::mutex> sync_lock(sync_mutex_);
    SyncTransactionGuard sync_guard(sync_transaction_active_);

    std::vector<std::uint8_t> tx_bytes;
    tx_bytes.reserve(1 + payload.size() + 1);
    tx_bytes.push_back(request_cmd);
    tx_bytes.insert(tx_bytes.end(), payload.begin(), payload.end());
    tx_bytes.push_back(MksProtocol::computeCrc(can_id, tx_bytes));
    if (tx_bytes.size() > 8U) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::InvalidArgument, "sync raw command payload exceeds CAN frame size"});
    }

    CanFrame tx{};
    tx.id = can_id;
    tx.dlc = static_cast<std::uint8_t>(tx_bytes.size());
    std::copy(tx_bytes.begin(), tx_bytes.end(), tx.data);

    {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        if (!can_port_ || !can_port_->isOpen()) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::NotConnected, "CAN port is not open"});
        }
        if (!can_port_->write(tx)) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "failed to write sync raw command"});
        }
    }

    const auto started = std::chrono::steady_clock::now();
    constexpr unsigned int kReadStepTimeoutMs = 1U;

    while (true) {
        const auto elapsed_ms = static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started).count());
        if (elapsed_ms >= timeout_ms) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "timeout waiting for sync raw command response"});
        }

        CanFrame rx{};
        bool got_frame = false;
        {
            std::lock_guard<std::mutex> io_lock(io_mutex_);
            if (!can_port_ || !can_port_->isOpen()) {
                return motion_core::Result<std::vector<std::uint8_t>>::failure(
                    {motion_core::ErrorCode::NotConnected, "CAN port closed during sync raw command"});
            }

            const unsigned int time_left_ms = timeout_ms - elapsed_ms;
            got_frame = can_port_->read(rx, std::min<unsigned int>(time_left_ms, kReadStepTimeoutMs));
        }

        if (!got_frame) {
            continue;
        }
        if (rx.id != can_id || rx.dlc < 3U || rx.dlc > 8U) {
            continue;
        }
        if (rx.data[0] != expected_response_cmd) {
            continue;
        }
        if (!validate_response_crc(can_id, rx)) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "CRC mismatch in sync raw command response"});
        }

        std::vector<std::uint8_t> response;
        response.assign(rx.data + 1, rx.data + (rx.dlc - 1U));
        return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(response));
    }
}

MksCanBusManager::BusStatistics MksCanBusManager::get_bus_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return current_stats_;
}

void MksCanBusManager::poll_cycle() {
    if (sync_transaction_active_.load(std::memory_order_acquire)) {
        // Isolate sync parameter transaction from RT poll I/O path.
        // Runtime loop keeps ticking, but CAN I/O is delegated to sync path until it completes.
        return;
    }

    auto now = std::chrono::steady_clock::now();

    // Calculate stats every 1 second
    if (auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time_).count(); elapsed >= 1000) {
        std::lock_guard<std::mutex> stat_lock(stats_mutex_);
        
        current_stats_.cycle_rate_hz = static_cast<double>(cycles_since_last_stats_) * 1000.0 / static_cast<double>(elapsed);
        
        if (protocol_) {
            const auto& io = protocol_->ioStats();
            uint64_t tx_diff = io.tx_bits - tx_bits_last_;
            uint64_t rx_diff = io.rx_bits - rx_bits_last_;
            
            double total_bits_per_sec = static_cast<double>(tx_diff + rx_diff) * 1000.0 / static_cast<double>(elapsed);
            if (config_.baud_rate > 0) {
                current_stats_.bus_load_percent = (total_bits_per_sec / static_cast<double>(config_.baud_rate)) * 100.0;
            } else {
                current_stats_.bus_load_percent = 0.0;
            }

            tx_bits_last_ = io.tx_bits;
            rx_bits_last_ = io.rx_bits;
        }

        cycles_since_last_stats_ = 0;
        last_stats_time_ = now;
    }
    ++cycles_since_last_stats_;

    std::uint16_t can_id = 0;
    bool fetch_telemetry = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_started_locked() || axis_order_.empty()) return;

        ++telemetry_tick_;
        fetch_telemetry = !sync_transaction_active_.load(std::memory_order_acquire);
        if (fetch_telemetry) {
            if (rr_index_ >= axis_order_.size()) rr_index_ = 0;
            can_id = axis_order_[rr_index_];
            rr_index_ = (rr_index_ + 1) % axis_order_.size();
        }
    }

    MksAxisTelemetryRaw snapshot{};
    bool fetched_telemetry = false;
    
    {
        std::unique_lock<std::mutex> io_lock(io_mutex_, std::try_to_lock);
        if (!io_lock.owns_lock() || !protocol_) return;

        // 1. FLUSH OUTGOING COMMANDS WITH BUDGET (bounded per cycle)
        std::queue<CmdRawData> high_priority_pending;
        std::queue<CmdRawData> pending;
        {
            std::lock_guard<std::mutex> q_lock(queue_mutex_);
            std::swap(high_priority_pending, high_priority_command_queue_);
            std::swap(pending, command_queue_);
        }

        std::queue<CmdRawData> deferred;
        std::size_t commands_sent_this_cycle = 0;

        // Always drain emergency-stop commands first (safety path, no budget gating).
        while (!high_priority_pending.empty()) {
            const auto cmd = high_priority_pending.front();
            high_priority_pending.pop();

            std::vector<std::uint8_t> response;
            (void)protocol_->sendCommand(cmd.can_id,
                                         static_cast<MksCommand>(cmd.cmd_byte),
                                         cmd.payload,
                                         response,
                                         0xFF,
                                         0,
                                         false);
        }

        while (!pending.empty()) {
            const auto cmd = pending.front();
            pending.pop();

            if (commands_sent_this_cycle >= kMaxCommandsPerCycle) {
                deferred.push(cmd);
                continue;
            }

            std::vector<std::uint8_t> response;
            // Send fire-and-forget
            (void)protocol_->sendCommand(cmd.can_id, static_cast<MksCommand>(cmd.cmd_byte), cmd.payload, response, 0xFF, 0, false);
            ++commands_sent_this_cycle;
        }

        if (!deferred.empty()) {
            std::lock_guard<std::mutex> q_lock(queue_mutex_);
            while (!deferred.empty() && command_queue_.size() < kMaxCommandQueueDepth) {
                command_queue_.push(std::move(deferred.front()));
                deferred.pop();
            }
        }

        // 2. Telemetry loop: try one axis per cycle with tight time budget.
        if (fetch_telemetry) {
            fetched_telemetry = true;
            std::vector<std::uint8_t> payload;
            constexpr unsigned int kTelemetryCommandTimeoutMs = 2;

            auto loop_start = std::chrono::steady_clock::now();
            auto has_budget = [&]() {
                return std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - loop_start)
                           .count() < 2;
            };

            // Read Motor Status
            if (has_budget() && protocol_->sendCommand(can_id,
                                      MksCommand::QueryMotorStatus,
                                      {},
                                      payload,
                                      static_cast<uint8_t>(MksCommand::QueryMotorStatus),
                                      kTelemetryCommandTimeoutMs,
                                      true) &&
                !payload.empty()) {
                snapshot.has_motor_status = true;
                snapshot.motor_status = payload.back();
            }

            // Read Motor Speed
            payload.clear();
            if (has_budget() && protocol_->sendCommand(can_id,
                                      MksCommand::ReadMotorSpeed,
                                      {},
                                      payload,
                                      static_cast<uint8_t>(MksCommand::ReadMotorSpeed),
                                      kTelemetryCommandTimeoutMs,
                                      true) &&
                payload.size() >= 2) {
                snapshot.has_speed_rpm = true;
                snapshot.speed_rpm = static_cast<std::int16_t>(MksProtocol::readBe16(payload.data()));
            }

            // Read Axis Position
            payload.clear();
            if (has_budget() && protocol_->sendCommand(can_id,
                                      MksCommand::ReadEncoderAddition,
                                      {},
                                      payload,
                                      static_cast<uint8_t>(MksCommand::ReadEncoderAddition),
                                      kTelemetryCommandTimeoutMs,
                                      true)) {
                if (payload.size() >= 6) {
                    snapshot.has_axis_position = true;
                    snapshot.axis_position = MksProtocol::readBe48s(payload.data());
                } else if (payload.size() == 4) {
                    snapshot.has_axis_position = true;
                    snapshot.axis_position = MksProtocol::readBe32s(payload.data());
                }
            }

            // Read Protection State
            payload.clear();
            if (has_budget() && protocol_->sendCommand(can_id,
                                      MksCommand::ReadProtectionState,
                                      {},
                                      payload,
                                      static_cast<uint8_t>(MksCommand::ReadProtectionState),
                                      kTelemetryCommandTimeoutMs,
                                      true) &&
                !payload.empty()) {
                snapshot.has_protection_state = true;
                snapshot.protection_state = payload.back();
            }
        }
    }

    if (!fetched_telemetry) return;

    snapshot.timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto axis_it = axes_.find(can_id);
    if (axis_it == axes_.end()) return;

    // Use a temporary copy to avoid tearing if read_axis_telemetry reads while we modify
    auto telemetry_copy = axis_it->second.telemetry;
    if (snapshot.has_axis_position) { telemetry_copy.has_axis_position = true; telemetry_copy.axis_position = snapshot.axis_position; }
    if (snapshot.has_speed_rpm) { telemetry_copy.has_speed_rpm = true; telemetry_copy.speed_rpm = snapshot.speed_rpm; }
    if (snapshot.has_protection_state) { telemetry_copy.has_protection_state = true; telemetry_copy.protection_state = snapshot.protection_state; }
    if (snapshot.has_motor_status) { telemetry_copy.has_motor_status = true; telemetry_copy.motor_status = snapshot.motor_status; }
    telemetry_copy.timestamp_ns = snapshot.timestamp_ns;
    
    axis_it->second.telemetry = telemetry_copy;
}

std::shared_ptr<MksCanBusManager> make_mks_can_bus_manager(MksCanBusConfig config) {
    return std::make_shared<MksCanBusManager>(std::move(config));
}

} // namespace mks
