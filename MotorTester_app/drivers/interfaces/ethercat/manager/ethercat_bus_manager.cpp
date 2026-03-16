#include "ethercat/manager/ethercat_bus_manager.h"
#include "ethercat/adapter/ethercat_axis_adapter.h"

#include <algorithm>
#include <iostream>

namespace ethercat_driver {

namespace {

constexpr uint32_t kSupportedVendorId = 0x00445653;
constexpr uint32_t kSupportedProductCode = 0x00009252;

} // namespace

EthercatBusManager::EthercatBusManager(EthercatBusConfig config)
    : config_(std::move(config)) {}

EthercatBusManager::~EthercatBusManager() {
    (void)close();
}

motion_core::Result<void> EthercatBusManager::open() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (master_) {
        return motion_core::Result<void>::success();
    }

    master_ = ecrt_request_master(0);
    if (!master_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to request EtherCAT master 0"});
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        ecrt_release_master(master_);
        master_ = nullptr;
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to create EtherCAT domain"});
    }

    discovered_slaves_.clear();
    axis_map_.clear();
    registered_adapters_.clear();

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatBusManager::close() {
    (void)stop();

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
        domain_ = nullptr;
    }
    discovered_slaves_.clear();
    axis_map_.clear();
    registered_adapters_.clear();

    return motion_core::Result<void>::success();
}

bool EthercatBusManager::is_open() const noexcept {
    return master_ != nullptr;
}

motion_core::Result<std::vector<std::uint16_t>> EthercatBusManager::scan_axes() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!master_) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::NotConnected, "EtherCAT master is not open"});
    }

    ec_master_info_t master_info{};
    if (ecrt_master(master_, &master_info)) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get master info"});
    }

    discovered_slaves_.clear();
    axis_map_.clear();
    std::vector<std::uint16_t> out;

    int runtime_index = 0;
    for (uint16_t i = 0; i < master_info.slave_count; ++i) {
        ec_slave_info_t slave_info{};
        if (ecrt_master_get_slave(master_, i, &slave_info) != 0) continue;
        if (slave_info.vendor_id != kSupportedVendorId ||
            slave_info.product_code != kSupportedProductCode) {
            continue;
        }

        SlaveInfo info;
        info.axis_id = i + 1; // Assuming sequential ID starting from 1
        info.bus_position = i;
        info.vendor_id = slave_info.vendor_id;
        info.product_code = slave_info.product_code;
        discovered_slaves_.push_back(info);

        SlaveBusInfo bus_info;
        bus_info.runtime_index = runtime_index;
        bus_info.bus_position = info.bus_position;
        axis_map_[info.axis_id] = bus_info;
        
        out.push_back(info.axis_id);
        ++runtime_index;
    }

    return motion_core::Result<std::vector<std::uint16_t>>::success(std::move(out));
}

motion_core::Result<motion_core::HalBusConfigEthercat> EthercatBusManager::discover_ethercat_topology(
    const std::string& interface_name) {
    if (interface_name.empty()) {
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::InvalidArgument, "interface_name is empty"});
    }

    ec_master_t* master = ecrt_request_master(0);
    if (!master) {
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to request EtherCAT master 0 for scanning"});
    }

    ec_master_info_t master_info{};
    if (ecrt_master(master, &master_info)) {
        ecrt_release_master(master);
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get master info"});
    }

    motion_core::HalBusConfigEthercat bus_cfg{};
    bus_cfg.interface_name = interface_name;

    for (uint16_t i = 0; i < master_info.slave_count; ++i) {
        ec_slave_info_t slave_info{};
        if (ecrt_master_get_slave(master, i, &slave_info) != 0) continue;
        if (slave_info.vendor_id != kSupportedVendorId ||
            slave_info.product_code != kSupportedProductCode) {
            continue;
        }

        uint16_t axis_id = i + 1; // Basic sequential ID mapping
        motion_core::HalAxisRuntimeEntry entry{};
        entry.axis_id = motion_core::AxisId{axis_id};
        entry.axis_name.value = "ECAT " + std::to_string(axis_id);
        entry.transport = motion_core::AxisTransportKind::Ethercat;
        entry.bus_ref = interface_name;
        entry.transport_address = axis_id;
        entry.enable_on_start = false;
        // The frontend loop will merge these discovered axes into a HalRuntimeConfig

        bus_cfg.axes.push_back(std::move(entry));
    }

    ecrt_release_master(master);
    return motion_core::Result<motion_core::HalBusConfigEthercat>::success(std::move(bus_cfg));
}

motion_core::Result<SlaveBusInfo> EthercatBusManager::get_slave_bus_info(const std::uint16_t axis_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = axis_map_.find(axis_id);
    if (it == axis_map_.end()) {
        return motion_core::Result<SlaveBusInfo>::failure(
            {motion_core::ErrorCode::NotFound, "Axis ID not found on EtherCAT bus"});
    }
    return motion_core::Result<SlaveBusInfo>::success(it->second);
}

motion_core::Result<motion_core::BusStatistics> EthercatBusManager::get_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return motion_core::Result<motion_core::BusStatistics>::success(current_stats_);
}

void EthercatBusManager::register_adapter(const int axis_index, EthercatAxisAdapter* adapter) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (static_cast<std::size_t>(axis_index) >= registered_adapters_.size()) {
        registered_adapters_.resize(axis_index + 1, nullptr);
    }
    registered_adapters_[axis_index] = adapter;
}

motion_core::Result<void> EthercatBusManager::start() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (started_) {
        return motion_core::Result<void>::success();
    }
    if (!master_ || !domain_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "Master or domain not initialized"});
    }

    if (ecrt_master_activate(master_)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to activate master"});
    }

    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get domain data pointer"});
    }

    started_ = true;
    last_stats_time_ = std::chrono::steady_clock::now();
    cycles_since_last_stats_ = 0;
    accrued_cycle_time_us_ = 0;

    const auto loop_result = runtime_loop_.start(config_.cycle_time, [this]() { poll_cycle(); });
    if (!loop_result.ok()) {
        started_ = false;
        return loop_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatBusManager::stop() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_) return motion_core::Result<void>::success();
        started_ = false;
    }

    (void)runtime_loop_.stop();
    return motion_core::Result<void>::success();
}

void EthercatBusManager::poll_cycle() {
    if (!started_ || !master_ || !domain_) return;

    auto cycle_start = std::chrono::steady_clock::now();

    // Calculate dt_s for speed calculation
    double dt_s = 0.0;
    if (cycles_since_last_stats_ > 0) { // skip first cycle dt calculation
        dt_s = std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_start - last_cycle_time_).count() * 1e-9;
    }
    last_cycle_time_ = cycle_start;

    if (auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_start - last_stats_time_).count(); elapsed >= 1000) {
        std::lock_guard<std::mutex> stat_lock(stats_mutex_);
        current_stats_.cycle_rate_hz = static_cast<double>(cycles_since_last_stats_) * 1000.0 / static_cast<double>(elapsed);
        current_stats_.bus_load_percent = (static_cast<double>(accrued_cycle_time_us_) / 1000.0) / static_cast<double>(elapsed) * 100.0;

        cycles_since_last_stats_ = 0;
        accrued_cycle_time_us_ = 0;
        last_stats_time_ = cycle_start;
    }
    ++cycles_since_last_stats_;

    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);

    // Safe copy of adapters array for process cycle to avoid any locks in fast path
    std::vector<EthercatAxisAdapter*> adapters_to_process;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (auto* adapter : registered_adapters_) {
            if (adapter) {
                adapters_to_process.push_back(adapter);
            }
        }
    }

    for (auto* adapter : adapters_to_process) {
        adapter->process_cycle(domain_pd_, dt_s);
    }

    ecrt_domain_queue(domain_);
    ecrt_master_send(master_);

    auto cycle_end = std::chrono::steady_clock::now();
    accrued_cycle_time_us_ += std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
}

std::shared_ptr<EthercatBusManager> make_ethercat_bus_manager(EthercatBusConfig config) {
    return std::make_shared<EthercatBusManager>(std::move(config));
}

} // namespace ethercat_driver
