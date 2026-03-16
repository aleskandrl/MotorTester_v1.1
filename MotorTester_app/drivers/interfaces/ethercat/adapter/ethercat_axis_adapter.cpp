#include "ethercat/adapter/ethercat_axis_adapter.h"
#include "ethercat/p100e_ethercat_dictionary.h"

#include <cmath>
#include <chrono>
#include <array>
#include <cstring>

namespace ethercat_driver {

constexpr uint32_t DVS_VENDOR_ID   = 0x00445653;
constexpr uint32_t DVS_PRODUCT_CODE = 0x00009252;

constexpr uint16_t ObjControlword         = 0x6040;
constexpr uint16_t ObjTargetPosition      = 0x607A;
constexpr uint16_t ObjModesOfOperation    = 0x6060;
constexpr uint16_t ObjMaxProfileVelocity  = 0x607F;
constexpr uint16_t ObjStatusword          = 0x6041;
constexpr uint16_t ObjActualPosition      = 0x6064;
constexpr uint16_t ObjErrorCode           = 0x603F;
constexpr uint16_t ObjActualTorque        = 0x6077;
constexpr uint16_t ObjTargetVelocity      = 0x60FF;

constexpr uint16_t CmdShutdown      = 0x0006;
constexpr uint16_t CmdSwitchOn      = 0x0007;
constexpr uint16_t CmdEnable        = 0x000F;
constexpr uint16_t CmdFaultReset    = 0x0080;

constexpr uint16_t MaskFault            = 0x004F;
constexpr uint16_t ValFault             = 0x0008;
constexpr uint16_t MaskOperationEnabled = 0x006F;
constexpr uint16_t ValOperationEnabled  = 0x0027;
constexpr uint16_t MaskReadyToSwitchOn  = 0x006F;
constexpr uint16_t ValReadyToSwitchOn   = 0x0021;
constexpr uint16_t MaskSwitchedOn       = 0x006F;
constexpr uint16_t ValSwitchedOn        = 0x0023;
constexpr uint16_t MaskSwitchOnDisabled = 0x004F;
constexpr uint16_t ValSwitchOnDisabled  = 0x0040;

static ec_pdo_entry_info_t dgn_pdo_entries[] = {
    {ObjControlword, 0x00, 16},
    {ObjTargetPosition, 0x00, 32},
    {ObjTargetVelocity, 0x00, 32},
    {ObjModesOfOperation, 0x00, 8},
    {ObjMaxProfileVelocity, 0x00, 32},
    {ObjStatusword, 0x00, 16},
    {ObjActualPosition, 0x00, 32},
    {ObjErrorCode, 0x00, 16},
    {ObjActualTorque, 0x00, 16},
};

static ec_pdo_info_t dgn_pdos[] = {
    {0x1600, 5, dgn_pdo_entries + 0}, 
    {0x1A00, 4, dgn_pdo_entries + 5}, 
};

static ec_sync_info_t dgn_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, dgn_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, dgn_pdos + 1, EC_WD_DISABLE},
    {0xFF, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE}
};

EthercatAxisAdapter::EthercatAxisAdapter(EthercatAxisAdapterConfig config)
    : config_(std::move(config)) {
    recalculate_counts_per_radian();
}
    
EthercatAxisAdapter::~EthercatAxisAdapter() = default;

motion_core::AxisInfo EthercatAxisAdapter::info() const {
    motion_core::AxisInfo info{};
    info.id = config_.axis_id;
    info.name = config_.axis_name;
    info.transport = motion_core::AxisTransportKind::Ethercat;
    info.capabilities = {
        motion_core::Capability::ReadTelemetry,
        motion_core::Capability::SetTargetPosition,
        motion_core::Capability::EnableDisable,
        motion_core::Capability::EmergencyStop,
        motion_core::Capability::ReadParameters,
        motion_core::Capability::WriteParameters,
    };
    return info;
}

motion_core::Result<void> EthercatAxisAdapter::configure_hardware() {
    auto* master = config_.bus_manager->master();
    auto* domain = config_.bus_manager->domain();
    if (!master || !domain) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "EtherCAT bus is not initialized"});
    }

    ec_slave_config_t* sc = ecrt_master_slave_config(master, 0, config_.ecat_bus_position, DVS_VENDOR_ID, DVS_PRODUCT_CODE);
    if (!sc) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get slave config"});
    }

    if (ecrt_slave_config_pdos(sc, EC_END, dgn_syncs)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to configure slave PDOs"});
    }

    // One bit-position slot per registered PDO entry below.
    // Keep the size in sync with the number of domain_regs entries that use bit_positions[i].
    std::array<unsigned int, 9> bit_positions{};

    ec_pdo_entry_reg_t domain_regs[] = {
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjControlword,
         0,
         &off_ctrl_,
         &bit_positions[0]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjTargetPosition,
         0,
         &off_target_pos_,
         &bit_positions[1]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjModesOfOperation,
         0,
         &off_modes_op_,
         &bit_positions[2]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjMaxProfileVelocity,
         0,
         &off_max_vel_,
         &bit_positions[3]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjStatusword,
         0,
         &off_status_,
         &bit_positions[4]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjActualPosition,
         0,
         &off_act_pos_,
         &bit_positions[5]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjErrorCode,
         0,
         &off_error_,
         &bit_positions[6]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjActualTorque,
         0,
         &off_act_torque_,
         &bit_positions[7]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjTargetVelocity,
         0,
         &off_target_vel_,
         &bit_positions[8]},
        {0, 0, 0, 0, 0, 0, nullptr, nullptr}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "PDO entry registration failed"});
    }

    config_.bus_manager->register_adapter(config_.ecat_axis_index, this);

    // Read axial resolution (0x6091:02) — instruction units per motor revolution.
    // This is the scale for all PDO position objects (0x6064, 0x607A).
    // Default is 10000. Do NOT use encoder_resolution_bits_ for this.
    uint32_t axial_res = 0;
    uint32_t abort_code = 0;
    size_t result_size = 0;
    if (ecrt_master_sdo_upload(master, config_.ecat_bus_position,
                               0x6091, 2,
                               reinterpret_cast<uint8_t*>(&axial_res), sizeof(axial_res),
                               &result_size, &abort_code) == 0 && result_size == 4 && axial_res > 0) {
        counts_per_revolution_.store(axial_res, std::memory_order_release);
        recalculate_counts_per_radian();
    }

    uint32_t max_profile_vel = 0;
    abort_code = 0;
    result_size = 0;
    if (ecrt_master_sdo_upload(master, config_.ecat_bus_position,
                               ObjMaxProfileVelocity, 0,
                               reinterpret_cast<uint8_t*>(&max_profile_vel), sizeof(max_profile_vel),
                               &result_size, &abort_code) == 0 && result_size == 4 && max_profile_vel > 0) {
        max_profile_velocity_instr_s_.store(max_profile_vel, std::memory_order_release);
        has_max_profile_velocity_instr_s_.store(true, std::memory_order_release);
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::start() {
    started_ = true;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::stop() {
    cmd_.enable_req.store(false, std::memory_order_release);
    started_ = false;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::set_enabled(bool enabled) {
    auto status = ensure_started();
    if (!status.ok()) return status;
    
    if (enabled && !cmd_.enable_req.load(std::memory_order_relaxed)) {
        // Sync target before enabling (bumpless transfer)
        cmd_.target_pos_deg.store(telem_.actual_position_deg.load(std::memory_order_acquire), std::memory_order_release);
    }
    cmd_.enable_req.store(enabled, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::set_mode(motion_core::AxisMode mode) {
    auto status = ensure_started();
    if (!status.ok()) return status;
    int8_t work_mode = mode_to_work_mode(mode);
    if (work_mode < 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::Unsupported, "Mode not supported"});
    }
    cmd_.mode_req.store(work_mode, std::memory_order_release);
    mode_ = mode;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::apply_command(const motion_core::AxisCommand& command) {
    auto status = ensure_started();
    if (!status.ok()) return status;

    if (command.emergency_stop) {
        cmd_.enable_req.store(false, std::memory_order_release);
    }
    if (command.clear_errors) {
        cmd_.reset_req.store(true, std::memory_order_release);
    }
    if (command.go_home) {
        cmd_.mode_req.store(6, std::memory_order_release);
        cmd_.homing_req.store(true, std::memory_order_release);
    }
    if (command.set_zero) {
        cmd_.set_zero_req.store(true, std::memory_order_release);
    }
    if (command.has_target_velocity) {
        cmd_.has_target_vel.store(true, std::memory_order_release);
        cmd_.target_vel_deg_s.store(command.target_velocity_deg_per_sec, std::memory_order_release);
    } else {
        cmd_.has_target_vel.store(false, std::memory_order_release);
    }
    if (command.has_profile_speed_rpm) {
        cmd_.has_profile_vel.store(true, std::memory_order_release);
        cmd_.profile_vel_rpm.store(command.profile_speed_rpm, std::memory_order_release);
    } else {
        cmd_.has_profile_vel.store(false, std::memory_order_release);
    }
    if (command.has_target_position) {
        if (mode_ != motion_core::AxisMode::ProfilePosition &&
            mode_ != motion_core::AxisMode::CyclicSyncPosition) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Unsupported, "Current EtherCAT mode does not accept position targets"});
        }
        double target_deg = command.target_position_deg;
        if (command.is_relative) {
            target_deg += telem_.actual_position_deg.load(std::memory_order_acquire);
        }
        cmd_.target_pos_deg.store(target_deg, std::memory_order_release);
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::AxisTelemetry> EthercatAxisAdapter::read_telemetry() const {
    auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::AxisTelemetry>::failure(status.error());

    motion_core::AxisTelemetry out{};
    out.timestamp_ns = telem_.timestamp_ns.load(std::memory_order_acquire);
    out.actual_position_deg = telem_.actual_position_deg.load(std::memory_order_acquire);
    out.actual_velocity_deg_per_sec = telem_.actual_velocity_deg_s.load(std::memory_order_acquire);
    out.actual_torque_percent = telem_.actual_torque_pct.load(std::memory_order_acquire);
    out.target_position_deg = cmd_.target_pos_deg.load(std::memory_order_acquire);
    out.status_word = telem_.statusword.load(std::memory_order_acquire);
    out.mode = mode_;

    if (telem_.is_faulted.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::Fault;
    } else if (telem_.is_enabled.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::OperationEnabled;
    } else if (telem_.is_ready.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::Ready;
    } else if ((out.status_word & MaskSwitchOnDisabled) == ValSwitchOnDisabled) {
        out.state = motion_core::AxisState::Disabled;
    } else {
        out.state = motion_core::AxisState::Unknown;
    }

    return motion_core::Result<motion_core::AxisTelemetry>::success(std::move(out));
}

std::vector<motion_core::ParameterDescriptor> EthercatAxisAdapter::make_parameter_descriptors() {
    std::vector<motion_core::ParameterDescriptor> out;
    out.reserve(p100e_dictionary.size() + 2);

    // Build descriptors from the authoritative p100e_dictionary.
    for (const auto& def : p100e_dictionary) {
        motion_core::ParameterDescriptor d{};
        d.id = def.id;
        d.name = def.name;
        d.group = def.group;
        d.unit = def.unit;
        d.read_only = def.is_read_only;
        d.persistable = def.persistable_runtime;
        d.has_min = true;
        d.has_max = true;
        d.min_value = def.min_value;
        d.max_value = def.max_value;
        out.push_back(d);
    }

    // Gear ratio is a local config parameter, not an SDO in the dictionary.
    motion_core::ParameterDescriptor gear{};
    gear.id = motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio);
    gear.name = "Gear Ratio";
    gear.group = "Common/Mechanics";
    gear.unit = "motor_turns_per_output_turn";
    gear.read_only = false;
    gear.persistable = true;
    gear.has_min = true;
    gear.has_max = true;
    gear.min_value = motion_core::ParameterValue::from_floating(0.001);
    gear.max_value = motion_core::ParameterValue::from_floating(1000.0);
    out.push_back(gear);

    return out;
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> EthercatAxisAdapter::list_parameters() const {
    return motion_core::Result<std::vector<motion_core::ParameterDescriptor>>::success(make_parameter_descriptors());
}

motion_core::Result<motion_core::ParameterSet> EthercatAxisAdapter::read_parameters() const {
    auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::ParameterSet>::failure(status.error());

    auto* master = config_.bus_manager->master();
    if (!master) {
        return motion_core::Result<motion_core::ParameterSet>::failure(
            {motion_core::ErrorCode::NotConnected, "Master not available"});
    }

    motion_core::ParameterSet out{};
    uint32_t abort_code = 0;
    size_t result_size = 0;

    // Read all SDO entries from the dictionary.
    for (const auto& def : p100e_dictionary) {
        uint8_t buf[8] = {};
        if (ecrt_master_sdo_upload(master, config_.ecat_bus_position,
                                   def.index, def.sub_index,
                                   buf, def.data_size,
                                   &result_size, &abort_code) != 0) {
            continue; // skip on SDO error
        }

        motion_core::ParameterValue val{};
        if (def.type == motion_core::ParameterValueType::SignedInteger) {
            int32_t v = 0;
            if (def.data_size == 1) v = static_cast<int8_t>(buf[0]);
            else if (def.data_size == 2) { int16_t t; std::memcpy(&t, buf, 2); v = t; }
            else if (def.data_size == 4) { std::memcpy(&v, buf, 4); }
            val = motion_core::ParameterValue::from_signed(v);
        } else {
            uint32_t v = 0;
            if (def.data_size == 1) v = buf[0];
            else if (def.data_size == 2) { uint16_t t; std::memcpy(&t, buf, 2); v = t; }
            else if (def.data_size == 4) { std::memcpy(&v, buf, 4); }
            val = motion_core::ParameterValue::from_unsigned(v);
        }
        out.entries.push_back({def.id, val});
    }

    // Gear ratio: local config, no SDO.
    out.entries.push_back({
        motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio),
        motion_core::ParameterValue::from_floating(gear_ratio_.load(std::memory_order_acquire))
    });

    return motion_core::Result<motion_core::ParameterSet>::success(std::move(out));
}

motion_core::Result<void> EthercatAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    auto status = ensure_started();
    if (!status.ok()) return status;

    auto* master = config_.bus_manager->master();
    if (!master) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "Master not available"});
    }

    for (const auto& entry : patch.entries) {
        if (entry.id.domain == motion_core::ParameterDomain::Common) {
            const auto common = static_cast<motion_core::CommonParameter>(entry.id.value);
            if (common == motion_core::CommonParameter::HardwareGearRatio) {
                double ratio = 0.0;
                if (entry.value.type == motion_core::ParameterValueType::FloatingPoint)
                    ratio = entry.value.floating_value;
                else if (entry.value.type == motion_core::ParameterValueType::UnsignedInteger)
                    ratio = static_cast<double>(entry.value.unsigned_value);
                else if (entry.value.type == motion_core::ParameterValueType::SignedInteger)
                    ratio = static_cast<double>(entry.value.signed_value);
                if (ratio > 0.0 && std::isfinite(ratio)) {
                    gear_ratio_.store(ratio, std::memory_order_release);
                    recalculate_counts_per_radian();
                }
            }
            continue;
        }

        if (entry.id.domain != motion_core::ParameterDomain::Ethercat) {
            continue;
        }

        // Find the SDO address in the dictionary.
        const ParameterDefinition* def = nullptr;
        for (const auto& d : p100e_dictionary) {
            if (d.id.domain == entry.id.domain && d.id.value == entry.id.value) {
                def = &d;
                break;
            }
        }
        if (!def || def->is_read_only) continue;

        // Marshal value into little-endian bytes.
        uint8_t buf[4] = {};
        if (def->type == motion_core::ParameterValueType::SignedInteger) {
            int32_t v = (entry.value.type == motion_core::ParameterValueType::SignedInteger)
                            ? static_cast<int32_t>(entry.value.signed_value)
                            : static_cast<int32_t>(entry.value.unsigned_value);
            std::memcpy(buf, &v, def->data_size);
        } else {
            uint32_t v = (entry.value.type == motion_core::ParameterValueType::UnsignedInteger)
                             ? static_cast<uint32_t>(entry.value.unsigned_value)
                             : static_cast<uint32_t>(entry.value.signed_value);
            std::memcpy(buf, &v, def->data_size);
        }

        uint32_t abort_code = 0;
        if (ecrt_master_sdo_download(master, config_.ecat_bus_position,
                                     def->index, def->sub_index,
                                     buf, def->data_size, &abort_code) != 0 || abort_code != 0) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::TransportFailure, "SDO download failed for EtherCAT parameter patch"});
        }

        // If this was AxialResolution, update local scale.
        if (entry.id.value == static_cast<uint32_t>(motion_core::EthercatParameter::AxialResolution)) {
            uint32_t new_res = 0;
            std::memcpy(&new_res, buf, 4);
            if (new_res > 0) {
                counts_per_revolution_.store(new_res, std::memory_order_release);
                recalculate_counts_per_radian();
            }
        }

        if (entry.id.value == static_cast<uint32_t>(motion_core::EthercatParameter::MaxProfileVelocityCountsPerSec)) {
            uint32_t max_profile_vel = 0;
            std::memcpy(&max_profile_vel, buf, 4);
            if (max_profile_vel > 0) {
                max_profile_velocity_instr_s_.store(max_profile_vel, std::memory_order_release);
                has_max_profile_velocity_instr_s_.store(true, std::memory_order_release);
            }
        }
    }

    return motion_core::Result<void>::success();
}

void EthercatAxisAdapter::process_cycle(uint8_t* domain_pd, double dt_s) {
    if (!domain_pd) return;

    // --- Read Process Data ---
    uint16_t statusword = EC_READ_U16(domain_pd + off_status_);
    int32_t act_pos_counts = EC_READ_S32(domain_pd + off_act_pos_);
    int16_t act_torque = EC_READ_S16(domain_pd + off_act_torque_);
    uint16_t err_code = 0;
    
    bool is_faulted = false;
    bool is_ready = false;
    bool is_enabled = false;

    if ((statusword & MaskFault) == ValFault) {
        is_faulted = true;
        err_code = EC_READ_U16(domain_pd + off_error_);
    } else if ((statusword & MaskOperationEnabled) == ValOperationEnabled) {
        is_enabled = true;
    } else if ((statusword & MaskSwitchedOn) == ValSwitchedOn || 
               (statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
        is_ready = true;
    }

    if (cmd_.set_zero_req.exchange(false, std::memory_order_relaxed)) {
        zero_offset_counts_.store(act_pos_counts, std::memory_order_release);
        cmd_.target_pos_deg.store(0.0, std::memory_order_release);
        cmd_.target_pos_counts.store(act_pos_counts, std::memory_order_release);
    }

    const auto zero_offset_counts = zero_offset_counts_.load(std::memory_order_acquire);
    const auto counts_per_radian = counts_per_radian_.load(std::memory_order_acquire);

    double pos_rad = static_cast<double>(act_pos_counts - zero_offset_counts) / counts_per_radian;
    double pos_deg = pos_rad * (180.0 / M_PI);

    // Calc velocity using cycle DT
    double velocity_deg_s = 0.0;
    if (dt_s > 0.0) {
        velocity_deg_s = (pos_deg - last_position_deg_) / dt_s;
    }
    last_position_deg_ = pos_deg;
    last_timestamp_ns_ += static_cast<uint64_t>(dt_s * 1e9);

    // Update Telemetry Atomically
    telem_.statusword.store(statusword, std::memory_order_release);
    telem_.error_code.store(err_code, std::memory_order_release);
    telem_.is_faulted.store(is_faulted, std::memory_order_release);
    telem_.is_ready.store(is_ready, std::memory_order_release);
    telem_.is_enabled.store(is_enabled, std::memory_order_release);
    telem_.actual_position_counts.store(act_pos_counts, std::memory_order_release);
    telem_.actual_position_deg.store(pos_deg, std::memory_order_release);
    telem_.actual_torque_pct.store(act_torque / 10.0, std::memory_order_release);
    telem_.actual_velocity_deg_s.store(velocity_deg_s, std::memory_order_release);
    telem_.timestamp_ns.store(last_timestamp_ns_, std::memory_order_release);

    // --- Write Process Data ---
    bool enable_req = cmd_.enable_req.load(std::memory_order_acquire);
    bool reset_req = cmd_.reset_req.exchange(false, std::memory_order_relaxed); // one-shot
    bool homing_req = cmd_.homing_req.load(std::memory_order_acquire);
    int8_t mode = cmd_.mode_req.load(std::memory_order_acquire);
    
    uint16_t controlword = 0;
    
    if (reset_req) {
         controlword = CmdFaultReset;
    } else if (enable_req) {
         if (is_faulted) {
             // wait for manual reset
         } else if ((statusword & MaskSwitchOnDisabled) == ValSwitchOnDisabled) {
             controlword = CmdShutdown;
         } else if ((statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
             controlword = CmdSwitchOn;
         } else if ((statusword & MaskSwitchedOn) == ValSwitchedOn) {
             controlword = CmdEnable;
             EC_WRITE_S8(domain_pd + off_modes_op_, mode);
         } else if (is_enabled) {
             controlword = CmdEnable;
             EC_WRITE_S8(domain_pd + off_modes_op_, mode);
         } else {
             controlword = CmdShutdown;
         }
    } else {
         if (is_enabled || (statusword & MaskSwitchedOn) == ValSwitchedOn || (statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
             controlword = CmdShutdown;
         }
    }

    if (mode == 6 && homing_req && is_enabled) {
        controlword |= 0x0010; // Homing Operation Start (Bit 4)
    }
    
    EC_WRITE_U16(domain_pd + off_ctrl_, controlword);
    cmd_.controlword_last_sent.store(controlword, std::memory_order_release);

    // Target Position
    double target_deg = cmd_.target_pos_deg.load(std::memory_order_acquire);
    double target_rad = target_deg * (M_PI / 180.0);
    int32_t target_counts = static_cast<int32_t>(target_rad * counts_per_radian) + zero_offset_counts;
    
    cmd_.target_pos_counts.store(target_counts, std::memory_order_release);
    EC_WRITE_S32(domain_pd + off_target_pos_, target_counts);
    
    // Target Velocity
    if (cmd_.has_target_vel.load(std::memory_order_acquire)) {
        double vel_deg_s = cmd_.target_vel_deg_s.load(std::memory_order_acquire);
        double vel_rad_s = vel_deg_s * (M_PI / 180.0);
        int32_t target_vel_counts = static_cast<int32_t>(vel_rad_s * counts_per_radian);
        EC_WRITE_S32(domain_pd + off_target_vel_, target_vel_counts);
    } else {
        EC_WRITE_S32(domain_pd + off_target_vel_, 0);
    }

    // Profile Velocity (Max Vel)
    if (cmd_.has_profile_vel.load(std::memory_order_acquire)) {
        double rps = cmd_.profile_vel_rpm.load(std::memory_order_acquire) / 60.0;
        double counts_per_rev_output = counts_per_revolution_.load(std::memory_order_acquire) * gear_ratio_.load(std::memory_order_acquire);
        uint32_t profile_vel_instr_s = static_cast<uint32_t>(rps * counts_per_rev_output);
        EC_WRITE_U32(domain_pd + off_max_vel_, profile_vel_instr_s);
    } else {
        const bool has_max_profile_velocity = has_max_profile_velocity_instr_s_.load(std::memory_order_acquire);
        const uint32_t max_profile_velocity = max_profile_velocity_instr_s_.load(std::memory_order_acquire);
        if (has_max_profile_velocity && max_profile_velocity > 0U) {
            EC_WRITE_U32(domain_pd + off_max_vel_, max_profile_velocity);
        }
    }
}

motion_core::Result<void> EthercatAxisAdapter::ensure_started() const {
    if (!started_) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "Axis not started"});
    return motion_core::Result<void>::success();
}

void EthercatAxisAdapter::recalculate_counts_per_radian() {
    const double counts_per_rev_output =
        static_cast<double>(counts_per_revolution_.load(std::memory_order_acquire))
        * gear_ratio_.load(std::memory_order_acquire);
    double counts_per_radian = counts_per_rev_output / (2.0 * M_PI);
    if (!(counts_per_radian > 0.0) || !std::isfinite(counts_per_radian)) {
        counts_per_radian = 1.0;
    }
    counts_per_radian_.store(counts_per_radian, std::memory_order_release);
}

int8_t EthercatAxisAdapter::mode_to_work_mode(motion_core::AxisMode mode) const {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition: return 1;    // PP
        case motion_core::AxisMode::ProfileVelocity: return 3;    // PV
        case motion_core::AxisMode::Homing: return 6;             // HM
        case motion_core::AxisMode::CyclicSyncPosition: return 8; // CSP
        case motion_core::AxisMode::CyclicSyncVelocity: return 9; // CSV
        case motion_core::AxisMode::CyclicSyncTorque: return 10;  // CST
        default: return -1;
    }
}

std::shared_ptr<EthercatAxisAdapter> make_ethercat_axis_adapter(EthercatAxisAdapterConfig config) {
    return std::make_shared<EthercatAxisAdapter>(std::move(config));
}

} // namespace ethercat_driver
