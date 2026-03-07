#include "mks/axis_manager.h"

#include "mks/port/can_port_interface.h"
#include "mks/port/gs_usb_can_port.h"
#include "mks/adapter/mks_axis_adapter.h"
#include "mks/protocol/mks_protocol.h"
#include "mks/adapter/mks_runtime_config.h"
#include "mks/adapter/mks_runtime_factory.h"
#include "mks/port/sim_can_port.h"
#include "ethercat/manager/ethercat_runtime_factory.h"
#include "motion_core/config/hal_runtime_config_json.h"
#include "motion_core/config/axis_config_json.h"

#include <QTimer>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <unordered_map>

namespace {

QVariant paramValueToVariant(const motion_core::ParameterValue& val) {
    switch (val.type) {
        case motion_core::ParameterValueType::SignedInteger: return static_cast<qlonglong>(val.signed_value);
        case motion_core::ParameterValueType::UnsignedInteger: return static_cast<qulonglong>(val.unsigned_value);
        case motion_core::ParameterValueType::FloatingPoint: return val.floating_value;
        case motion_core::ParameterValueType::Boolean: return val.bool_value;
    }
    return {};
}

motion_core::ParameterValue variantToParamValue(const QVariant& var, const motion_core::ParameterDescriptor* desc) {
    if (desc && desc->min_value.type == motion_core::ParameterValueType::Boolean &&
        desc->max_value.type == motion_core::ParameterValueType::Boolean) {
        return motion_core::ParameterValue::from_bool(var.toBool());
    }

    const bool prefer_unsigned = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::UnsignedInteger;
    const bool prefer_float = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::FloatingPoint;

    if (var.typeId() == QMetaType::Bool) return motion_core::ParameterValue::from_bool(var.toBool());
    if (var.typeId() == QMetaType::Double || var.typeId() == QMetaType::Float || prefer_float) {
        return motion_core::ParameterValue::from_floating(var.toDouble());
    }
    if (var.typeId() == QMetaType::LongLong) {
        const auto v = var.toLongLong();
        if (prefer_unsigned && v >= 0) {
            return motion_core::ParameterValue::from_unsigned(static_cast<std::uint64_t>(v));
        }
        return motion_core::ParameterValue::from_signed(v);
    }
    if (var.typeId() == QMetaType::ULongLong) {
        return motion_core::ParameterValue::from_unsigned(var.toULongLong());
    }
    if (var.typeId() == QMetaType::Int) {
        const auto v = var.toInt();
        if (prefer_unsigned && v >= 0) {
            return motion_core::ParameterValue::from_unsigned(static_cast<std::uint64_t>(v));
        }
        return motion_core::ParameterValue::from_signed(v);
    }
    if (var.typeId() == QMetaType::UInt) {
        return motion_core::ParameterValue::from_unsigned(var.toUInt());
    }
    if (prefer_unsigned) {
        bool ok = false;
        const auto uv = var.toULongLong(&ok);
        if (ok) {
            return motion_core::ParameterValue::from_unsigned(uv);
        }
    }
    return motion_core::ParameterValue::from_signed(var.toLongLong());
}

} // namespace

namespace mks {

AxisManager::AxisManager(QObject* parent) : QObject(parent) {
    fast_timer_ = new QTimer(this);
    fast_timer_->setTimerType(Qt::PreciseTimer);
    fast_timer_->setInterval(4); // ~250 Hz
    connect(fast_timer_, &QTimer::timeout, this, &AxisManager::onFastTick);

    slow_timer_ = new QTimer(this);
    slow_timer_->setInterval(1000); // 1 Hz
    connect(slow_timer_, &QTimer::timeout, this, [this]() {
        if (active_transport_ == ActiveTransport::Mks && !runtime_bus_managers_.empty()) {
            auto stats = runtime_bus_managers_.front()->get_bus_statistics();
            emit busStatisticsUpdated(stats.cycle_rate_hz, stats.bus_load_percent);
        } else if (active_transport_ == ActiveTransport::Ethercat && runtime_ethercat_bus_manager_) {
            auto stats = runtime_ethercat_bus_manager_->get_bus_statistics();
            emit busStatisticsUpdated(stats.cycle_rate_hz, stats.bus_load_percent);
        }
    });
}

AxisManager::~AxisManager() {
    if (fast_timer_) {
        fast_timer_->stop();
    }
    if (slow_timer_) {
        slow_timer_->stop();
    }
    closeDevice();
}

bool AxisManager::isReady() const {
    return control_service_ != nullptr;
}

void AxisManager::applySafetyBaselineForAxis(const int axis_id,
                                             const QString& reason,
                                             const bool force_disable) {
    if (!control_service_) {
        return;
    }

    motion_core::SafetyBaselineOptions options{};
    options.force_disable = force_disable;
    options.sync_target_to_actual = true;

    const auto result = control_service_->apply_safe_baseline(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, options);

    emit logMessage(QString("Axis %1 safety baseline (%2, force_disable=%3) -> %4")
                        .arg(axis_id)
                        .arg(reason)
                        .arg(force_disable ? 1 : 0)
                        .arg(result.ok() ? "OK" : result.error().message));
}

void AxisManager::reset_runtime_state() {
    runtime_started_axes_.clear();
    runtime_known_axes_.clear();

    if (control_service_) {
        const auto listed = control_service_->list_axes();
        if (listed.ok()) {
            for (const auto& axis_info : listed.value()) {
                (void)control_service_->stop_axis(axis_info.id);
                (void)control_service_->remove_axis(axis_info.id);
            }
        }
        (void)control_service_->stop_dispatch_loop();
    }

    for (const auto& bus_manager : runtime_bus_managers_) {
        if (bus_manager) {
            (void)bus_manager->stop();
        }
    }
    runtime_bus_managers_.clear();

    if (runtime_ethercat_bus_manager_) {
        (void)runtime_ethercat_bus_manager_->stop_runtime();
        (void)runtime_ethercat_bus_manager_->close();
        runtime_ethercat_bus_manager_.reset();
    }

    control_service_.reset();
}

motion_core::Result<void> AxisManager::rebuild_ethercat_runtime_for_discovered_axes(
    const std::vector<std::uint16_t>& axis_ids,
    const std::shared_ptr<ethercat_driver::EthercatBusManager>& existing_bus_manager) {
    if (opened_device_path_.isEmpty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "no opened EtherCAT interface"});
    }
    if (axis_ids.empty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotFound, "no EtherCAT axes discovered"});
    }

    stopRuntime();

    std::vector<ethercat_driver::EthercatAxisRuntimeConfig> axes_cfg;
    axes_cfg.reserve(axis_ids.size());

    for (const auto axis_id : axis_ids) {
        ethercat_driver::EthercatAxisRuntimeConfig acfg{};
        acfg.axis_id = motion_core::AxisId{axis_id};
        acfg.axis_name.value = "ECAT " + std::to_string(axis_id);
        axes_cfg.push_back(std::move(acfg));
    }

    motion_core::Result<ethercat_driver::EthercatRuntimeBuildResult> built =
        existing_bus_manager
            ? ethercat_driver::build_ethercat_runtime_on_existing_bus(existing_bus_manager, axes_cfg)
            : [&]() {
                  ethercat_driver::EthercatRuntimeConfig cfg{};
                  cfg.bus.interface_name = opened_device_path_.toStdString();
                  cfg.axes = axes_cfg;
                  return ethercat_driver::build_ethercat_runtime(cfg);
              }();
    if (!built.ok()) {
        return motion_core::Result<void>::failure(built.error());
    }

    reset_runtime_state();

    control_service_ = std::make_unique<motion_core::AxisControlService>();
    runtime_ethercat_bus_manager_ = built.value().bus_manager;

    for (const auto& axis : built.value().axes) {
        const auto reg = control_service_->add_axis(axis);
        if (!reg.ok()) {
            reset_runtime_state();
            return motion_core::Result<void>::failure(reg.error());
        }

        const auto axis_info = axis->info();
        runtime_known_axes_.insert(static_cast<int>(axis_info.id.value));
    }

    // After discovering axes, we might need to apply their configs if we had any
    // For now, just start what we have
    return startRuntimeHeadless();
}

motion_core::Result<void> AxisManager::startRuntimeHeadless() {
    if (!control_service_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "control service is not attached"});
    }

    const auto start_result = control_service_->start_runtime();
    if (!start_result.ok()) {
        return start_result;
    }

    for (const int axis_id : runtime_known_axes_) {
        runtime_started_axes_.insert(axis_id);
        applySafetyBaselineForAxis(axis_id, "startRuntimeHeadless", true);
    }

    watched_axes_.clear();
    rr_index_ = 0;
    if (!runtime_started_axes_.isEmpty()) {
        fast_timer_->start();
        slow_timer_->start();
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint16_t>> AxisManager::discover_axes(const QString& device_path,
                                                                            const int baud_rate,
                                                                            const int max_id) const {
    if (device_path.isEmpty()) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::InvalidArgument, "device_path is empty"});
    }
    if (max_id < 1 || max_id > 2047) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::InvalidArgument, "scan max id must be in [1, 2047]"});
    }

    std::unique_ptr<ICanPort> can_port;
    if (device_path.startsWith("sim", Qt::CaseInsensitive)) {
        can_port = std::make_unique<SimCanPort>();
    } else {
        can_port = std::make_unique<GsUsbCanPort>();
    }

    if (!can_port->open(device_path.toStdString().c_str(), static_cast<unsigned int>(baud_rate))) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::TransportFailure,
             "failed to open CAN device for scan (device busy, disconnected, or insufficient permissions)"});
    }

    MksProtocol protocol(*can_port);

    // Safety feature: disable all motors on the bus before scanning
    std::vector<std::uint8_t> disable_payload = {0x00};
    std::vector<std::uint8_t> dummy_resp;
    (void)protocol.sendCommand(0x00, MksCommand::EnableMotor, disable_payload, dummy_resp, 0xFF, 0, false);

    auto scan_once = [&](const int timeout_ms) {
        auto ids = protocol.scanBus(1, static_cast<std::uint16_t>(max_id), static_cast<unsigned int>(timeout_ms));
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    };

    auto found = scan_once(25);

    // Robustness on noisy CAN: confirm candidates with stricter timeout and keep stable intersection.
    if (found.size() > 1) {
        const auto confirm = scan_once(40);
        std::vector<std::uint16_t> stable;
        std::set_intersection(found.begin(),
                              found.end(),
                              confirm.begin(),
                              confirm.end(),
                              std::back_inserter(stable));

        if (!stable.empty()) {
            found = std::move(stable);
        }
    }

    // Final pass: query status directly to filter out IDs that only intermittently echoed.
    std::vector<std::uint16_t> verified;
    verified.reserve(found.size());
    for (const auto id : found) {
        std::vector<std::uint8_t> resp;
        if (protocol.sendCommand(id, MksCommand::QueryMotorStatus, {}, resp, 0xFF, 50, true)) {
            verified.push_back(id);
        }
    }
    found = std::move(verified);

    can_port->close();

    std::sort(found.begin(), found.end());
    found.erase(std::unique(found.begin(), found.end()), found.end());
    return motion_core::Result<std::vector<std::uint16_t>>::success(std::move(found));
}

motion_core::Result<void> AxisManager::rebuild_runtime_for_discovered_axes(
    const std::vector<std::uint16_t>& can_ids) {
    if (opened_device_path_.isEmpty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "no opened device path"});
    }
    if (opened_baud_rate_ <= 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "invalid opened baud rate"});
    }
    if (can_ids.empty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotFound, "no motors discovered"});
    }

    stopRuntime();

    MksRuntimeConfig cfg{};
    MksBusRuntimeConfig bus_cfg{};
    bus_cfg.interface_id = opened_device_path_.toStdString();
    bus_cfg.device_path = opened_device_path_.toStdString();
    bus_cfg.baud_rate = static_cast<unsigned int>(opened_baud_rate_);
    bus_cfg.cycle_time_ms = 4;

    for (const auto can_id : can_ids) {
        const MksAxisRuntimeConfig default_axis_cfg{};
        MksAxisRuntimeConfig acfg{};
        acfg.axis_id = motion_core::AxisId{can_id};
        acfg.can_id = can_id;
        acfg.axis_name.value = "Axis " + std::to_string(can_id);
        acfg.axis_units_per_degree = default_axis_cfg.axis_units_per_degree;
        acfg.default_speed = default_axis_cfg.default_speed;
        acfg.default_accel = default_axis_cfg.default_accel;
        bus_cfg.axes.push_back(acfg);
    }
    cfg.buses.push_back(std::move(bus_cfg));

    // On real hardware ensure response mode is ON before runtime starts.
    if (!opened_device_path_.startsWith("sim", Qt::CaseInsensitive)) {
        std::unique_ptr<ICanPort> can_port = std::make_unique<GsUsbCanPort>();
        if (can_port->open(opened_device_path_.toStdString().c_str(), static_cast<unsigned int>(opened_baud_rate_))) {
            MksProtocol protocol(*can_port);
            for (const auto can_id : can_ids) {
                std::vector<std::uint8_t> dummy;
                (void)protocol.sendCommand(can_id,
                                           MksCommand::SetSlaveRespondActive,
                                           {1, 1},
                                           dummy,
                                           0xFF,
                                           0,
                                           false);
            }
            can_port->close();
        }
    }

    const auto built = build_mks_runtime(cfg);
    if (!built.ok()) {
        return motion_core::Result<void>::failure(built.error());
    }

    reset_runtime_state();

    control_service_ = std::make_unique<motion_core::AxisControlService>();
    runtime_bus_managers_ = built.value().bus_managers;

    for (const auto& axis : built.value().axes) {
        const auto reg = control_service_->add_axis(axis);
        if (!reg.ok()) {
            reset_runtime_state();
            return motion_core::Result<void>::failure(reg.error());
        }

        const auto axis_info = axis->info();
        runtime_known_axes_.insert(static_cast<int>(axis_info.id.value));
    }

    (void)control_service_->start_dispatch_loop(std::chrono::milliseconds{4});

    for (const int axis_id : runtime_known_axes_) {
        const auto start_res = control_service_->start_axis(motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
        if (!start_res.ok()) {
            emit logMessage(QString("Runtime start axis %1 failed: %2").arg(axis_id).arg(start_res.error().message));
            continue;
        }
        runtime_started_axes_.insert(axis_id);
        applySafetyBaselineForAxis(axis_id, "rebuild_runtime", true);
    }

    watched_axes_.clear();
    rr_index_ = 0;
    if (!runtime_started_axes_.isEmpty()) {
        fast_timer_->start();
        slow_timer_->start();
    }

    return motion_core::Result<void>::success();
}

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    closeDevice();

    if (device_path.isEmpty()) {
        emit logMessage("Open failed: device path is empty");
        emit connectionChanged(false);
        return;
    }
    if (baud_rate <= 0) {
        emit logMessage("Open failed: invalid baud rate");
        emit connectionChanged(false);
        return;
    }

    std::unique_ptr<ICanPort> probe_port;
    if (device_path.startsWith("sim", Qt::CaseInsensitive)) {
        probe_port = std::make_unique<SimCanPort>();
    } else {
        probe_port = std::make_unique<GsUsbCanPort>();
    }

    if (!probe_port->open(device_path.toStdString().c_str(), static_cast<unsigned int>(baud_rate))) {
        emit logMessage(QString("Open failed: cannot access %1 @ %2 (device busy, disconnected, or insufficient permissions)")
                            .arg(device_path)
                            .arg(baud_rate));
        emit connectionChanged(false);
        return;
    }
    probe_port->close();

    opened_device_path_ = device_path;
    opened_baud_rate_ = baud_rate;
    device_opened_ = true;
    active_transport_ = ActiveTransport::Mks;

    emit logMessage(QString("Device opened: %1 @ %2. Use Scan to discover real motors.")
                        .arg(device_path)
                        .arg(baud_rate));

    runtime_known_axes_.clear();
    runtime_started_axes_.clear();
    watched_axes_.clear();
    rr_index_ = 0;
    emit scanFinished({});
    emit connectionChanged(false);
}

void AxisManager::openEthercatDevice(const QString& interface_name) {
    closeDevice();

    if (interface_name.isEmpty()) {
        emit logMessage("EtherCAT open failed: interface name is empty");
        emit connectionChanged(false);
        return;
    }

    opened_device_path_ = interface_name;
    opened_baud_rate_ = 0;
    device_opened_ = true;
    active_transport_ = ActiveTransport::Ethercat;

    emit logMessage(QString("EtherCAT interface opened: %1. Use Scan to discover slaves.")
                        .arg(interface_name));

    runtime_known_axes_.clear();
    runtime_started_axes_.clear();
    watched_axes_.clear();
    rr_index_ = 0;
    emit ethercatScanFinished({});
    emit connectionChanged(false);
}

void AxisManager::closeDevice() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();

    watched_axes_.clear();
    rr_index_ = 0;

    reset_runtime_state();

    opened_device_path_.clear();
    opened_baud_rate_ = 0;
    device_opened_ = false;
    active_transport_ = ActiveTransport::None;

    emit connectionChanged(false);
    emit logMessage("Device/runtime closed");
}

void AxisManager::loadRuntimeConfig(const QString& config_path) {
    runtime_config_path_ = config_path;
    emit logMessage(QString("Runtime config set: %1").arg(runtime_config_path_));
}

void AxisManager::loadHalConfig(const QString& config_path) {
    if (!control_service_) {
        control_service_ = std::make_unique<motion_core::AxisControlService>();
    }

    const auto hal_cfg = motion_core::load_hal_runtime_config_from_file(config_path.toStdString());
    if (!hal_cfg.ok()) {
        emit logMessage(QString("HAL config load failed: %1").arg(hal_cfg.error().message));
        return;
    }

    const auto open_res = control_service_->open_runtime(hal_cfg.value());
    if (!open_res.ok()) {
        emit logMessage(QString("HAL open runtime failed: %1").arg(open_res.error().message));
        return;
    }

    runtime_known_axes_.clear();
    const auto listed = control_service_->list_axes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            runtime_known_axes_.insert(static_cast<int>(info.id.value));
        }
    }

    emit logMessage(QString("HAL config loaded and runtime opened: %1 axis(es) found").arg(runtime_known_axes_.size()));

    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) {
        out.push_back(id);
    }
    emit scanFinished(out);
}

void AxisManager::startRuntime() {
    if (!control_service_ || !control_service_->is_runtime_open()) {
        emit logMessage("Runtime start failed: runtime is not open. Load HAL config or scan first.");
        emit connectionChanged(false);
        return;
    }

    if (control_service_->is_runtime_active()) {
        emit logMessage("Runtime already running; duplicate start skipped.");
        emit connectionChanged(true);
        return;
    }

    const auto start_result = startRuntimeHeadless();
    if (!start_result.ok()) {
        emit logMessage(QString("Runtime start failed: %1").arg(start_result.error().message));
        emit connectionChanged(false);
        return;
    }

    emit connectionChanged(!runtime_started_axes_.isEmpty());
    emit logMessage(QString("Runtime started: %1 axis(es)").arg(runtime_started_axes_.size()));
}

void AxisManager::stopRuntime() {
    if (fast_timer_) {
        fast_timer_->stop();
    }
    if (slow_timer_) {
        slow_timer_->stop();
    }

    reset_runtime_state();
    emit connectionChanged(false);
    emit logMessage("Runtime stopped");
}

void AxisManager::scanMotors(int max_id) {
    if (active_transport_ == ActiveTransport::Ethercat) {
        Q_UNUSED(max_id);
        scanEthercatMotors();
        return;
    }

    if (!device_opened_) {
        emit logMessage("Scan failed: open device first");
        emit scanFinished({});
        emit connectionChanged(false);
        return;
    }

    // If a runtime is currently active from a previous scan/start, release its CAN handle
    // before opening a temporary port for discovery.
    if (!runtime_bus_managers_.empty() || control_service_) {
        if (fast_timer_) {
            fast_timer_->stop();
        }
        if (slow_timer_) {
            slow_timer_->stop();
        }
        reset_runtime_state();
        watched_axes_.clear();
        rr_index_ = 0;
    }

    const auto discovered = discover_axes(opened_device_path_, opened_baud_rate_, max_id);
    if (!discovered.ok()) {
        emit logMessage(QString("Scan failed: %1").arg(discovered.error().message));
        emit scanFinished({});
        emit connectionChanged(false);
        return;
    }

    if (discovered.value().empty()) {
        emit logMessage(QString("Scan done: no motors found on %1 @ %2 (1..%3)")
                            .arg(opened_device_path_)
                            .arg(opened_baud_rate_)
                            .arg(max_id));
        emit scanFinished({});
        emit connectionChanged(false);
        return;
    }

    const auto rebuild = rebuild_runtime_for_discovered_axes(discovered.value());
    if (!rebuild.ok()) {
        emit logMessage(QString("Runtime rebuild failed after scan: %1").arg(rebuild.error().message));
        emit scanFinished({});
        emit connectionChanged(false);
        return;
    }

    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) {
        out.push_back(id);
    }

    emit logMessage(QString("Scan done: %1 axis(es) discovered and attached to runtime")
                        .arg(out.size()));
    emit scanFinished(out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
}

void AxisManager::scanEthercatMotors() {
    if (active_transport_ != ActiveTransport::Ethercat || !device_opened_) {
        emit logMessage("EtherCAT scan failed: open EtherCAT interface first");
        emit ethercatScanFinished({});
        emit connectionChanged(false);
        return;
    }

    if (runtime_ethercat_bus_manager_ || control_service_) {
        if (fast_timer_) {
            fast_timer_->stop();
        }
        if (slow_timer_) {
            slow_timer_->stop();
        }
        reset_runtime_state();
        watched_axes_.clear();
        rr_index_ = 0;
    }

    ethercat_driver::EthercatBusConfig bus_cfg{};
    bus_cfg.interface_name = opened_device_path_.toStdString();
    auto probe_bus = ethercat_driver::make_ethercat_bus_manager(std::move(bus_cfg));

    const auto open_result = probe_bus->open();
    if (!open_result.ok()) {
        emit logMessage(QString("EtherCAT scan failed: %1").arg(open_result.error().message));
        emit ethercatScanFinished({});
        emit connectionChanged(false);
        return;
    }

    const auto discovered = probe_bus->scan_axes();

    if (!discovered.ok()) {
        (void)probe_bus->close();
        emit logMessage(QString("EtherCAT scan failed: %1").arg(discovered.error().message));
        emit ethercatScanFinished({});
        emit connectionChanged(false);
        return;
    }

    if (discovered.value().empty()) {
        (void)probe_bus->close();
        emit logMessage(QString("EtherCAT scan done: no slaves found on %1").arg(opened_device_path_));
        emit ethercatScanFinished({});
        emit connectionChanged(false);
        return;
    }

    const auto rebuild = rebuild_ethercat_runtime_for_discovered_axes(discovered.value(), probe_bus);
    if (!rebuild.ok()) {
        (void)probe_bus->close();
        emit logMessage(QString("EtherCAT runtime rebuild failed after scan: %1").arg(rebuild.error().message));
        emit ethercatScanFinished({});
        emit connectionChanged(false);
        return;
    }

    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) {
        out.push_back(id);
    }

    emit logMessage(QString("EtherCAT scan done: %1 axis(es) discovered and attached to runtime")
                        .arg(out.size()));
    emit ethercatScanFinished(out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
}

void AxisManager::watchAxis(int axis_id, bool enabled) {
    if (axis_id < 1 || axis_id > 2047) return;
    if (enabled) {
        watched_axes_.insert(axis_id);
    } else {
        watched_axes_.remove(axis_id);
    }
}

void AxisManager::enableMotor(int axis_id, bool enabled) {
    if (!control_service_) {
        emit logMessage("Enable failed: control service is null");
        return;
    }

    const auto result = control_service_->enable_axis(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, enabled);
    emit logMessage(QString("Axis %1 enable=%2 -> %3")
                        .arg(axis_id)
                        .arg(enabled ? 1 : 0)
                        .arg(result.ok() ? "OK" : result.error().message));
}

void AxisManager::emergencyStop(int axis_id) {
    if (!control_service_) {
        emit logMessage("E-STOP failed: control service is null");
        return;
    }

    motion_core::AxisCommand cmd{};
    cmd.emergency_stop = true;
    const auto result = control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
    emit logMessage(QString("Axis %1 E-STOP -> %2")
                        .arg(axis_id)
                        .arg(result.ok() ? "OK" : result.error().message));
}

void AxisManager::clearErrors(int axis_id) {
    if (!control_service_) {
        emit logMessage("Clear Errors failed: control service is null");
        return;
    }

    motion_core::AxisCommand cmd{};
    cmd.clear_errors = true;
    const auto result = control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
    emit logMessage(QString("Axis %1 Clear Errors -> %2")
                        .arg(axis_id)
                        .arg(result.ok() ? "OK" : result.error().message));
    if (result.ok()) {
        applySafetyBaselineForAxis(axis_id, "clear_errors", false);
    }
}

void AxisManager::sendRawCommand(int axis_id, quint8 cmd, const QByteArray& data) {
    if (active_transport_ == ActiveTransport::Ethercat) {
        emit logMessage("Raw CAN command is not supported for EtherCAT transport");
        Q_UNUSED(axis_id);
        Q_UNUSED(cmd);
        Q_UNUSED(data);
        return;
    }

    if (runtime_bus_managers_.empty()) {
        emit logMessage("Failed to send raw command: no bus managers attached");
        return;
    }

    std::vector<std::uint8_t> payload(data.begin(), data.end());

    // Try sending it via every bus manager (usually there will only be one).
    // The bus manager will reject it if it doesn't own that axis_id.
    bool sent = false;
    for (const auto& bus : runtime_bus_managers_) {
        const auto result = bus->send_raw_command(static_cast<std::uint16_t>(axis_id), cmd, payload);
        if (result.ok()) {
            sent = true;
            break;
        }
    }

    if (!sent) {
        emit logMessage(QString("Axis %1 send raw command (0x%2) -> Failed (Not Found or Error)")
                            .arg(axis_id)
                            .arg(cmd, 2, 16, QChar('0')));
    } else {
        emit logMessage(QString("Axis %1 send raw command (0x%2) -> Dispatched")
                            .arg(axis_id)
                            .arg(cmd, 2, 16, QChar('0')));
    }
}

void AxisManager::moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg) {
    if (!control_service_) {
        return; // Suppressed log spam when control service is not attached
    }

    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.target_position_deg = axis_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    
    // Send command without emitting logMessage spam every 4ms!
    (void)control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
}

void AxisManager::moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg) {
    if (!control_service_) return;

    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.is_relative = true;
    cmd.target_position_deg = delta_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    
    (void)control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
}

void AxisManager::setMotionQueueOptions(int axis_id, int capacity, int policy_code) {
    if (!control_service_) {
        emit logMessage("setMotionQueueOptions failed: control service is null");
        return;
    }

    motion_core::MotionQueueOptions options{};
    options.capacity = static_cast<std::size_t>(std::max(1, capacity));
    switch (policy_code) {
        case 1:
            options.policy = motion_core::QueuePolicy::ReplaceLatest;
            break;
        case 2:
            options.policy = motion_core::QueuePolicy::DropNewest;
            break;
        case 3:
            options.policy = motion_core::QueuePolicy::DropOldest;
            break;
        case 0:
        default:
            options.policy = motion_core::QueuePolicy::Fifo;
            break;
    }

    const auto result = control_service_->set_motion_queue_options(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, options);
    if (!result.ok()) {
        emit logMessage(QString("Axis %1 queue options failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
    }
}

void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    if (!control_service_) {
        return;
    }

    std::vector<motion_core::MotionPoint> batch;
    batch.reserve(static_cast<std::size_t>(points.size()));
    for (const auto& value : points) {
        const QVariantMap map = value.toMap();
        motion_core::MotionPoint point{};
        point.position_deg = map.value("position_deg").toDouble();
        point.relative = map.value("relative", false).toBool();
        point.has_speed_rpm = map.value("has_speed_rpm", false).toBool();
        point.speed_rpm = static_cast<std::uint16_t>(std::clamp(map.value("speed_rpm").toInt(), 0, 3000));
        point.has_accel_percent = map.value("has_accel_percent", false).toBool();
        point.accel_percent = std::clamp(map.value("accel_percent").toDouble(), 0.0, 100.0);
        batch.push_back(point);
    }

    if (batch.empty()) {
        return;
    }

    const auto result = control_service_->enqueue_motion_batch(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, batch);
    if (!result.ok() && result.error().code != motion_core::ErrorCode::Busy) {
        emit logMessage(QString("Axis %1 enqueue batch failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
    }
}

void AxisManager::clearMotionQueue(int axis_id) {
    if (!control_service_) {
        return;
    }

    const auto result = control_service_->clear_motion_queue(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!result.ok()) {
        emit logMessage(QString("Axis %1 clear queue failed: %2")
                            .arg(axis_id)
                            .arg(result.error().message));
    }
}

QVariantMap AxisManager::queryMotionQueueStats(int axis_id) const {
    QVariantMap out;
    if (!control_service_) {
        return out;
    }

    const auto result = control_service_->get_motion_queue_stats(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!result.ok()) {
        return out;
    }

    out["size"] = static_cast<qulonglong>(result.value().size);
    out["capacity"] = static_cast<qulonglong>(result.value().capacity);
    out["pushed"] = static_cast<qulonglong>(result.value().pushed);
    out["dropped"] = static_cast<qulonglong>(result.value().dropped);
    return out;
}

void AxisManager::setAxisMode(int axis_id, int mode_code) {
    if (!control_service_) {
        return;
    }

    motion_core::AxisMode mode = motion_core::AxisMode::ProfilePosition;
    switch (mode_code) {
        case 9:
            mode = motion_core::AxisMode::CyclicSyncVelocity; // CSV
            break;
        case 8:
            mode = motion_core::AxisMode::CyclicSyncPosition; // CSP
            break;
        case 6:
            mode = motion_core::AxisMode::Homing;
            break;
        case 3:
            mode = motion_core::AxisMode::ProfileVelocity;
            break;
        case 1:
        default:
            mode = motion_core::AxisMode::ProfilePosition;
            break;
    }

    const auto result = control_service_->set_axis_mode(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, mode);
    emit logMessage(QString("Axis %1 set mode %2 -> %3")
                        .arg(axis_id)
                        .arg(mode_code)
                        .arg(result.ok() ? "OK" : result.error().message));
}

void AxisManager::setZeroPosition(int axis_id) {
    if (!control_service_) return;

    motion_core::AxisCommand cmd{};
    cmd.set_zero = true;
    const auto result = control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
    
    if (result.ok()) {
        emit logMessage(QString("Axis %1 set zero -> OK").arg(axis_id));
    } else {
        emit logMessage(QString("Axis %1 set zero failed: %2").arg(axis_id).arg(result.error().message));
    }
}

void AxisManager::goHome(int axis_id) {
    if (!control_service_) return;

    motion_core::AxisCommand cmd{};
    cmd.go_home = true;
    const auto result = control_service_->submit_command(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, cmd);
        
    if (result.ok()) {
        emit logMessage(QString("Axis %1 go home -> OK").arg(axis_id));
    } else {
        emit logMessage(QString("Axis %1 go home failed: %2").arg(axis_id).arg(result.error().message));
    }
}

QVariantList AxisManager::listParameters(int axis_id) const {
    QVariantList out;
    if (!control_service_) return out;

    const auto res = control_service_->list_parameters(motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!res.ok()) {
        return out;
    }

    for (const auto& desc : res.value()) {
        QVariantMap map;
        map["domain"] = static_cast<int>(desc.id.domain);
        map["value"] = static_cast<int>(desc.id.value);
        map["name"] = QString::fromUtf8(desc.name);
        map["group"] = QString::fromUtf8(desc.group);
        map["unit"] = QString::fromUtf8(desc.unit);
        map["read_only"] = desc.read_only;
        map["has_min"] = desc.has_min;
        map["has_max"] = desc.has_max;
        if (desc.has_min) map["min_value"] = paramValueToVariant(desc.min_value);
        if (desc.has_max) map["max_value"] = paramValueToVariant(desc.max_value);
        out.push_back(map);
    }
    return out;
}

QVariantList AxisManager::readParameters(int axis_id) const {
    QVariantList out;
    if (!control_service_) return out;

    const auto set_res = control_service_->read_parameters(motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!set_res.ok()) {
        return out;
    }

    for (const auto& entry : set_res.value().entries) {
        QVariantMap map;
        map["domain"] = static_cast<int>(entry.id.domain);
        map["value"] = static_cast<int>(entry.id.value);
        map["data"] = paramValueToVariant(entry.value);
        out.push_back(map);
    }
    return out;
}

void AxisManager::requestListParameters(int axis_id) {
    emit parameterListReady(axis_id, listParameters(axis_id));
}

void AxisManager::requestReadParameters(int axis_id) {
    emit parametersRead(axis_id, readParameters(axis_id));
}

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch_entries) {
    if (!control_service_) {
        emit logMessage("applyParameterPatch: control service is null");
        return;
    }

    std::unordered_map<std::uint32_t, motion_core::ParameterDescriptor> descriptors;
    const auto list_res = control_service_->list_parameters(motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (list_res.ok()) {
        for (const auto& d : list_res.value()) {
            const auto key = (static_cast<std::uint32_t>(d.id.domain) << 24) | (d.id.value & 0x00FFFFFFu);
            descriptors[key] = d;
        }
    }

    motion_core::ParameterPatch patch{};
    for (const auto& v : patch_entries) {
        const QVariantMap map = v.toMap();
        if (!map.contains("domain") || !map.contains("value") || !map.contains("data")) continue;
        
        motion_core::ParameterEntry entry{};
        entry.id.domain = static_cast<motion_core::ParameterDomain>(map["domain"].toInt());
        entry.id.value = static_cast<std::uint32_t>(map["value"].toInt());
        const auto key = (static_cast<std::uint32_t>(entry.id.domain) << 24) | (entry.id.value & 0x00FFFFFFu);
        const auto it = descriptors.find(key);
        const motion_core::ParameterDescriptor* desc = (it != descriptors.end()) ? &it->second : nullptr;
        entry.value = variantToParamValue(map["data"], desc);
        patch.entries.push_back(entry);
    }

    if (patch.entries.empty()) return;

    const auto result = control_service_->apply_parameter_patch(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, patch);

    if (!result.ok()) {
        emit logMessage(QString("Axis %1 apply patch error: %2 (code: %3)")
                            .arg(axis_id)
                            .arg(result.error().message)
                            .arg(static_cast<int>(result.error().code)));
    } else {
        emit logMessage(QString("Axis %1 applied patch with %2 entries")
                            .arg(axis_id)
                            .arg(patch.entries.size()));
        applySafetyBaselineForAxis(axis_id, "apply_parameter_patch", false);
    }
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    if (!control_service_) return;

    const auto export_res = control_service_->export_axis_config(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!export_res.ok()) {
        emit logMessage(QString("Axis %1 export config failed: %2")
                            .arg(axis_id)
                            .arg(export_res.error().message));
        return;
    }

    const auto save_res = motion_core::save_axis_config_to_file(path.toStdString(), export_res.value());
    if (!save_res.ok()) {
        emit logMessage(QString("Axis %1 save config failed: %2")
                            .arg(axis_id)
                            .arg(save_res.error().message));
    } else {
        emit logMessage(QString("Axis %1 config exported to %2").arg(axis_id).arg(path));
    }
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    if (!control_service_) return;

    const auto load_res = motion_core::load_axis_config_from_file(path.toStdString());
    if (!load_res.ok()) {
        emit logMessage(QString("Axis %1 load config failed: %2")
                            .arg(axis_id)
                            .arg(load_res.error().message));
        return;
    }

    const auto apply_res = control_service_->apply_axis_config(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)}, load_res.value());
    if (!apply_res.ok()) {
        emit logMessage(QString("Axis %1 apply config failed: %2")
                            .arg(axis_id)
                            .arg(apply_res.error().message));
    } else {
        emit logMessage(QString("Axis %1 config imported from %2").arg(axis_id).arg(path));
        applySafetyBaselineForAxis(axis_id, "import_axis_config", false);
    }
}

void AxisManager::onFastTick() {
    if (watched_axes_.isEmpty()) return;

    const QList<int> ids = watched_axes_.values();
    if (ids.isEmpty()) return;

    rr_index_ %= ids.size();
    const int axis_id = ids.at(rr_index_);
    rr_index_ = (rr_index_ + 1) % ids.size();

    QVariantMap t;

    if (!control_service_) return;

    const auto telemetry_res = control_service_->read_telemetry(
        motion_core::AxisId{static_cast<std::uint16_t>(axis_id)});
    if (!telemetry_res.ok()) {
        return;
    }

    const auto& telemetry = telemetry_res.value();
    t["status"] = static_cast<int>(telemetry.status_word);
    t["state"] = static_cast<int>(telemetry.state);
    t["axis"] = static_cast<double>(telemetry.actual_position_deg);
    t["target"] = static_cast<double>(telemetry.target_position_deg);
    t["speed"] = static_cast<double>(telemetry.actual_velocity_deg_per_sec);
    t["torque"] = static_cast<double>(telemetry.actual_torque_percent);
    t["protection"] = static_cast<int>(telemetry.status_word & 0xFF);
    emit telemetryUpdated(axis_id, t);
}

} // namespace mks
