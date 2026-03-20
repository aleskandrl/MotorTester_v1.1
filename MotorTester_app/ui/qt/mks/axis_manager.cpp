#include "mks/axis_manager.h"

#include "motion_core/config/hal_runtime_config_json.h"

#include <QTimer>

#include <algorithm>
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
    const bool prefer_unsigned = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::UnsignedInteger;
    const bool prefer_float = desc && desc->has_min && desc->min_value.type == motion_core::ParameterValueType::FloatingPoint;

    if (var.typeId() == QMetaType::Bool) return motion_core::ParameterValue::from_bool(var.toBool());
    if (var.typeId() == QMetaType::Double || var.typeId() == QMetaType::Float || prefer_float) {
        return motion_core::ParameterValue::from_floating(var.toDouble());
    }
    if (var.typeId() == QMetaType::ULongLong) return motion_core::ParameterValue::from_unsigned(var.toULongLong());
    if (var.typeId() == QMetaType::UInt) return motion_core::ParameterValue::from_unsigned(var.toUInt());

    const auto ll = var.toLongLong();
    if (prefer_unsigned && ll >= 0) {
        return motion_core::ParameterValue::from_unsigned(static_cast<std::uint64_t>(ll));
    }
    return motion_core::ParameterValue::from_signed(ll);
}

} // namespace

namespace mks {

AxisManager::AxisManager(QObject* parent) : QObject(parent) {
    fast_timer_ = new QTimer(this);
    fast_timer_->setTimerType(Qt::PreciseTimer);
    fast_timer_->setInterval(4);
    connect(fast_timer_, &QTimer::timeout, this, &AxisManager::onFastTick);

    slow_timer_ = new QTimer(this);
    slow_timer_->setInterval(1000);
    connect(slow_timer_, &QTimer::timeout, this, [this]() {
        QVariantMap stats_map;
        for (const auto& bus : hal_runtime_.bus_managers_snapshot()) {
            if (!bus) {
                continue;
            }
            const auto stat_res = bus->get_statistics();
            if (!stat_res.ok()) {
                continue;
            }
            QVariantMap bus_map;
            bus_map["cycle_rate_hz"] = stat_res.value().cycle_rate_hz;
            bus_map["bus_load_percent"] = stat_res.value().bus_load_percent;
            stats_map[QString::fromStdString(bus->get_name())] = bus_map;
        }
        if (!stats_map.isEmpty()) {
            emit busStatisticsUpdated(stats_map);
        }
    });
}

AxisManager::~AxisManager() {
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    closeDevice();
}

bool AxisManager::isReady() const {
    if (!hal_runtime_.is_open()) {
        return false;
    }
    const auto listed = hal_runtime_.list_axes();
    return listed.ok() && !listed.value().empty();
}

motion_core::Result<std::shared_ptr<motion_core::IAxis>> AxisManager::findAxis(const std::uint16_t axis_id) const {
    return hal_runtime_.find_axis(axis_id);
}

motion_core::Result<std::vector<motion_core::AxisInfo>> AxisManager::listAxes() const {
    return hal_runtime_.list_axes();
}

void AxisManager::applySafetyBaselineForAxis(const int axis_id, const QString& reason, const bool force_disable) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        return;
    }
    auto axis = axis_res.value();
    if (force_disable) {
        const auto disable_res = axis->set_enabled(false);
        if (!disable_res.ok()) {
            emit logMessage(QString("Axis %1 disable in safety baseline failed (%2): %3")
                                .arg(axis_id)
                                .arg(reason)
                                .arg(disable_res.error().message));
        }
    }

    const auto telem = axis->read_telemetry();
    if (telem.ok()) {
        motion_core::AxisCommand cmd{};
        cmd.has_target_position = true;
        cmd.target_position_deg = telem.value().actual_position_deg;
        (void)axis->apply_command(cmd);
    }
}

motion_core::Result<void> AxisManager::openRuntimeFromConfig(const motion_core::HalRuntimeConfig& config) {
    return hal_runtime_.open_from_config(config);
}

motion_core::Result<void> AxisManager::startRuntimeInternal() {
    return hal_runtime_.start();
}

motion_core::Result<void> AxisManager::stopRuntimeInternal() {
    return hal_runtime_.stop();
}

motion_core::Result<void> AxisManager::closeRuntimeInternal() {
    return hal_runtime_.close();
}

void AxisManager::reset_runtime_state() {
    runtime_started_axes_.clear();
    runtime_known_axes_.clear();
    (void)closeRuntimeInternal();
}

motion_core::Result<void> AxisManager::startRuntimeHeadless() {
    const auto start_result = startRuntimeInternal();
    if (!start_result.ok()) {
        return start_result;
    }
    runtime_started_axes_.clear();
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

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    closeDevice();
    if (device_path.isEmpty() || baud_rate <= 0) {
        emit logMessage("Open failed: invalid device path or baud rate");
        emit connectionChanged(false);
        return;
    }
    opened_device_path_ = device_path;
    opened_baud_rate_ = baud_rate;
    device_opened_ = true;
    active_transport_ = ActiveTransport::Mks;
    emit logMessage(QString("Device opened: %1 @ %2").arg(device_path).arg(baud_rate));
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
    device_opened_ = true;
    active_transport_ = ActiveTransport::Ethercat;
    emit logMessage(QString("EtherCAT interface opened: %1").arg(interface_name));
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
    current_hal_config_ = {};
    emit connectionChanged(false);
    emit logMessage("Device/runtime closed");
}

void AxisManager::loadRuntimeConfig(const QString& config_path) {
    runtime_config_path_ = config_path;
    emit logMessage(QString("Runtime config set: %1").arg(runtime_config_path_));
}

void AxisManager::loadHalConfig(const QString& config_path) {
    const auto hal_cfg = motion_core::load_hal_runtime_config_from_file(config_path.toStdString());
    if (!hal_cfg.ok()) {
        emit logMessage(QString("HAL config load failed: %1").arg(hal_cfg.error().message));
        return;
    }
    reset_runtime_state();
    current_hal_config_ = hal_cfg.value();
    const auto open_res = openRuntimeFromConfig(current_hal_config_);
    if (!open_res.ok()) {
        emit logMessage(QString("HAL open runtime failed: %1").arg(open_res.error().message));
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) {
            runtime_known_axes_.insert(static_cast<int>(info.id.value));
        }
    }
    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) out.push_back(id);
    emit scanFinished(out);
    emit logMessage(QString("HAL config loaded: %1 axis(es)").arg(out.size()));
}

void AxisManager::saveHalConfig(const QString& config_path) {
    const auto res = motion_core::save_hal_runtime_config_to_file(config_path.toStdString(), current_hal_config_);
    if (!res.ok()) {
        emit logMessage(QString("Configuration save failed: %1").arg(res.error().message));
    } else {
        emit logMessage(QString("Saved master configuration to %1").arg(config_path));
    }
}

void AxisManager::startRuntime() {
    if (!isReady()) {
        emit logMessage("Runtime start failed: runtime is not open");
        emit connectionChanged(false);
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
    if (fast_timer_) fast_timer_->stop();
    if (slow_timer_) slow_timer_->stop();
    watched_axes_.clear();
    rr_index_ = 0;
    (void)stopRuntimeInternal();
    runtime_started_axes_.clear();
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
        return;
    }
    reset_runtime_state();
    motion_core::MksScanRequest request{};
    request.device_path = opened_device_path_.toStdString();
    request.baud_rate = static_cast<std::uint32_t>(opened_baud_rate_);
    request.max_id = max_id;
    const auto discovered = hal_runtime_.scan_mks_topology(request);
    if (!discovered.ok()) {
        emit logMessage(QString("Scan failed via HalRuntime: %1").arg(discovered.error().message));
        emit scanFinished({});
        return;
    }
    motion_core::HalRuntimeConfig hal_cfg = discovered.value();
    current_hal_config_ = hal_cfg;
    const auto open_res = openRuntimeFromConfig(hal_cfg);
    if (!open_res.ok()) {
        emit logMessage(QString("Runtime build failed after scan: %1").arg(open_res.error().message));
        emit scanFinished({});
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) runtime_known_axes_.insert(static_cast<int>(info.id.value));
    }
    (void)startRuntimeHeadless();
    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) out.push_back(id);
    emit scanFinished(out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
}

void AxisManager::scanEthercatMotors() {
    if (active_transport_ != ActiveTransport::Ethercat || !device_opened_) {
        emit logMessage("EtherCAT scan failed: open EtherCAT interface first");
        emit ethercatScanFinished({});
        return;
    }
    reset_runtime_state();
    motion_core::EthercatScanRequest request{};
    request.interface_name = opened_device_path_.toStdString();
    const auto discovered = hal_runtime_.scan_ethercat_topology(request);
    if (!discovered.ok()) {
        emit logMessage(QString("EtherCAT scan failed via HalRuntime: %1").arg(discovered.error().message));
        emit ethercatScanFinished({});
        return;
    }
    motion_core::HalRuntimeConfig hal_cfg = discovered.value();
    current_hal_config_ = hal_cfg;
    const auto open_res = openRuntimeFromConfig(hal_cfg);
    if (!open_res.ok()) {
        emit logMessage(QString("EtherCAT runtime rebuild failed after scan: %1").arg(open_res.error().message));
        emit ethercatScanFinished({});
        return;
    }
    runtime_known_axes_.clear();
    const auto listed = listAxes();
    if (listed.ok()) {
        for (const auto& info : listed.value()) runtime_known_axes_.insert(static_cast<int>(info.id.value));
    }
    (void)startRuntimeHeadless();
    QVariantList out;
    auto sorted_ids = runtime_known_axes_.values();
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const int id : sorted_ids) out.push_back(id);
    emit ethercatScanFinished(out);
    emit connectionChanged(!runtime_started_axes_.isEmpty());
}

void AxisManager::watchAxis(int axis_id, bool enabled) {
    if (axis_id < 1 || axis_id > 2047) return;
    if (enabled) watched_axes_.insert(axis_id); else watched_axes_.remove(axis_id);
}

void AxisManager::enableMotor(int axis_id, bool enabled) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage(QString("Enable failed for axis %1: %2").arg(axis_id).arg(axis_res.error().message));
        return;
    }
    const auto result = axis_res.value()->set_enabled(enabled);
    if (!result.ok()) {
        emit logMessage(QString("Axis %1 enable=%2 failed: %3").arg(axis_id).arg(enabled ? 1 : 0).arg(result.error().message));
    }
}

void AxisManager::emergencyStop(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.emergency_stop = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(QString("Axis %1 E-STOP failed: %2").arg(axis_id).arg(result.error().message));
}

void AxisManager::clearErrors(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.clear_errors = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) {
        emit logMessage(QString("Axis %1 clear errors failed: %2").arg(axis_id).arg(result.error().message));
        return;
    }
    applySafetyBaselineForAxis(axis_id, "clear_errors", false);
}

void AxisManager::sendRawCommand(int axis_id, quint8 cmd, const QByteArray& data) {
    Q_UNUSED(axis_id);
    Q_UNUSED(cmd);
    Q_UNUSED(data);
    emit logMessage("Raw CAN commands are deprecated and unsupported in unified runtime.");
}

void AxisManager::moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.target_position_deg = axis_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    const auto res = axis_res.value()->apply_command(cmd);
    if (!res.ok()) emit logMessage(QString("Axis %1 absolute move failed: %2").arg(axis_id).arg(res.error().message));
}

void AxisManager::moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.has_target_position = true;
    cmd.is_relative = true;
    cmd.target_position_deg = delta_deg;
    cmd.has_profile_speed_rpm = true;
    cmd.profile_speed_rpm = static_cast<std::uint16_t>(std::clamp(speed, 0, 3000));
    cmd.has_profile_accel_percent = true;
    cmd.profile_accel_percent = std::clamp(static_cast<double>(accel), 0.0, 100.0);
    const auto res = axis_res.value()->apply_command(cmd);
    if (!res.ok()) emit logMessage(QString("Axis %1 relative move failed: %2").arg(axis_id).arg(res.error().message));
}

void AxisManager::setMotionQueueOptions(int axis_id, int capacity, int policy_code) {
    Q_UNUSED(axis_id);
    Q_UNUSED(capacity);
    Q_UNUSED(policy_code);
    emit logMessage("Motion queue options API removed: direct command path is used.");
}

void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    for (const auto& value : points) {
        const QVariantMap map = value.toMap();
        const bool relative = map.value("relative", false).toBool();
        const double pos = map.value("position_deg").toDouble();
        const int speed = map.value("speed_rpm", 400).toInt();
        const int accel = static_cast<int>(map.value("accel_percent", 50.0).toDouble());
        if (relative) moveRelativeAxis(axis_id, speed, accel, pos); else moveAbsoluteAxis(axis_id, speed, accel, pos);
    }
}

void AxisManager::clearMotionQueue(int axis_id) {
    Q_UNUSED(axis_id);
}

QVariantMap AxisManager::queryMotionQueueStats(int axis_id) const {
    Q_UNUSED(axis_id);
    QVariantMap out;
    out["size"] = static_cast<qulonglong>(0);
    out["capacity"] = static_cast<qulonglong>(0);
    out["pushed"] = static_cast<qulonglong>(0);
    out["dropped"] = static_cast<qulonglong>(0);
    return out;
}

void AxisManager::setAxisMode(int axis_id, int mode_code) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisMode mode = motion_core::AxisMode::ProfilePosition;
    switch (mode_code) {
        case 9: mode = motion_core::AxisMode::CyclicSyncVelocity; break;
        case 8: mode = motion_core::AxisMode::CyclicSyncPosition; break;
        case 6: mode = motion_core::AxisMode::Homing; break;
        case 3: mode = motion_core::AxisMode::ProfileVelocity; break;
        default: mode = motion_core::AxisMode::ProfilePosition; break;
    }
    const auto result = axis_res.value()->set_mode(mode);
    if (!result.ok()) emit logMessage(QString("Axis %1 set mode %2 failed: %3").arg(axis_id).arg(mode_code).arg(result.error().message));
}

void AxisManager::setZeroPosition(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.set_zero = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(QString("Axis %1 set zero failed: %2").arg(axis_id).arg(result.error().message));
}

void AxisManager::goHome(int axis_id) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    motion_core::AxisCommand cmd{};
    cmd.go_home = true;
    const auto result = axis_res.value()->apply_command(cmd);
    if (!result.ok()) emit logMessage(QString("Axis %1 go home failed: %2").arg(axis_id).arg(result.error().message));
}

QVariantList AxisManager::listParameters(int axis_id) const {
    QVariantList out;
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return out;
    const auto res = axis_res.value()->list_parameters();
    if (!res.ok()) return out;
    for (const auto& desc : res.value()) {
        QVariantMap map;
        map["domain"] = static_cast<int>(desc.id.domain);
        map["value"] = static_cast<int>(desc.id.value);
        map["name"] = QString::fromUtf8(desc.name);
        map["group"] = QString::fromUtf8(desc.group);
        map["unit"] = QString::fromUtf8(desc.unit);
        map["read_only"] = desc.read_only;
        map["persistable"] = desc.persistable;
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
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return out;
    const auto set_res = axis_res.value()->read_parameters();
    if (!set_res.ok()) return out;
    for (const auto& entry : set_res.value().entries) {
        QVariantMap map;
        map["domain"] = static_cast<int>(entry.id.domain);
        map["value"] = static_cast<int>(entry.id.value);
        map["data"] = paramValueToVariant(entry.value);
        out.push_back(map);
    }
    return out;
}

void AxisManager::requestListParameters(int axis_id) { emit parameterListReady(axis_id, listParameters(axis_id)); }
void AxisManager::requestReadParameters(int axis_id) { emit parametersRead(axis_id, readParameters(axis_id)); }

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch_entries) {
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) {
        emit logMessage("applyParameterPatch: axis not found");
        return;
    }
    std::unordered_map<std::uint32_t, motion_core::ParameterDescriptor> descriptors;
    const auto list_res = axis_res.value()->list_parameters();
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
    const auto result = axis_res.value()->apply_parameter_patch(patch);
    if (!result.ok()) {
        emit logMessage(QString("Axis %1 apply patch error: %2").arg(axis_id).arg(result.error().message));
    } else {
        emit logMessage(QString("Axis %1 applied patch with %2 entries").arg(axis_id).arg(patch.entries.size()));
        applySafetyBaselineForAxis(axis_id, "apply_parameter_patch", false);
    }
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    const auto save_res = hal_runtime_.export_axis_config_to_file(static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!save_res.ok()) {
        emit logMessage(QString("Axis %1 save config failed: %2").arg(axis_id).arg(save_res.error().message));
    } else {
        emit logMessage(QString("Axis %1 config exported to %2").arg(axis_id).arg(path));
    }
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    const auto apply_res = hal_runtime_.apply_axis_config_file(static_cast<std::uint16_t>(axis_id), path.toStdString());
    if (!apply_res.ok()) {
        emit logMessage(QString("Axis %1 apply config failed: %2").arg(axis_id).arg(apply_res.error().message));
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
    const auto axis_res = findAxis(static_cast<std::uint16_t>(axis_id));
    if (!axis_res.ok()) return;
    const auto telemetry_res = axis_res.value()->read_telemetry();
    if (!telemetry_res.ok()) return;

    QVariantMap t;
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
