#include "mks/axis_manager/axis_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

namespace {
[[nodiscard]] static QString transport_tag_for_axis(const std::shared_ptr<motion_core::IAxis>& axis) {
    if (!axis) return QStringLiteral("hal");
    switch (axis->info().transport) {
        case motion_core::AxisTransportKind::CanBus:   return QStringLiteral("mks");
        case motion_core::AxisTransportKind::Ethercat: return QStringLiteral("ecat");

        case motion_core::AxisTransportKind::Unknown:  return QStringLiteral("hal");
    }
    return QStringLiteral("hal");
}
}

namespace mks {

namespace {

[[nodiscard]] bool should_emit_throttled_axis_log(const int axis_id,
                                                  const std::chrono::milliseconds period) {
    static std::unordered_map<int, std::chrono::steady_clock::time_point> next_allowed_log_time{};
    const auto now = std::chrono::steady_clock::now();
    const auto it = next_allowed_log_time.find(axis_id);
    if (it != next_allowed_log_time.end() && now < it->second) {
        return false;
    }
    next_allowed_log_time[axis_id] = now + period;
    return true;
}

} // namespace


void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    if (!runtime_queue_ingress_ || points.isEmpty()) {
        return;
    }

    const auto axis_result = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
    if (!axis_result.ok()) {
        emit logMessage(QStringLiteral("hal"),
                        QString("Axis %1 motion enqueue failed: axis not found")
                            .arg(axis_id));
        return;
    }
    const auto axis = axis_result.value();

    for (const QVariant& v : points) {
        const QVariantMap map = v.toMap();

        motion_core::MotionCommandPoint pt{};
        const int kc = map.value(QStringLiteral("kind"), 0).toInt();
        if (kc == 2) {
            pt.type = motion_core::MotionCommandType::Velocity;
        } else if (kc == 0) {
            pt.type = motion_core::MotionCommandType::Stream;
        } else {
            pt.type = motion_core::MotionCommandType::Position;
        }

        pt.axis_id = axis_id;
        pt.value = map.value(QStringLiteral("target_position_deg")).toDouble();
        pt.velocity = map.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
        if (!std::isfinite(pt.velocity) || pt.velocity <= 0.0) {
            const int profile_speed_rpm =
                std::clamp(map.value(QStringLiteral("profile_speed_rpm")).toInt(), 0, 3000);
            pt.velocity = static_cast<double>(profile_speed_rpm) * 6.0;
        }
        pt.acceleration =
            std::clamp(map.value(QStringLiteral("profile_accel_percent")).toDouble(), 0.0, 100.0);
        pt.timestamp_us = 0U;
        const double sample_period = map.value(QStringLiteral("sample_period_sec")).toDouble();
        pt.sample_period_sec = (std::isfinite(sample_period) && sample_period > 0.0) ? sample_period : 0.005;
        pt.is_relative = map.value(QStringLiteral("is_relative")).toBool();

        const auto result = runtime_queue_ingress_->enqueueCommandPoint(
            hal_host_service::MotionControlSource::Ui,
            static_cast<std::uint16_t>(axis_id),
            pt);
        if (!result.ok()) {
            if (should_emit_throttled_axis_log(axis_id, std::chrono::milliseconds(500))) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 motion enqueue failed: %2")
                                    .arg(axis_id)
                                    .arg(QString::fromStdString(result.error().message)));
            }
            return;
        }
    }
}

void AxisManager::enqueueServiceBatch(int axis_id, const QVariantList& commands) {
    if (!runtime_queue_ingress_ || commands.isEmpty()) {
        return;
    }

    for (const QVariant& v : commands) {
        const QVariantMap map = v.toMap();
        motion_core::ServiceCommandPoint command{};
        command.axis_id = axis_id;

        const int kind_value = map.value(QStringLiteral("kind"), 0).toInt();
        switch (kind_value) {
            case 1:
                command.type = motion_core::ServiceCommandType::Enable;
                break;
            case 2:
                command.type = motion_core::ServiceCommandType::Disable;
                break;
            case 3:
                command.type = motion_core::ServiceCommandType::ClearErrors;
                break;
            case 4:
                command.type = motion_core::ServiceCommandType::Home;
                break;
            case 5:
                command.type = motion_core::ServiceCommandType::SetZero;
                break;
            case 6:
                command.type = motion_core::ServiceCommandType::SetOperatingMode;
                break;
            case 0:
                command.type = motion_core::ServiceCommandType::ClearMotionQueue;
                break;
            default:
                command.type = motion_core::ServiceCommandType::ResetDrive;
                break;
        }

        const int requested_mode_value = map.value(QStringLiteral("requested_mode"),
                                                   static_cast<int>(motion_core::AxisMode::ProfilePosition)).toInt();
        if (requested_mode_value >= static_cast<int>(motion_core::AxisMode::ProfilePosition)
            && requested_mode_value <= static_cast<int>(motion_core::AxisMode::VendorSpecific)) {
            const auto requested_mode = static_cast<motion_core::AxisMode>(requested_mode_value);
            command.requested_mode = requested_mode;

            if (command.type == motion_core::ServiceCommandType::SetOperatingMode) {
                const auto axis_result = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
                if (axis_result.ok()) {
                    const auto axis_transport = axis_result.value()->info().transport;
                    if (axis_transport == motion_core::AxisTransportKind::Ethercat
                        && requested_mode == motion_core::AxisMode::Homing) {
                        emit logMessage(transport_tag_for_axis(axis_result.value()),
                                        QString("Axis %1: EtherCAT standard homing is blocked, mode request ignored")
                                            .arg(axis_id));
                        continue;
                    }
                }
            }

            if (command.type == motion_core::ServiceCommandType::SetOperatingMode) {
                pending_mode_switch_requested_[axis_id] = requested_mode_value;
                pending_mode_switch_cycles_[axis_id] = 0U;
                const auto axis_result = unified_runtime_.find_axis(static_cast<std::uint16_t>(axis_id));
                emit logMessage(axis_result.ok() ? transport_tag_for_axis(axis_result.value()) : QStringLiteral("hal"),
                                QString("Axis %1 mode switch requested -> %2")
                                    .arg(axis_id)
                                    .arg(requested_mode_value));
            }
        }

        if (command.type == motion_core::ServiceCommandType::Home) {
            const double target_velocity_deg_s =
                map.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
            if (std::isfinite(target_velocity_deg_s) && target_velocity_deg_s > 0.0) {
                command.homing_speed_deg_per_sec = target_velocity_deg_s;
            } else {
                const int profile_speed_rpm =
                    std::clamp(map.value(QStringLiteral("profile_speed_rpm")).toInt(), 0, 3000);
                if (profile_speed_rpm > 0) {
                    command.homing_speed_deg_per_sec = static_cast<double>(profile_speed_rpm) * 6.0;
                }
            }
        }


        const auto result = runtime_queue_ingress_->enqueueServicePoint(
            hal_host_service::MotionControlSource::Ui,
            static_cast<std::uint16_t>(axis_id),
            command);
        if (!result.ok()) {
            emit logMessage(QStringLiteral("hal"),
                            QString("Axis %1 service enqueue failed: %2")
                                .arg(axis_id)
                                .arg(QString::fromStdString(result.error().message)));
            return;
        }
    }
}

} // namespace mks
