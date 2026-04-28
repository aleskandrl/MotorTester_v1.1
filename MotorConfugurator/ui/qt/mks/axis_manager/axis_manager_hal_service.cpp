#include "mks/axis_manager/axis_manager.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include <algorithm>
#include <cmath>

namespace mks {

// ---------------------------------------------------------------------------
// executeAxisOperation
//
// The single execution path for ALL axis control commands — whether they
// originate from the local UI or arrive via IPC from HexaMotion.
//
// Registered as `set_axis_operation_handler` on HalHostService in constructor.
// ---------------------------------------------------------------------------
motion_core::Result<std::string> AxisManager::executeAxisOperation(
    hal_ipc::OwnerRole caller,
    hal_ipc::ControlOp op,
    std::uint16_t axis_id,
    const hal_ipc::AxisPointDto* point,
    const hal_ipc::HalControlFrameDto& frame) {

    const hal_host_service::MotionControlSource source =
        (caller == hal_ipc::OwnerRole::HexaMotion)
            ? hal_host_service::MotionControlSource::HexaMotion
            : hal_host_service::MotionControlSource::Ui;

    const auto submit_motion = [this, source, axis_id](const motion_core::MotionCommandPoint& point)
        -> motion_core::Result<std::string> {
        if (!runtime_queue_ingress_) {
            return motion_core::Result<std::string>::failure(
                {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
        }
        const auto result = runtime_queue_ingress_->enqueueCommandPoint(source, axis_id, point);
        if (!result.ok()) {
            return motion_core::Result<std::string>::failure(result.error());
        }
        return motion_core::Result<std::string>::success("");
    };

    const auto submit_service = [this, source, axis_id](const motion_core::ServiceCommandPoint& point)
        -> motion_core::Result<std::string> {
        if (!runtime_queue_ingress_) {
            return motion_core::Result<std::string>::failure(
                {motion_core::ErrorCode::InternalError, "runtime queue ingress is not configured"});
        }
        const auto result = runtime_queue_ingress_->enqueueServicePoint(source, axis_id, point);
        if (!result.ok()) {
            return motion_core::Result<std::string>::failure(result.error());
        }
        return motion_core::Result<std::string>::success("");
    };

    // Enforce estop latch — only certain "recovery" commands bypass it.
    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        if (estop_active_) {
            const bool recovery_op =
                op == hal_ipc::ControlOp::ClearFault  ||
                op == hal_ipc::ControlOp::DisableAxis  ||
                op == hal_ipc::ControlOp::EnableAxis;
            if (!recovery_op)
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::Busy, "global estop is active"});
        }
    }

    // Global stop — iterate all known axes.
    if (op == hal_ipc::ControlOp::Stop && axis_id == 0U) {
        return executeStopAllAxes(caller);
    }

    const auto axis_res = unified_runtime_.find_axis(axis_id);
    if (!axis_res.ok())
        return motion_core::Result<std::string>::failure(axis_res.error());

    auto& axis = axis_res.value();

    switch (op) {
        case hal_ipc::ControlOp::EnableAxis: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Enable;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::DisableAxis: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Disable;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::Stop: {
            axis->estop();
            return motion_core::Result<std::string>::success("");
        }

        case hal_ipc::ControlOp::Hold: {
            const auto telem = axis->telemetry();
            motion_core::MotionCommandPoint point{};
            point.type = motion_core::MotionCommandType::Position;
            point.axis_id = static_cast<int>(axis_id);
            point.value = telem.position;
            point.velocity = 0.0;
            point.acceleration = 0.0;
            point.timestamp_us = telem.timestamp_us;
            point.sample_period_sec = 0.004;
            return submit_motion(point);
        }

        case hal_ipc::ControlOp::SetZero: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::SetZero;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::ClearFault: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::ClearErrors;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::Home: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Home;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::StartManualHoming: {
            if (axis->info().transport != motion_core::AxisTransportKind::Ethercat) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::Unsupported, "manual DI3 homing is supported only for EtherCAT axes"});
            }
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::Home;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::StreamPoint: {
            if (!point)
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "StreamPoint requires axis point"});
            motion_core::MotionCommandPoint sp{};
            sp.type = motion_core::MotionCommandType::Stream;
            sp.axis_id = static_cast<int>(axis_id);
            sp.value = point->has_interpolated_position
                ? point->interpolated_position_deg
                : point->segment_target_deg;
            sp.velocity = point->has_interpolated_velocity ? point->interpolated_velocity_deg_s : 0.0;
            sp.acceleration = 0.0;
            sp.timestamp_us = 0U;
            sp.sample_period_sec = (point->segment_duration_sec > 0.0) ? point->segment_duration_sec : 0.004;
            return submit_motion(sp);
        }

        case hal_ipc::ControlOp::EnqueueMotionBatch: {
            if (frame.service_string_value.empty())
                return motion_core::Result<std::string>::success("");
            const QJsonDocument doc = QJsonDocument::fromJson(
                QByteArray::fromStdString(frame.service_string_value));
            if (!doc.isArray())
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "EnqueueMotionBatch expects JSON array"});
            for (const QJsonValue& jv : doc.array()) {
                if (!jv.isObject()) continue;
                const QJsonObject obj = jv.toObject();
                motion_core::MotionCommandPoint sp{};
                sp.type = motion_core::MotionCommandType::Stream;
                sp.axis_id = static_cast<int>(axis_id);
                sp.value = obj.value(QStringLiteral("target_position_deg")).toDouble();
                sp.is_relative = obj.value(QStringLiteral("is_relative")).toBool();
                sp.velocity = obj.value(QStringLiteral("target_velocity_deg_per_sec")).toDouble();
                sp.acceleration = std::clamp(obj.value(QStringLiteral("profile_accel_percent")).toDouble(), 0.0, 100.0);
                sp.timestamp_us = 0U;
                const double sp_sec = obj.value(QStringLiteral("sample_period_sec")).toDouble();
                sp.sample_period_sec = (std::isfinite(sp_sec) && sp_sec > 0.0) ? sp_sec : 0.005;
                const auto submit_res = submit_motion(sp);
                if (!submit_res.ok()) {
                    return submit_res;
                }
            }
            return motion_core::Result<std::string>::success("");
        }

        case hal_ipc::ControlOp::ConfigureMotionQueue: {
            return motion_core::Result<std::string>::failure(
                {motion_core::ErrorCode::Unsupported, "motion queue configuration removed from minimal public contract"});
        }

        case hal_ipc::ControlOp::ClearMotionQueue: {
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::ClearMotionQueue;
            command.axis_id = static_cast<int>(axis_id);
            return submit_service(command);
        }

        case hal_ipc::ControlOp::SetAxisMode: {
            const auto requested_mode = static_cast<motion_core::AxisMode>(frame.service_int_value);
            if (axis->info().transport == motion_core::AxisTransportKind::Ethercat
                && requested_mode == motion_core::AxisMode::Homing) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::Unsupported,
                     "EtherCAT standard homing is temporarily blocked; use ManualHoming (DI3)"});
            }
            motion_core::ServiceCommandPoint command{};
            command.type = motion_core::ServiceCommandType::SetOperatingMode;
            command.axis_id = static_cast<int>(axis_id);
            command.requested_mode = requested_mode;
            return submit_service(command);
        }

        case hal_ipc::ControlOp::StartMksHomingSequence: {
            // Parse axis IDs from frame.service_string_value (JSON array of ints).
            // If empty, use all known MKS axes.
            QList<int> axis_ids;
            if (!frame.service_string_value.empty()) {
                const QJsonDocument doc = QJsonDocument::fromJson(
                    QByteArray::fromStdString(frame.service_string_value));
                if (doc.isArray()) {
                    for (const QJsonValue& jv : doc.array()) {
                        if (jv.isDouble()) {
                            axis_ids.push_back(jv.toInt());
                        }
                    }
                }
            }
            if (axis_ids.isEmpty()) {
                // Default: all MKS axes.
                for (const int id : mks_axes_) {
                    axis_ids.push_back(id);
                }
            }
            if (axis_ids.isEmpty()) {
                return motion_core::Result<std::string>::failure(
                    {motion_core::ErrorCode::InvalidArgument, "no MKS axes available for homing sequence"});
            }
            QMetaObject::invokeMethod(this,
                                      [this, axis_ids]() { startMksHomingSequence(axis_ids); },
                                      Qt::QueuedConnection);
            return motion_core::Result<std::string>::success("");
        }

        case hal_ipc::ControlOp::StopMksHomingSequence: {
            QMetaObject::invokeMethod(this,
                                      [this]() { stopMksHomingSequence(); },
                                      Qt::QueuedConnection);
            return motion_core::Result<std::string>::success("");
        }

        case hal_ipc::ControlOp::QueryMotionQueueStats:
        case hal_ipc::ControlOp::ReadParameters:
        case hal_ipc::ControlOp::ListParameters:
        case hal_ipc::ControlOp::ApplyParameterPatch:
        case hal_ipc::ControlOp::SetPersistent:
        case hal_ipc::ControlOp::ImportAxisConfig:
        case hal_ipc::ControlOp::ImportAxisConfigPreview:
        case hal_ipc::ControlOp::ExportAxisConfig:
        case hal_ipc::ControlOp::None:
        default:
            return motion_core::Result<std::string>::success("");
    }
}

// ---------------------------------------------------------------------------
// executeStopAllAxes
// ---------------------------------------------------------------------------
motion_core::Result<std::string> AxisManager::executeStopAllAxes(hal_ipc::OwnerRole caller) {
    Q_UNUSED(caller);
    const auto listed = unified_runtime_.list_axes();
    if (!listed.ok())
        return motion_core::Result<std::string>::failure(listed.error());

    for (const auto& info : listed.value()) {
        const auto axis_res = unified_runtime_.find_axis(info.id.value);
        if (!axis_res.ok()) continue;
        axis_res.value()->estop();
    }
    return motion_core::Result<std::string>::success("");
}

} // namespace mks
