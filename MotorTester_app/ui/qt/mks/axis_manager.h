#pragma once

#include <QObject>
#include <QSet>
#include <QString>
#include <QVariantList>
#include <QVariantMap>

#include "motion_core/axis_control_service.h"
#include "motion_core/result.h"
#include "ethercat/manager/ethercat_bus_manager.h"

#include <cstdint>
#include <memory>
#include <vector>

class QTimer;

namespace motion_core {
}

namespace mks {
class ICanPort;
class MksProtocol;
class MksCanBusManager;

class AxisManager : public QObject {
    Q_OBJECT

public:
    explicit AxisManager(QObject* parent = nullptr);
    ~AxisManager() override;

public slots:
    void openDevice(const QString& device_path, int baud_rate);
    void openEthercatDevice(const QString& interface_name);
    void closeDevice();
    void scanMotors(int max_id);
    void scanEthercatMotors();
    void watchAxis(int axis_id, bool enabled);

    void enableMotor(int axis_id, bool enabled);
    void emergencyStop(int axis_id);
    void clearErrors(int axis_id);
    void moveAbsoluteAxis(int axis_id, int speed, int accel, double axis_deg);
    void moveRelativeAxis(int axis_id, int speed, int accel, double delta_deg);
    void setMotionQueueOptions(int axis_id, int capacity, int policy_code);
    void enqueueMotionBatch(int axis_id, const QVariantList& points);
    void clearMotionQueue(int axis_id);
    void setAxisMode(int axis_id, int mode_code);
    void setZeroPosition(int axis_id);
    void goHome(int axis_id);

    QVariantList listParameters(int axis_id) const;
    QVariantList readParameters(int axis_id) const;
    void requestListParameters(int axis_id);
    void requestReadParameters(int axis_id);
    void applyParameterPatch(int axis_id, const QVariantList& patch);

    void exportAxisConfig(int axis_id, const QString& path);
    void importAxisConfig(int axis_id, const QString& path);

    void loadRuntimeConfig(const QString& config_path);
    void loadHalConfig(const QString& config_path);
    void startRuntime();
    void stopRuntime();

    void sendRawCommand(int axis_id, quint8 cmd, const QByteArray& data);

public:
    [[nodiscard]] QVariantMap queryMotionQueueStats(int axis_id) const;

signals:
    void logMessage(const QString& line);
    void connectionChanged(bool connected);
    void scanFinished(const QVariantList& axis_ids);
    void ethercatScanFinished(const QVariantList& axis_ids);
    void telemetryUpdated(int axis_id, const QVariantMap& telemetry);
    void busStatisticsUpdated(double cycle_rate_hz, double bus_load_percent);
    void configReadback(int axis_id, const QVariantMap& config);
    void parameterListReady(int axis_id, const QVariantList& params);
    void parametersRead(int axis_id, const QVariantList& params);

private slots:
    void onFastTick();

private:
    enum class ActiveTransport {
        None = 0,
        Mks,
        Ethercat,
    };

    bool isReady() const;
    void applySafetyBaselineForAxis(int axis_id, const QString& reason, bool force_disable = true);

    void reset_runtime_state();
    motion_core::Result<void> startRuntimeHeadless();
    motion_core::Result<std::vector<std::uint16_t>> discover_axes(const QString& device_path,
                                                                  int baud_rate,
                                                                  int max_id) const;
    motion_core::Result<void> rebuild_runtime_for_discovered_axes(const std::vector<std::uint16_t>& can_ids);
    motion_core::Result<void> rebuild_ethercat_runtime_for_discovered_axes(
        const std::vector<std::uint16_t>& axis_ids,
        const std::shared_ptr<ethercat_driver::EthercatBusManager>& existing_bus_manager = {});

    std::unique_ptr<motion_core::AxisControlService> control_service_;
    std::vector<std::shared_ptr<MksCanBusManager>> runtime_bus_managers_;
    std::shared_ptr<ethercat_driver::EthercatBusManager> runtime_ethercat_bus_manager_;
    QSet<int> runtime_known_axes_;
    QSet<int> runtime_started_axes_;
    QString runtime_config_path_;
    QString opened_device_path_;
    int opened_baud_rate_{0};
    bool device_opened_{false};
    ActiveTransport active_transport_{ActiveTransport::None};

    QTimer* fast_timer_{nullptr};
    QTimer* slow_timer_{nullptr};
    QSet<int> watched_axes_;
    int rr_index_{0};
};

} // namespace mks
