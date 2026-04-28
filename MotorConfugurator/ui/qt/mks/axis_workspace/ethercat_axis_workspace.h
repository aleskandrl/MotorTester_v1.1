#pragma once

#include "mks/axis_workspace/axis_workspace.h"

#include <atomic>
#include <cstdint>



class EthercatAxisWorkspace final : public AxisWorkspace {
public:
    explicit EthercatAxisWorkspace(int axis_id,
                                   mks::AxisManager* manager,
                                   QWidget* parent = nullptr);
    ~EthercatAxisWorkspace() override;

private:
    void configureTransportUi() override;
    void onTransportSineToggled(bool enabled) override;
    void onTransportMotionQueueStatsUpdated(const QVariantMap& stats) override;
    void onTransportTelemetryUpdated(const QVariantMap& telemetry,
                                     const QString& transport,
                                     double t_sec) override;
    bool transportOwnsTargetUi() const override;
    bool transportProvidesTargetTrace() const override;
    double samplePeriodSec() const override;
    QString transportTag() const override;
    void onBeforeDisableAxis() override;
    void onBeforeHomeAxis() override;
    void onBeforeSetZeroAxis() override;

    bool supportsSineMode() const;
    void updateSineControlsAvailability();
    void disableSineUiState();

    static constexpr int kDefaultEthercatSpeedRpm = 60;

    bool drop_baseline_initialized_{false};
    bool sine_stopped_due_to_drops_{false};
    std::uint64_t last_driver_dropped_{0U};
    std::uint64_t last_driver_mode_{0U};
};
