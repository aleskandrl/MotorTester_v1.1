#pragma once

#include "ethercat/manager/ethercat_bus_manager.h"
#include "motion_core/axis_interface.h"

#include <memory>
#include <string>
#include <vector>

namespace ethercat_driver {

struct EthercatAxisRuntimeConfig {
    motion_core::AxisId axis_id{};
    motion_core::AxisName axis_name{};
};

struct EthercatRuntimeConfig {
    EthercatBusConfig bus{};
    std::vector<EthercatAxisRuntimeConfig> axes{};
};

struct EthercatRuntimeBuildResult {
    std::shared_ptr<EthercatBusManager> bus_manager{};
    std::vector<std::shared_ptr<motion_core::IAxis>> axes{};
};

[[nodiscard]] motion_core::Result<EthercatRuntimeBuildResult> build_ethercat_runtime_on_existing_bus(
    const std::shared_ptr<EthercatBusManager>& bus_manager,
    const std::vector<EthercatAxisRuntimeConfig>& axes);

[[nodiscard]] motion_core::Result<EthercatRuntimeBuildResult> build_ethercat_runtime(
    const EthercatRuntimeConfig& config);

} // namespace ethercat_driver
