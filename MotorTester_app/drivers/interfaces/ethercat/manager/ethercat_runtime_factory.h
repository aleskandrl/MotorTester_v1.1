#pragma once

#include "ethercat/manager/ethercat_bus_manager.h"
#include "motion_core/axis_interface.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/runtime_factory_registry.h"

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

// EthercatRuntimeBuildResult removed since we use motion_core::RuntimeBuildResult

[[nodiscard]] motion_core::Result<motion_core::RuntimeBuildResult> build_ethercat_runtime_on_existing_bus(
    const std::shared_ptr<EthercatBusManager>& bus_manager,
    const std::vector<EthercatAxisRuntimeConfig>& axes);

[[nodiscard]] motion_core::Result<motion_core::RuntimeBuildResult> build_ethercat_runtime(
    const motion_core::HalRuntimeConfig& config);

} // namespace ethercat_driver
