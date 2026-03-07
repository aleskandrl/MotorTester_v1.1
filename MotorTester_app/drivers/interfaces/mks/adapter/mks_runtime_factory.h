#pragma once

#include "mks/manager/mks_can_bus_manager.h"
#include "mks/adapter/mks_runtime_config.h"
#include "motion_core/axis_interface.h"

#include <memory>
#include <vector>

namespace mks {

struct MksRuntimeBuildResult {
    std::vector<std::shared_ptr<MksCanBusManager>> bus_managers{};
    std::vector<std::shared_ptr<motion_core::IAxis>> axes{};
};

[[nodiscard]] motion_core::Result<MksRuntimeBuildResult> build_mks_runtime(const MksRuntimeConfig& config);

} // namespace mks
