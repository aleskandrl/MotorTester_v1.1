#pragma once

#include "motion_core/axis_interface.h"
#include <vector>

namespace mks {

[[nodiscard]] const motion_core::ParameterDescriptor* find_mks_descriptor(motion_core::ParameterId id);
[[nodiscard]] std::vector<motion_core::ParameterDescriptor> get_mks_parameter_descriptors();

// Helper functions for parameter conversion
[[nodiscard]] motion_core::Result<std::uint64_t> require_unsigned_value(const motion_core::ParameterValue& value, const char* field_name);
[[nodiscard]] motion_core::Result<double> require_floating_value(const motion_core::ParameterValue& value, const char* field_name);
[[nodiscard]] motion_core::Result<bool> require_bool_value(const motion_core::ParameterValue& value, const char* field_name);

} // namespace mks
