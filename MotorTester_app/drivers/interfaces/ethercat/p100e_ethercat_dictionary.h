#pragma once

#include "motion_core/parameter_id.h"
#include "motion_core/parameter_types.h"

#include <array>
#include <cstdint>

namespace ethercat_driver {

struct ParameterDefinition {
    // Technical data
    motion_core::ParameterId id;
    const char* rdt_key;
    uint16_t index;
    uint8_t sub_index;
    uint8_t data_size;
    motion_core::ParameterValueType type;
    bool is_read_only;

    // UI and validation data
    const char* name;
    const char* group;
    const char* unit;
    const char* description;
    motion_core::ParameterValue min_value;
    motion_core::ParameterValue max_value;
};

constexpr std::array<ParameterDefinition, 5> p100e_dictionary = {{
    {
        /*id=*/motion_core::make_parameter_id(motion_core::CommonParameter::HomingMethod),
        /*rdt_key=*/"homing.method",
        /*index=*/0x6098,
        /*sub_index=*/0,
        /*data_size=*/1,
        /*type=*/motion_core::ParameterValueType::SignedInteger,
        /*is_read_only=*/false,
        /*name=*/"Homing method",
        /*group=*/"Common/Homing",
        /*unit=*/"",
        /*description=*/"The homing method to be used.",
        /*min_value=*/motion_core::ParameterValue::from_signed(0),
        /*max_value=*/motion_core::ParameterValue::from_signed(35)
    },
    {
        /*id=*/motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedSwitchDegPerSec),
        /*rdt_key=*/"homing.speed_switch_deg_s",
        /*index=*/0x6099,
        /*sub_index=*/1,
        /*data_size=*/4,
        /*type=*/motion_core::ParameterValueType::UnsignedInteger,
        /*is_read_only=*/false,
        /*name=*/"Homing speed (switch)",
        /*group=*/"Common/Homing",
        /*unit=*/"deg/s",
        /*description=*/"Speed during search for switch.",
        /*min_value=*/motion_core::ParameterValue::from_unsigned(0),
        /*max_value=*/motion_core::ParameterValue::from_unsigned(4294967295)
    },
    {
        /*id=*/motion_core::make_parameter_id(motion_core::CommonParameter::HomingSpeedZeroDegPerSec),
        /*rdt_key=*/"homing.speed_zero_deg_s",
        /*index=*/0x6099,
        /*sub_index=*/2,
        /*data_size=*/4,
        /*type=*/motion_core::ParameterValueType::UnsignedInteger,
        /*is_read_only=*/false,
        /*name=*/"Homing speed (zero)",
        /*group=*/"Common/Homing",
        /*unit=*/"deg/s",
        /*description=*/"Speed during search for zero.",
        /*min_value=*/motion_core::ParameterValue::from_unsigned(0),
        /*max_value=*/motion_core::ParameterValue::from_unsigned(4294967295)
    },
    {
        /*id=*/motion_core::make_parameter_id(motion_core::CommonParameter::LimitsSoftwareMinDeg),
        /*rdt_key=*/"limits.software_limit_min_deg",
        /*index=*/0x607D,
        /*sub_index=*/1,
        /*data_size=*/4,
        /*type=*/motion_core::ParameterValueType::SignedInteger,
        /*is_read_only=*/false,
        /*name=*/"Software position limit (min)",
        /*group=*/"Common/Limits",
        /*unit=*/"deg",
        /*description=*/"Minimum software position limit.",
        /*min_value=*/motion_core::ParameterValue::from_signed(-2147483648),
        /*max_value=*/motion_core::ParameterValue::from_signed(2147483647)
    },
    {
        /*id=*/motion_core::make_parameter_id(motion_core::CommonParameter::LimitsSoftwareMaxDeg),
        /*rdt_key=*/"limits.software_limit_max_deg",
        /*index=*/0x607D,
        /*sub_index=*/2,
        /*data_size=*/4,
        /*type=*/motion_core::ParameterValueType::SignedInteger,
        /*is_read_only=*/false,
        /*name=*/"Software position limit (max)",
        /*group=*/"Common/Limits",
        /*unit=*/"deg",
        /*description=*/"Maximum software position limit.",
        /*min_value=*/motion_core::ParameterValue::from_signed(-2147483648),
        /*max_value=*/motion_core::ParameterValue::from_signed(2147483647)
    }
}};

} // namespace ethercat_driver
