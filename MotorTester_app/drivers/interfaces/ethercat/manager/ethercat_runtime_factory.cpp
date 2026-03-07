#include "ethercat/manager/ethercat_runtime_factory.h"

#include "ethercat/adapter/ethercat_axis_adapter.h"

#include <unordered_set>

namespace ethercat_driver {

namespace {

motion_core::Result<EthercatRuntimeBuildResult> build_axes_on_bus(
    const std::shared_ptr<EthercatBusManager>& bus_manager,
    const std::vector<EthercatAxisRuntimeConfig>& axes,
    const bool close_bus_on_failure) {
    if (!bus_manager) {
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime has null bus manager"});
    }
    if (axes.empty()) {
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime config has no axes"});
    }

    EthercatRuntimeBuildResult out{};
    out.bus_manager = bus_manager;

    auto fail_and_close = [&out, close_bus_on_failure](const motion_core::Error& error)
        -> motion_core::Result<EthercatRuntimeBuildResult> {
        if (close_bus_on_failure && out.bus_manager) {
            (void)out.bus_manager->close();
        }
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(error);
    };

    std::unordered_set<std::uint16_t> axis_ids_seen;
    out.axes.reserve(axes.size());

    for (const auto& axis_cfg : axes) {
        if (!axis_cfg.axis_id.valid()) {
            return fail_and_close(
                {motion_core::ErrorCode::InvalidArgument, "axis_id must be > 0"});
        }
        if (axis_cfg.axis_name.value.empty()) {
            return fail_and_close(
                {motion_core::ErrorCode::InvalidArgument, "axis_name cannot be empty"});
        }

        const auto axis_id_raw = axis_cfg.axis_id.value;
        if (!axis_ids_seen.insert(axis_id_raw).second) {
            return fail_and_close(
                {motion_core::ErrorCode::AlreadyExists, "duplicate axis_id in ethercat runtime config"});
        }

        const auto axis_info_result = out.bus_manager->get_slave_bus_info(axis_id_raw);
        if (!axis_info_result.ok()) {
            return fail_and_close(
                {motion_core::ErrorCode::NotFound, "axis_id from config is not present on scanned EtherCAT bus"});
        }

        EthercatAxisAdapterConfig adapter_cfg{};
        adapter_cfg.axis_id = axis_cfg.axis_id;
        adapter_cfg.axis_name = axis_cfg.axis_name;
        adapter_cfg.ecat_axis_index = axis_info_result.value().runtime_index;
        adapter_cfg.ecat_bus_position = axis_info_result.value().bus_position;
        adapter_cfg.bus_manager = out.bus_manager;

        auto adapter = make_ethercat_axis_adapter(std::move(adapter_cfg));
        auto config_res = adapter->configure_hardware();
        if (!config_res.ok()) {
            return fail_and_close(config_res.error());
        }
        
        out.axes.push_back(std::move(adapter));
    }

    return motion_core::Result<EthercatRuntimeBuildResult>::success(std::move(out));
}

} // namespace

motion_core::Result<EthercatRuntimeBuildResult> build_ethercat_runtime_on_existing_bus(
    const std::shared_ptr<EthercatBusManager>& bus_manager,
    const std::vector<EthercatAxisRuntimeConfig>& axes) {
    return build_axes_on_bus(bus_manager, axes, false);
}

motion_core::Result<EthercatRuntimeBuildResult> build_ethercat_runtime(const EthercatRuntimeConfig& config) {
    if (config.bus.interface_name.empty()) {
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(
            {motion_core::ErrorCode::InvalidArgument, "ethercat runtime config has empty interface_name"});
    }

    auto bus_manager = make_ethercat_bus_manager(config.bus);
    const auto open_result = bus_manager->open();
    if (!open_result.ok()) {
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(open_result.error());
    }

    const auto discovered = bus_manager->scan_axes();
    if (!discovered.ok()) {
        (void)bus_manager->close();
        return motion_core::Result<EthercatRuntimeBuildResult>::failure(discovered.error());
    }

    return build_axes_on_bus(bus_manager, config.axes, true);
}

} // namespace ethercat_driver
