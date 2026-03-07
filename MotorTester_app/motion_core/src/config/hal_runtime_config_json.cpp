#include "motion_core/config/hal_runtime_config_json.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace motion_core {

void to_json(json& j, const HalBusConfigMks& b) {
    j = json{{"id", b.interface_id}, {"path", b.device_path}, {"baud", b.baud_rate}};
}
void from_json(const json& j, HalBusConfigMks& b) {
    b.interface_id = j.at("id").get<std::string>();
    b.device_path = j.at("path").get<std::string>();
    b.baud_rate = j.at("baud").get<uint32_t>();
}

void to_json(json& j, const HalBusConfigEthercat& b) {
    j = json{{"iface", b.interface_name}};
}
void from_json(const json& j, HalBusConfigEthercat& b) {
    b.interface_name = j.at("iface").get<std::string>();
}

void to_json(json& j, const HalAxisRuntimeEntry& e) {
    j = json{
        {"id", e.axis_id.value},
        {"name", e.axis_name.value},
        {"transport", static_cast<int>(e.transport)},
        {"bus", e.bus_ref},
        {"addr", e.transport_address},
        {"conf", e.config_file},
        {"en", e.enable_on_start}
    };
}
void from_json(const json& j, HalAxisRuntimeEntry& e) {
    e.axis_id.value = j.at("id").get<uint16_t>();
    e.axis_name.value = j.at("name").get<std::string>();
    e.transport = static_cast<AxisTransportKind>(j.at("transport").get<int>());
    e.bus_ref = j.at("bus").get<std::string>();
    e.transport_address = j.at("addr").get<uint16_t>();
    e.config_file = j.at("conf").get<std::string>();
    e.enable_on_start = j.at("en").get<bool>();
}

Result<HalRuntimeConfig> load_hal_runtime_config_from_file(const std::string& path) {
    try {
        std::ifstream f(path);
        if (!f.is_open()) {
            return Result<HalRuntimeConfig>::failure({ErrorCode::NotFound, "failed to open hal config file"});
        }
        json j;
        f >> j;
        
        HalRuntimeConfig cfg;
        cfg.version = j.at("version").get<uint32_t>();
        cfg.runtime.dispatch_period_ms = j.at("runtime").at("period").get<uint32_t>();
        cfg.runtime.telemetry_period_ms = j.at("runtime").at("telemetry").get<uint32_t>();
        
        if (j.contains("mks_buses")) cfg.mks_buses = j.at("mks_buses").get<std::vector<HalBusConfigMks>>();
        if (j.contains("ecat_buses")) cfg.ethercat_buses = j.at("ecat_buses").get<std::vector<HalBusConfigEthercat>>();
        if (j.contains("axes")) cfg.axes = j.at("axes").get<std::vector<HalAxisRuntimeEntry>>();
        
        return Result<HalRuntimeConfig>::success(std::move(cfg));
    } catch (...) {
        return Result<HalRuntimeConfig>::failure({ErrorCode::ProtocolFailure, "hal json parse error"});
    }
}

Result<void> save_hal_runtime_config_to_file(const std::string& path, const HalRuntimeConfig& config) {
    try {
        json j;
        j["version"] = config.version;
        j["runtime"]["period"] = config.runtime.dispatch_period_ms;
        j["runtime"]["telemetry"] = config.runtime.telemetry_period_ms;
        j["mks_buses"] = config.mks_buses;
        j["ecat_buses"] = config.ethercat_buses;
        j["axes"] = config.axes;
        
        std::ofstream f(path);
        if (!f.is_open()) {
            return Result<void>::failure({ErrorCode::InternalError, "failed to write hal config file"});
        }
        f << j.dump(4);
        return Result<void>::success();
    } catch (...) {
        return Result<void>::failure({ErrorCode::InternalError, "failed to serialize hal config"});
    }
}

} // namespace motion_core
