#include "mks/adapter/mks_runtime_factory.h"
#include "mks/internal/port/gs_usb_can_port.h"
#include "ethercat/manager/ethercat_runtime_factory.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/runtime_factory_registry.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

void printUsage() {
    std::cout << "mks_can_cli usage:\n"
                 "  mks_can_cli --list\n"
                 "  mks_can_cli --runtime-config <path.json> --control-status [--id <axis_id>]\n"
                 "  mks_can_cli --device usb:BUS:ADDR --id <can_id> --control-status\n"
                 "\n"
                 "Note: direct IAxis + BusManager runtime path is used.\n";
}

} // namespace

int main(int argc, char* argv[]) {
    motion_core::RuntimeFactoryRegistry::register_factory(
        motion_core::AxisTransportKind::CanBus,
        [](const motion_core::HalRuntimeConfig& cfg) { return mks::build_mks_runtime(cfg); }
    );
    motion_core::RuntimeFactoryRegistry::register_factory(
        motion_core::AxisTransportKind::Ethercat,
        [](const motion_core::HalRuntimeConfig& cfg) { return ethercat_driver::build_ethercat_runtime(cfg); }
    );

    if (argc < 2) {
        printUsage();
        return 0;
    }

    std::vector<std::string> args;
    args.reserve(static_cast<std::size_t>(argc));
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    bool list_only = false;
    bool do_control_status = false;
    bool has_explicit_id = false;

    int can_id = 1;
    int baud_rate = 1000000;
    std::string device_path;
    std::string runtime_config_path;

    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--list") {
            list_only = true;
        } else if ((args[i] == "--runtime-config" || args[i] == "--config") && i + 1 < args.size()) {
            runtime_config_path = args[++i];
        } else if (args[i] == "--device" && i + 1 < args.size()) {
            device_path = args[++i];
        } else if (args[i] == "--id" && i + 1 < args.size()) {
            has_explicit_id = true;
            can_id = std::atoi(args[++i].c_str());
        } else if ((args[i] == "--baud" || args[i] == "--bitrate") && i + 1 < args.size()) {
            baud_rate = std::atoi(args[++i].c_str());
        } else if (args[i] == "--control-status") {
            do_control_status = true;
        }
    }

    if (list_only) {
        std::cout << "sim:default : Virtual Simulator CAN Device\n";
        const auto devs = mks::GsUsbCanPort::enumerateDevices();
        for (const auto& d : devs) {
            std::cout << d.path << " : " << d.description << "\n";
        }
        return 0;
    }

    if (!do_control_status) {
        std::cerr << "Only --control-status mode is supported\n";
        printUsage();
        return 2;
    }

    if (device_path.empty() && runtime_config_path.empty()) {
        std::cerr << "--device or --runtime-config is required\n";
        printUsage();
        return 2;
    }

    motion_core::HalRuntime runtime{};

    if (!runtime_config_path.empty()) {
        const auto config_result = motion_core::load_hal_runtime_config_from_file(runtime_config_path);
        if (!config_result.ok()) {
            std::cerr << "Runtime config load failed: " << config_result.error().message << "\n";
            return 12;
        }

        const auto runtime_result = runtime.open_from_config(config_result.value());
        if (!runtime_result.ok()) {
            std::cerr << "Runtime open failed: " << runtime_result.error().message << "\n";
            return 13;
        }
    } else {
        motion_core::HalRuntimeConfig cfg{};
        motion_core::HalBusConfigMks bus{};
        bus.interface_id = "cli_mks_bus_0";
        bus.device_path = device_path;
        bus.baud_rate = static_cast<std::uint32_t>(baud_rate);
        cfg.mks_buses.push_back(bus);

        motion_core::HalAxisRuntimeEntry axis{};
        axis.axis_id = motion_core::AxisId{static_cast<std::uint16_t>(can_id)};
        axis.axis_name = motion_core::AxisName{"mks_axis_" + std::to_string(can_id)};
        axis.transport = motion_core::AxisTransportKind::CanBus;
        axis.bus_ref = bus.interface_id;
        axis.transport_address = static_cast<std::uint16_t>(can_id);
        axis.enable_on_start = true;
        cfg.axes.push_back(axis);

        const auto runtime_result = runtime.open_from_config(cfg);
        if (!runtime_result.ok()) {
            std::cerr << "Runtime open failed: " << runtime_result.error().message << "\n";
            return 14;
        }
    }

    std::vector<motion_core::AxisId> target_axes;
    if (has_explicit_id) {
        target_axes.push_back(motion_core::AxisId{static_cast<std::uint16_t>(can_id)});
    } else {
        const auto listed = runtime.list_axes();
        if (!listed.ok()) {
            std::cerr << "List axes failed: " << listed.error().message << "\n";
            return 15;
        }
        for (const auto& info : listed.value()) {
            target_axes.push_back(info.id);
        }
        std::sort(target_axes.begin(), target_axes.end(), [](const motion_core::AxisId lhs, const motion_core::AxisId rhs) {
            return lhs.value < rhs.value;
        });
    }

    const auto start_runtime = runtime.start();
    if (!start_runtime.ok()) {
        std::cerr << "Runtime start failed: " << start_runtime.error().message << "\n";
        return 16;
    }

    std::vector<motion_core::AxisId> started_axes;
    for (const auto axis_id : target_axes) {
        const auto axis_res = runtime.find_axis(axis_id.value);
        if (!axis_res.ok()) {
            continue;
        }
        started_axes.push_back(axis_id);
    }

    if (started_axes.empty()) {
        std::cerr << "No axis was started via control service\n";
        (void)runtime.stop();
        (void)runtime.close();
        return 10;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    for (int i = 0; i < 5; ++i) {
        for (const auto axis_id : started_axes) {
            const auto axis_res = runtime.find_axis(axis_id.value);
            if (!axis_res.ok()) {
                continue;
            }
            const auto telemetry_result = axis_res.value()->read_telemetry();
            if (!telemetry_result.ok()) {
                std::cerr << "Telemetry failed for axis " << axis_id.value << ": "
                          << telemetry_result.error().message << "\n";
                continue;
            }

            const auto& telemetry = telemetry_result.value();
            std::cout << "telemetry axis=" << axis_id.value
                      << " pos_deg=" << telemetry.actual_position_deg
                      << " vel_deg_s=" << telemetry.actual_velocity_deg_per_sec
                      << " status_word=" << telemetry.status_word
                      << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    (void)runtime.stop();
    (void)runtime.close();

    return 0;
}
