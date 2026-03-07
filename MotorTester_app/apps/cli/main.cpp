#include "mks/adapter/mks_axis_adapter.h"
#include "mks/adapter/mks_runtime_config.h"
#include "mks/adapter/mks_runtime_factory.h"
#include "mks/manager/mks_can_bus_manager.h"
#include "mks/port/gs_usb_can_port.h"
#include "motion_core/axis_control_service.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
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
                 "Note: Only control logic via AxisControlService is supported.\n";
}

} // namespace

int main(int argc, char* argv[]) {
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

    motion_core::AxisControlService control_service;
    std::vector<std::shared_ptr<mks::MksCanBusManager>> bus_managers;

    if (!runtime_config_path.empty()) {
        const auto config_result = mks::load_mks_runtime_config_from_file(runtime_config_path);
        if (!config_result.ok()) {
            std::cerr << "Runtime config load failed: " << config_result.error().message << "\n";
            return 12;
        }

        const auto runtime_result = mks::build_mks_runtime(config_result.value());
        if (!runtime_result.ok()) {
            std::cerr << "Runtime build failed: " << runtime_result.error().message << "\n";
            return 13;
        }

        bus_managers = runtime_result.value().bus_managers;
        for (const auto& axis : runtime_result.value().axes) {
            const auto register_result = control_service.add_axis(axis);
            if (!register_result.ok()) {
                std::cerr << "ControlService register failed: " << register_result.error().message << "\n";
                return 9;
            }
        }
    } else {
        mks::MksCanBusConfig bus_config{};
        bus_config.device_path = device_path;
        bus_config.baud_rate = static_cast<unsigned int>(baud_rate);
        bus_config.cycle_time = std::chrono::milliseconds(4);
        auto bus_manager = mks::make_mks_can_bus_manager(std::move(bus_config));
        bus_managers.push_back(bus_manager);

        mks::MksAxisAdapterConfig axis_config{};
        axis_config.axis_id = motion_core::AxisId{static_cast<std::uint16_t>(can_id)};
        axis_config.axis_name = motion_core::AxisName{"mks_axis_" + std::to_string(can_id)};
        axis_config.can_id = static_cast<std::uint16_t>(can_id);
        axis_config.bus_manager = bus_manager;
        axis_config.auto_start_bus_manager = true;
        axis_config.axis_units_per_degree = 16384.0 / 360.0;

        auto axis = mks::make_mks_axis_adapter(std::move(axis_config));
        const auto register_result = control_service.add_axis(axis);
        if (!register_result.ok()) {
            std::cerr << "ControlService register failed: " << register_result.error().message << "\n";
            return 9;
        }
    }

    std::vector<motion_core::AxisId> target_axes;
    if (has_explicit_id) {
        target_axes.push_back(motion_core::AxisId{static_cast<std::uint16_t>(can_id)});
    } else {
        const auto list_result = control_service.list_axes();
        if (!list_result.ok()) {
            std::cerr << "ControlService list failed: " << list_result.error().message << "\n";
            return 14;
        }
        for (const auto& axis_info : list_result.value()) {
            target_axes.push_back(axis_info.id);
        }
        std::sort(target_axes.begin(), target_axes.end(), [](const motion_core::AxisId lhs, const motion_core::AxisId rhs) {
            return lhs.value < rhs.value;
        });
    }

    (void)control_service.start_dispatch_loop(std::chrono::milliseconds(4));

    std::vector<motion_core::AxisId> started_axes;
    for (const auto axis_id : target_axes) {
        const auto start_result = control_service.start_axis(axis_id);
        if (!start_result.ok()) {
            std::cerr << "ControlService start failed for axis " << axis_id.value << ": " << start_result.error().message << "\n";
            continue;
        }
        started_axes.push_back(axis_id);
    }

    if (started_axes.empty()) {
        std::cerr << "No axis was started via control service\n";
        for (const auto& bus_manager : bus_managers) {
            (void)bus_manager->stop();
        }
        return 10;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    for (int i = 0; i < 5; ++i) {
        for (const auto axis_id : started_axes) {
            const auto telemetry_result = control_service.read_telemetry(axis_id);
            if (!telemetry_result.ok()) {
                std::cerr << "ControlService telemetry failed for axis " << axis_id.value << ": "
                          << telemetry_result.error().message << "\n";
                continue;
            }

            const auto& telemetry = telemetry_result.value();
            std::cout << "ControlService telemetry axis=" << axis_id.value
                      << " pos_deg=" << telemetry.actual_position_deg
                      << " vel_deg_s=" << telemetry.actual_velocity_deg_per_sec
                      << " status_word=" << telemetry.status_word
                      << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (const auto axis_id : started_axes) {
        (void)control_service.stop_axis(axis_id);
    }

    (void)control_service.stop_dispatch_loop();

    return 0;
}
