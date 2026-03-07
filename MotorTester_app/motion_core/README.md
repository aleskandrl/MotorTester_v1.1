# motion_core

Qt-free industrial core for multi-axis control systems.

## Design Rules

- No Qt dependencies in core (`STL only`).
- Explicit error handling via `Result<T>` and `ErrorCode`.
- Strongly-typed domain model (`AxisId`, `AxisMode`, `AxisTelemetry`, ...).
- No magic numeric parameter identifiers in public APIs.
- Protocol parameters are represented through enums:
  - `CommonParameter`
  - `EthercatParameter`
  - `MksParameter`
- Runtime loops are encapsulated in core classes (`RuntimeLoop`), UI only calls top-level APIs.

## Main Components

- `result.h` — explicit operation result model.
- `types.h` — common domain types.
- `parameter_id.h` — enum-based parameter identifiers.
- `parameter_types.h` — parameter descriptors, value containers, patch/set models.
- `axis_interface.h` — abstract axis contract for protocol-specific adapters.
- `axis_control_service.h/.cpp` — thread-safe axis registry and unified control façade.
- `runtime_loop.h/.cpp` — encapsulated periodic loop utility.

## Integration Pattern

UI layer (Qt or any other HMI) must depend on `AxisControlService` and `IAxis` abstractions only.
Protocol-specific implementations (EtherCAT/MKS/...) remain below this layer and expose no Qt types.

## MKS Adapter Integration (example)

The repository now contains a Qt-free adapter implementation:

- Bus manager header/source: `MotorTester_app/drivers/interfaces/mks/manager/mks_can_bus_manager.h`, `MotorTester_app/drivers/interfaces/mks/manager/mks_can_bus_manager.cpp`
- Header: `MotorTester_app/drivers/interfaces/mks/adapter/mks_axis_adapter.h`
- Source: `MotorTester_app/drivers/interfaces/mks/adapter/mks_axis_adapter.cpp`

Usage pattern:

1. Create one `MksCanBusManager` per physical GS-USB interface (`make_mks_can_bus_manager`).
2. Build `MksAxisAdapterConfig` with explicit axis id/name, `can_id`, and shared `bus_manager`.
3. Create adapter via `make_mks_axis_adapter(config)`.
4. Register adapters in `AxisControlService`.
5. Start axis (`start_axis`) and read telemetry / apply commands / patch parameters through the service API.

Runtime loop and CAN access are centralized at bus level (`MksCanBusManager`), so axes do not open CAN individually.
No Qt types are used in this stack.
