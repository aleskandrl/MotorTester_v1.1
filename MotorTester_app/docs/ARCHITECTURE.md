# ARCHITECTURE (ACTUAL)

Документ фиксирует текущее состояние `MotorTester_app` после введения канонического headless runtime-модуля.

## 1. Цель

Система управляет приводами по двум транспортам:

- MKS CAN
- EtherCAT (IgH / ecrt)

Клиенты:

- `motor_tester_gui` (Qt)
- `mks_can_cli` (CLI)

## 2. Канонический runtime-path

### Единый headless путь (source of truth)

```text
Qt GUI / CLI
  -> motion_core::HalRuntime
    -> RuntimeFactoryRegistry
      -> RuntimeBuildResult { buses, axes }
        -> IBusManager (MKS/EtherCAT)
        -> IAxis adapters (MKS/EtherCAT)

Qt GUI scan
  -> AxisManager::scan*
    -> motion_core::HalRuntime::scan_*_topology
      -> RuntimeFactoryRegistry topology scanners
        -> transport discovery (MKS/EtherCAT)
```

`HalRuntime` является единым API для lifecycle/config/control-path.

### Qt GUI

```text
AxisWorkspace / MainWindow
  -> AxisManager (thin Qt bridge)
    -> motion_core::HalRuntime
```

### CLI

```text
apps/cli/main.cpp
  -> motion_core::HalRuntime
```

`AxisControlService` удалён из production path.

## 3. Слои

### motion_core (Qt-free)

- `IAxis`, `IBusManager`
- `RuntimeFactoryRegistry`
- `HalRuntime` (канонический headless orchestration)
- `RuntimeLoop`
- `Result<T>`, типы осей/телеметрии/параметров

### drivers/interfaces/mks

- `MksCanBusManager`
- `MksAxisAdapter : IAxis`
- `mks_runtime_factory`
- `mks_dictionary`

### drivers/interfaces/ethercat

- `EthercatBusManager`
- `EthercatAxisAdapter : IAxis`
- `ethercat_runtime_factory`
- `p100e_ethercat_dictionary`

### ui/qt/mks

- `AxisManager`: Qt bridge + safety/policy + сигналы/слоты UI
- `AxisWorkspace`: HMI, команды, телеметрия, parameter tree

## 4. Runtime responsibilities

### HalRuntime

- `open_from_config/start/stop/close`
- `scan_mks_topology/scan_ethercat_topology` (headless scan API)
- `find_axis/list_axes`
- доступ к bus snapshot для статистики
- `export_axis_config_to_file/apply_axis_config_file`

### AxisManager

- делегирование runtime lifecycle в `HalRuntime`
- делегирование scan в `HalRuntime` (без transport-specific includes/calls в UI)
- safety baseline после критичных операций
- bridge к UI (signals/slots, timers, QVariant адаптация)

## 5. Safety / policy

- Единый safety baseline применяется в `AxisManager` после критичных операций.
- Runtime lifecycle выполняется через `HalRuntime`, вызываемый из `AxisManager`/CLI.
- MKS `0x8C` (`SlaveRespond/Active`) policy-locked: запись через patch API запрещена, чтобы исключить отключение ответов привода и потерю телеметрии.
- Raw command UI-path деактивирован в unified runtime (только typed API/parameter tree).

## 6. Параметры и конфиги

- Источники параметров: dictionary-слои MKS/EtherCAT.
- UI строит parameter tree через `IAxis::list_parameters()`.
- Export/import осевого конфига выполняется через `HalRuntime` (persistable + writable policy).
- По умолчанию в GUI используются текущие считанные значения с привода; auto-apply осевого конфига при scan/open/start не выполняется.
- `HalRuntimeConfig` остаётся master bootstrap config для mixed runtime.

## 7. Command truth-source

Канонический источник соответствия команд и семантики:

- `docs/mks_protocol_description.md`
- вендорная документация (PDF/ESI)

Любое расхождение между деревом команд/UI и документацией трактуется как дефект реализации.

## 8. Командная адресация в runtime (MKS)

- Для CAN-native runtime используется 11-bit адресация CAN ID: `0x001..0x7FF`.
- Команда `8B` (Set CAN ID): writable runtime-команда, но non-persistable в `AxisConfig`.
- Команда `8D` (Set Group ID): диапазон runtime `0x001..0x7FF`.
