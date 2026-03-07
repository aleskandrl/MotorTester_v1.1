# ARCHITECTURE (CANONICAL, ACTUAL)

Документ фиксирует **текущую** архитектуру `MotorTester_v1` и только актуальные runtime-path.
Исторические заметки и длинный журнал изменений должны вестись отдельно (например, в `ARCHITECTURE_LOG.md`).

---

## 1) Назначение

`MotorTester_v1` — стенд управления и диагностики осей по двум транспортам:

- **MKS CAN**
- **EtherCAT (IgH / ecrt)**

Клиентские входы:

- **Qt GUI**: `motor_tester_gui`
- **CLI**: `mks_can_cli`

Ключевой инвариант: верхний уровень работает через единый осевой контракт `motion_core::IAxis`.

---

## 2) Актуальные runtime-path

### 2.1 Qt path (production)

```text
MainWindow / AxisWorkspace
  -> AxisManager (QThread)
    -> motion_core::AxisControlService
      -> IAxis adapters
        -> MksCanBusManager | EthercatBusManager
```

### 2.2 CLI path (актуальный)

```text
apps/cli/main.cpp
  -> motion_core::AxisControlService
    -> IAxis adapters
      -> MksCanBusManager
```

### 2.3 Legacy path

- Legacy-слой `AxisOrchestrator` удалён из `motion_core`.
- Для GUI/CLI поддерживается единый control-path через `motion_core::AxisControlService`.

### 2.4 Configuration path (Phase 1-2)

- **Master Bootstrap Config (`HalRuntimeConfig`)**: Описывает топологию шин и базовую адресацию (CAN ID, EtherCAT station). Используется для открытия рантайма.
- **Runtime Axis Config (`AxisConfig`)**: Полный снимок прикладных параметров (PID, Gear Ratio, Limits). Применяется к уже обнаруженным осям.

---

## 3) Слои системы

### 3.1 `motion_core` (Qt-independent)

Содержит:

- `IAxis`
- `AxisControlService` (единственная точка входа API, оркестрация рантайма, команды, телеметрия, параметры, motion queue)
- `RuntimeLoop`
- `Result<T>`, `ErrorCode`, типы оси/параметров/телеметрии

Инвариант: `motion_core` не знает о Qt и не зависит от деталей транспортного протокола.

### 3.2 MKS stack (`drivers/interfaces/mks`)

- `ICanPort` + реализации (`GsUsbCanPort`, `SimCanPort`)
- `MksProtocol`
- `MksCanBusManager`
- `MksAxisAdapter : IAxis`
- `mks_runtime_config` / `mks_runtime_factory`

### 3.3 EtherCAT stack (`drivers/interfaces/ethercat`)

- `EthercatBusManager`
- `EthercatAxisAdapter : IAxis`
- `ethercat_runtime_factory`

### 3.4 Application layer

- `apps/qt/qt_main.cpp`, `ui/qt/mks/*`
- `apps/cli/main.cpp`
- `AxisManager` как bridge между UI и `AxisControlService`
- Deprecated API-методы `applyBasicConfig/readBasicConfig/changeCanId` удалены из `AxisManager`.

---

## 4) Runtime особенности

### 4.1 MKS CAN

`MksCanBusManager`:

- владеет портом и протоколом;
- обслуживает async-команды (очередь, priority path для emergency);
- выполняет sync-транзакции параметров;
- выполняет polling-цикл телеметрии.

Актуальная защита от конкуренции sync/RT:

- на время sync-транзакции активируется `sync_transaction_active_`;
- `poll_cycle()` пропускает I/O при активной sync-транзакции (изоляция sync path от RT poll path).

### 4.2 EtherCAT

`EthercatBusManager`:

- `open/scan/start_runtime/stop_runtime`;
- циклический обмен PDO через зарегистрированные адаптеры.

`AxisManager`:

- scan-flow строит runtime поверх уже открытого bus-manager;
- дублирующий `startRuntime()` при уже запущенных осях пропускается (idempotent guard).

---

## 5) Потоки и синхронизация

Потоки:

- `T_UI`: Qt main thread
- `T_MANAGER`: поток `AxisManager`
- `T_BUS_MKS`: runtime loop в `MksCanBusManager`
- `T_BUS_ECAT`: runtime loop в `EthercatBusManager`
- `T_AXIS_TRAJ`: trajectory thread в `AxisWorkspace`

Основные примитивы:

- `MksCanBusManager`: `state_mutex_`, `queue_mutex_`, `io_mutex_`, `sync_mutex_`
- `AxisControlService`: внутренний mutex для реестра осей и queue
- `AxisWorkspace`: `trajectory_mutex_` + atomics

---

## 6) Команды и параметры

- Командный API: через `AxisControlService` (`enable_axis`, `set_axis_mode`, `submit_command`, motion queue API)
- Параметрический API: `list_parameters/read_parameters/apply_parameter_patch` через `IAxis`
- Safety baseline: реализован в `AxisControlService`, вызывается из `AxisManager` после критичных операций

---

## 7) Build

- C++20, GCC required
- Таргеты: `mks_can_cli`, `motor_tester_gui`, `motion_core`
- `mks_can_cli` не использует Qt UI include-директорию

---

## 8) Что намеренно исключено из этого файла

- длинный исторический changelog;
- временные аудиторские заметки;
- устаревшие/экспериментальные execution-path без пометки legacy.

Этот файл описывает только текущее состояние архитектуры.

---

## 9) Актуальные остаточные риски

- Для MKS под нагрузкой возможны деградации telemetry/джиттер из-за ограничений bandwidth и общего I/O ресурса;
- В UI есть отдельный trajectory thread в `AxisWorkspace`, параллельный core motion queue — требуется дальнейшая унификация.


---

## 10) Каноничность

`ARCHITECTURE.md` — каноничное описание текущей архитектуры.
При изменении runtime-path документ обновляется синхронно с кодом.
