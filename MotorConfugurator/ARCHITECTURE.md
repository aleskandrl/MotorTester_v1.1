## Update — 2026-04-28 05:52 Asia/Tbilisi

### Deep safety audit report added for MKS/MS motor-control path after recent MKS control-panel refactor

По результатам глубокого аудита добавлен отдельный инженерный отчёт по safety-рискaм в цепочке:

- `UI / MKS control pages`
- `AxisManager`
- `RuntimeQueueIngress`
- `HalRuntime`
- `MksAxisAdapter / MksAxisWorker / MksCanBusManager`

#### Что зафиксировано

1. Добавлен новый документ:
   - `docs/SAFETY_AUDIT_MKS_MS_2026-04-28.md`

2. Отчёт не меняет runtime behavior сам по себе, а документирует найденные потенциально опасные дефекты, которые могут привести к неконтролируемому поведению моторов.

3. Основной фокус аудита:
   - hard safety gate на `service` и `motion` путях,
   - поведение `E-STOP / Disable / Stop / Runtime stop / Rebuild`,
   - согласованность единиц измерения в MKS motion path,
   - корректность MKS homing completion semantics,
   - порядок startup baseline / config apply / enable-on-start.

#### Архитектурный смысл

- Аудит зафиксировал, что после добавления MKS control-panel в системе появились риски не только в UI, но и на уровне orchestration contracts.
- Критично, что некоторые safety-invariants сейчас обеспечиваются частично UI-политикой, а не жёстким ingress/runtime gate.
- Документ нужен как инженерная база для точечных safety-fixes без лишней переработки архитектуры.

## Update — 2026-04-28 05:05 Asia/Tbilisi

### Added MKS-only all-axes control workspace in the same MDI area, with sequential homing UI orchestration and UI conflict lock

По новому требованию добавлена отдельная **MKS-only control page** для управления всеми MKS-осями сразу, без ломки существующих single-axis workspace.

#### Что изменено

1. **Добавлена новая MDI workspace-страница `MksAllAxesControlWorkspace`**
   - Новые файлы:
     - `ui/qt/mks/mks_all_axes_control_workspace.h`
     - `ui/qt/mks/mks_all_axes_control_workspace.cpp`
   - Страница открывается в том же `QMdiArea`, что и обычные workspace осей.
   - Это намеренно сделано как отдельный UI path, чтобы не усложнять `AxisWorkspace` и не смешивать single-axis UX с multi-axis UX.

2. **Новая страница встраивается через topology tree рядом с обычными MKS осями**
   - В `MainWindow::updateTopologyTree(...)` под `MKS CAN Bus` теперь добавляется отдельный узел:
     - `MKS Control Page`
   - Double-click по нему открывает новое MDI-окно, аналогично тому, как сейчас открываются per-axis workspace.
   - То есть навигационная модель не менялась: всё по-прежнему открывается через существующий workspace/MDI механизм.

3. **Страница показывает только базовый control UX по всем MKS осям**
   - Для каждой оси отображаются:
     - текущая позиция,
     - статус,
     - slider target,
     - кнопки `Enable`, `Disable`, `Home`.
   - Сверху добавлены global действия:
     - `Enable All`,
     - `Disable All`,
     - `Start Homing Sequence`,
     - `Stop Sequence`.
   - Управление использует уже существующие `AxisManager::enqueueServiceBatch(...)` и `AxisManager::enqueueMotionBatch(...)` paths — без нового runtime control API.

4. **Последовательный homing реализован как UI-level orchestration только для MKS**
   - Первая версия сознательно ограничена только MKS-path.
   - Причина прагматичная: у MKS уже есть готовый telemetry signal:
     - `mks_homing_sequence_status`
   - Новая страница запускает homing по `axis_id` по порядку:
     1. отправляет `Home` текущей оси,
     2. ждёт `Completed`,
     3. выдерживает паузу `1 сек`,
     4. запускает следующую ось.
   - При `Failed` sequence останавливается.
   - `Stop Sequence` завершает orchestration и может отправить `Disable` активной оси как safe abort UI policy.

5. **Конфликты между общей MKS page и обычными MKS workspace погашены через простой UI lock**
   - В `AxisManager` добавлен минимальный UI-level флаг:
     - `mks_homing_sequence_ui_lock_active_`
   - Он публикуется в `hostStateUpdated(...)` как:
     - `mks_homing_sequence_active`
   - Пока общий MKS homing sequence активен:
     - новая общая страница блокирует свои manual controls;
     - обычные `MksAxisWorkspace` тоже отключают свои manual controls через уже существующий `onHostStateUpdated(...)` path.
   - Это intentionally UI-only policy, а не новый runtime lock manager.

6. **Решение осталось least-invasive**
   - Не менялись:
     - parameter tree,
     - runtime factory model,
     - transport driver contracts,
     - single-axis workspace structure.
   - Все команды идут через уже существующие queue ingress paths.
   - Логика последовательного homing не поднималась в runtime и не добавляла новые abstraction layers.

7. **В MKS Control Page добавлены `Set Zero` / `Set Zero All` / `E-STOP ALL`**
   - По дополнительному требованию босса новая общая MKS страница получила:
     - per-axis `Set Zero`,
     - global `SET ZERO ALL`,
     - global `E-STOP ALL`.
   - `Set Zero` и `Set Zero All` используют уже существующий `ServiceCommandType::SetZero` path.
   - `E-STOP ALL` использует тот же глобальный emergency-stop path, что и остальные E-STOP кнопки в системе.
   - Архитектурное правило теперь жёстко зафиксировано: **любой E-STOP из UI должен немедленно останавливать все оси**, а не только локальную ось/страницу.
   - При этом `E-STOP ALL` остаётся доступным всегда, даже при активном homing-sequence UI lock, тогда как `Set Zero` / `Set Zero All` блокируются как обычные manual actions.

8. **В левой верхней части левой панели добавлен глобальный `E-STOP ALL`, а startup baseline теперь делает `Disable + SetZero` для всех поднятых осей**
   - В `MainWindow` в верхней части левого network dock добавлена отдельная красная кнопка `E-STOP ALL`.
   - Она использует тот же глобальный `AxisManager::emergencyStop(...)` path, что и остальные emergency-stop действия.
   - Дополнительно в `AxisManager::startRuntimeHeadless()` через существующий `applySafetyBaselineForAxis(...)` для каждой поднятой оси теперь автоматически запрашивается baseline:
     - `Disable`
     - `SetZero`
   - Это означает, что после запуска runtime все поднятые оси намеренно оказываются в disabled state с запрошенным zero-reset baseline; дальнейшее включение делается оператором явно.

#### Архитектурный смысл

- `MksAllAxesControlWorkspace` — это отдельный UI consumer поверх уже существующего `AxisManager`, а не новая control subsystem.
- Multi-axis orchestration для первой версии оставлена в UI, потому что это даёт минимальный риск и максимальную предсказуемость.
- Конфликт между несколькими UI-страницами решён не через усложнение runtime ownership, а через простой и явный UI lock flag.
- Это соответствует прагматичному подходу:
  - минимум новых сущностей,
  - reuse текущих queue/telemetry paths,
  - локальное расширение UI без перепроектирования всего HAL.

## Update — 2026-04-28 04:30 Asia/Tbilisi

### HAL model simplified around 3 first-class domains: EtherCAT, MKS CAN, HexaMotion; live transport state is no longer loaded from HAL config

По новому требованию архитектура упрощена вокруг трёх доменов верхнего уровня:

1. **EtherCAT**
2. **MKS CAN**
3. **HexaMotion**

Ключевое изменение: `HAL config` больше не считается источником истины для текущего live transport binding в UI. Теперь topology/config и live-open state разделены по смыслу.

#### Что изменено

1. **MKS и EtherCAT bus identity больше не опираются на временный physical endpoint**
   - Для MKS логический `bus_ref/interface_id` теперь нормализован к `mks_can`.
   - Для EtherCAT логический `bus_ref` нормализован к `ethercat`.
   - Временные значения вроде `usb:2:6` или Linux interface name остались transport endpoint’ами, но больше не играют роль архитектурной identity шины.

2. **`AxisManager` больше не восстанавливает opened-state из `current_hal_config_`**
   - Удалена прежняя семантика `syncOpenStateFromCurrentConfig()`, где наличие bus в json означало, что устройство будто бы уже "open".
   - Теперь opened/live state задаётся только через:
     - `openDevice(...)` для MKS CAN,
     - `openEthercatDevice(...)` для EtherCAT.

3. **Runtime теперь строится из `HAL topology + current live transport selection`**
   - В `rebuildRuntimeFromCurrentConfig()` перед `open_from_config(...)` создаётся effective runtime config:
     - для MKS в bus config подставляется текущий `opened_mks_device_path_ + baud_rate`;
     - для EtherCAT подставляется текущий `opened_ethercat_interface_`;
     - если transport сейчас не открыт, соответствующие bus/axes исключаются из effective runtime build.
   - Это устранило прежнюю поломку, когда runtime пытался открыть stale `usb:X:Y` из старого HAL json.

4. **Применение per-axis config перенесено в общий rebuild/runtime-start path**
   - Раньше `loadHalConfig()` отдельно пытался применить axis configs только если runtime уже active.
   - Теперь после успешного rebuild/start `AxisManager` проходит по `current_hal_config_.axes` и вызывает `apply_axis_config_file(...)` только для реально поднятых осей.
   - Тем самым semantics стали проще:
     - load topology,
     - rebuild runtime из live transport state,
     - apply per-axis configs к тем transport-доменам, которые реально активны.

5. **Scan path теперь сохраняет transport-domain identity и не теряет axis metadata**
   - MKS scan публикует bus как `mks_can`, а оси получают `bus_ref = mks_can`.
   - EtherCAT scan работает аналогично с `ethercat`.
   - При повторном scan старая axis metadata сохраняется по transport address, включая:
     - `config_file`,
     - `enable_on_start`,
     - осмысленное `axis_name`, если оно уже было задано.

6. **Диагностические логи стали честнее по transport bind failure**
   - При `Runtime start failed after rebuild` теперь в лог дописывается текущий live endpoint:
     - MKS path + baud,
     - EtherCAT interface.
   - Это позволяет сразу видеть, какой именно transport binding реально пытались поднять.

#### Архитектурный смысл

- `HAL config` теперь описывает **topology/runtime intent**, а не текущее состояние UI transport-open.
- `MKS CAN` и `EtherCAT` трактуются как два transport-domain’а, а не как случайные physical endpoint строки.
- `HexaMotion` остаётся отдельным control/IPC domain и не смешивается с transport binding.
- Решение намеренно сделано прагматично и без новых abstraction layers:
  - без новых manager’ов и helper-обвязок,
  - с локальными правками в существующих `AxisManager`, runtime factory и topology scanner,
  - с сохранением общей структуры проекта, но с более чистой семантикой состояния.

## Update — 2026-04-28 02:25 Asia/Tbilisi

### Axis config / HAL project now store full axis parameter snapshots with readable metadata, and load applies only writable diffs

По новому требованию конфиг оси и HAL project больше не сохраняют урезанный subset параметров. Теперь система сохраняет **полный snapshot parameter tree каждой оси**, а при загрузке сначала читает текущее состояние оси и применяет только реально отличающиеся writable values.

#### Что изменено

1. **`AxisConfig` теперь хранит полный snapshot + human-readable metadata**
   - В `motion_core::AxisConfig` добавлены записи `parameter_records`.
   - Каждая запись содержит:
     - canonical `ParameterId + ParameterValue`,
     - а также `meta`:
       - `domain_name`,
       - `name`,
       - `group`,
       - `unit`,
       - `read_only`,
       - `persistable`.
   - JSON остаётся машинно-надёжным за счёт `d/i/v`, но теперь стал и читаемым для инженера.

2. **Сериализация `AxisConfig` переведена на version 2 без поломки старого формата**
   - `motion_core/src/config/axis_config_json.cpp` теперь пишет `params[]` как объекты с `meta`.
   - Loader совместим с legacy-path:
     - если в `params[]` лежат старые голые `ParameterEntry`, они всё ещё читаются.
   - Таким образом, новый формат богаче по контексту, но остаётся backward-compatible.

3. **Export AxisConfig сохраняет все прочитанные параметры оси, а не только `persistable`**
   - `HalRuntime::export_axis_config_to_file(...)` теперь:
     - читает весь `read_parameters()` snapshot,
     - сопоставляет его с descriptors,
     - сохраняет все значения,
     - дописывает metadata с уже нормализованной UI-группой.
   - Это относится и к standalone `Export AxisConfig`, и к per-axis файлам внутри HAL project.

4. **Load/import больше не льёт конфиг вслепую — строится diff against current runtime state**
   - `HalRuntime::build_axis_config_patch(...)` теперь:
     - загружает snapshot из файла,
     - читает текущее состояние оси через `read_parameters()`,
     - сравнивает значения по `ParameterId`,
     - в patch кладёт только реально отличающиеся параметры.
   - Это уменьшает лишние writes и делает загрузку предсказуемой.

5. **Применяются только writable параметры, transport-breaking идут последними**
   - Read-only параметры сохраняются в snapshot для наблюдаемости, но не применяются.
   - Transport-breaking параметры определяются по descriptor semantics:
     - `CAN ID`,
     - `CAN Bitrate Index`,
     - `Fixed address`.
   - Они не выбрасываются из snapshot, но при построении patch переносятся в конец, чтобы не ломать последовательность обычных apply-операций.

6. **HAL project save/load теперь тоже использует full snapshot path**
   - В `ui/qt/mks/axis_manager/axis_manager_config.cpp` удалена старая MKS-specific subset policy, где в project axis file сохранялись только `Common` параметры.
   - Теперь HAL project per-axis files строятся через тот же canonical path:
     - save → `HalRuntime::export_axis_config_to_file(...)`
     - load → `HalRuntime::build_axis_config_patch(...)`
   - То есть standalone AxisConfig и HAL project теперь имеют одну и ту же семантику.

7. **Дерево параметров в UI теперь наглядно разделено на `Software Defined HAL` и `Drive/...`**
   - В `AxisManager::requestListParameters(...)` группы descriptors нормализуются по transport semantics:
     - MKS `Common/*` → `Software Defined HAL/...`
     - MKS native parameters → `Drive/MKS/...`
     - EtherCAT local software helpers (`Gear Ratio`, `Home Switch To Zero Shift`) → `Software Defined HAL/...`
     - остальные EtherCAT drive-backed параметры → `Drive/EtherCAT/...`
   - В `AxisWorkspace::onParameterListReady(...)` дерево больше не плоское по строке group:
     - путь `A/B/C` реально строится как вложенные узлы.

#### Архитектурный смысл

- Конфиг оси теперь — это **полный наблюдаемый snapshot axis parameter space**, а не только «кусок того, что можно сохранить». 
- JSON одновременно выполняет две роли:
  - canonical machine format для apply,
  - инженерно-читаемый диагностический документ.
- Load path стал ближе к ожидаемой эксплуатационной семантике:
  - сначала понять текущее состояние,
  - затем применить только необходимые writable differences.
- UI-группы теперь честно отражают две разные природы параметров:
  - **software-defined HAL/runtime state**,
  - **native drive/driver parameter space**.
- Решение осталось прагматичным:
  - без смены `ParameterId` модели,
  - без новых abstraction layers,
  - с использованием существующих `list/read/apply` contracts оси как единого источника истины.

## Update — 2026-04-28 01:27 Asia/Tbilisi

### MKS HAL project axis configs are now software-only, and default gear ratio fallback is 1:1

После полевого фидбека исправлены две связанные проблемы MKS-path: legacy fallback `gear ratio = 100` и некорректный HAL project auto-load, который пытался применять в MKS ось transport-native drive parameters вместе с software/runtime tree values.

#### Что изменено

1. **Legacy default/fallback `gear ratio = 100` удалён**
   - В MKS bootstrap/runtime config path дефолтное значение теперь:
     - `software_gear_ratio = 1.0`
     - `axis_units_per_degree = 16384.0 / 360.0`
   - Это синхронизировано в:
     - `MksRuntimeConfig`
     - `MksAxisAdapterConfig`
     - `MksAxisConfigurator::Config`
     - runtime atomics configurator/adapter.
   - Таким образом, если из конфига не загружено другое значение, система живёт в нейтральной `1:1` семантике без скрытого legacy хвоста `100:1`.

2. **HAL project save для MKS теперь экспортирует только software/common parameter tree**
   - При bundle-style `Save HAL Config` MKS axis files больше не включают drive-native MKS domain параметры.
   - Для MKS в `axes/axis_<id>.json` попадают только `Common` параметры, то есть именно software/runtime tree values:
     - gear ratio,
     - invert direction,
     - telemetry sign invert,
     - limits,
     - homing offset,
     - и т.д.
   - Это оставляет HAL project config в зоне runtime/software semantics, а не неявной drive reprogramming.

3. **HAL project load для MKS больше не ломается на drive write reject**
   - Раньше auto-load строил patch, содержащий и software/common values, и MKS native drive config.
   - `MksAxisConfigurator::apply_parameter_patch()` успевал обновить internal software state, но затем падал на первом rejected native drive write (`MKS write rejected by drive`), из-за чего:
     - дерево параметров показывало новые значения,
     - а runtime sync фактически не завершался.
   - Теперь HAL load строит для MKS только software/common patch и применяет его через `apply_axis_config_patch(...)`.
   - Поэтому `gear ratio` и другие software параметры реально доходят до runtime sync path и применяются без участия native drive writes.

#### Архитектурный смысл

- Для MKS разделены две разные сущности:
  - **HAL project axis config** = software/runtime parameter tree,
  - **native drive config** = отдельная транспортная история, не часть auto-load bundle.
- Это убирает ложную полутранзакционную семантику, где UI-дерево уже показывало изменённые значения, а runtime/drive оставались на старом состоянии.
- Решение остаётся прагматичным и локальным:
  - без новых abstraction layers,
  - через уже существующие axis-config export/load/apply paths,
  - с точечной MKS-specific policy только там, где действительно есть transport-specific риск reject'а.

## Update — 2026-04-28 00:57 Asia/Tbilisi

### Config dialogs now open in the executable working directory by default

По дополнительному требованию оператора дефолтная директория для config dialogs больше не берётся из Documents/Home. Теперь все основные config-related dialogs стартуют из директории исполняемого приложения.

#### Что изменено

1. **Левая панель HAL save/load**
   - В `MainWindow` initial directory для:
     - `Load Master HAL Config`
     - `Select HAL Config Project Directory`
   - теперь берётся из:
     - `QCoreApplication::applicationDirPath()`

2. **Axis config import/export**
   - В `AxisWorkspace` initial path для:
     - `Export AxisConfig`
     - `Import AxisConfig`
   - тоже переведён на `QCoreApplication::applicationDirPath()`.

#### Практический эффект

- Окна выбора конфигов по умолчанию открываются рядом с исполняемым файлом/рабочей директорией приложения.
- Это соответствует реальному операторскому workflow, где конфиги ожидаются возле runtime/app deployment, а не в пользовательских Documents.

## Update — 2026-04-28 00:50 Asia/Tbilisi

### HAL config save/load now works as a project directory with per-axis config files, with Linux-safe non-native dialogs

По требованию оператора общий HAL config больше не сохраняется как одинокий json-файл. Теперь сохранение работает как project-style bundle: master topology/runtime config + отдельные axis config files. Заодно убран Linux-specific blackout issue у file dialogs за счёт отказа от native dialogs в этих путях.

#### Что изменено

1. **Левая панель сохраняет HAL config в директорию проекта, а не в один файл**
   - В `MainWindow` кнопка `Save Config...` теперь открывает **directory chooser**, а не `SaveFile` dialog.
   - Для совместимости с Linux dialog path использует:
     - `QFileDialog::getExistingDirectory(...)`
     - `QFileDialog::DontUseNativeDialog`
   - Выбранная директория становится root project folder.

2. **Структура сохранения теперь bundle-style**
   - `AxisManager::saveHalConfig(...)` теперь создаёт:
     - `hal_config.json`
     - `axes/axis_<id>.json`
   - Для каждой оси вызывается существующий runtime export path:
     - `HalRuntime::export_axis_config_to_file(...)`
   - После экспорта master HAL config сохраняется со ссылками `config_file` на relative paths вида:
     - `axes/axis_<id>.json`

3. **Загрузка master HAL config автоматически применяет per-axis configs**
   - `AxisManager::loadHalConfig(...)` после rebuild runtime проходит по `current_hal_config_.axes`.
   - Для каждой оси `config_file` резолвится относительно директории `hal_config.json`, если путь относительный.
   - Затем автоматически вызывается:
     - `HalRuntime::apply_axis_config_file(axis_id, resolved_path)`
   - То есть load path теперь восстанавливает не только topology/buses, но и parameter trees осей.

4. **Файловый слой стал надёжнее**
   - В `save_hal_runtime_config_to_file(...)` и `save_axis_config_to_file(...)` добавлено создание parent directories через `std::filesystem::create_directories(...)`.
   - Сообщения об ошибке теперь содержат конкретный путь, который не удалось создать/открыть.

5. **Non-native dialogs включены и для axis config import/export**
   - `AxisWorkspace` import/export dialogs тоже используют `QFileDialog::DontUseNativeDialog`.
   - Это уменьшает риск того же Linux blackout поведения в остальных config workflows.

#### Архитектурный смысл

- `hal_config.json` остаётся master bootstrap/topology документом.
- Реальные per-axis parameter trees живут отдельно в axis config files.
- `config_file` в `HalAxisRuntimeEntry` теперь реально участвует в полном save/load цикле, а не является пассивным полем.
- Решение осталось простым и прагматичным:
  - без новых manager layers,
  - с использованием уже существующих `export_axis_config_to_file` / `apply_axis_config_file` путей,
  - с минимальным изменением UI semantics ровно под реальную операционную задачу.

## Update — 2026-04-27 23:23 Asia/Tbilisi

### MKS telemetry angle sign can now be inverted per axis in driver parameter tree

Для MKS добавлен отдельный параметр driver-level telemetry correction, чтобы чинить баг осей, у которых CAN-телеметрия возвращает угол с противоположным знаком.

#### Что изменено

1. **Добавлен новый common parameter для MKS**
   - Новый id в `motion_core::CommonParameter`:
     - `TelemetryInvertPositionSign`
   - В MKS dictionary он публикуется как:
     - `Invert Telemetry Angle Sign`
     - group = `Common/Telemetry`
   - Параметр writable и persistable, поэтому автоматически входит в UI parameter tree и в axis-config import/export flow.

2. **Параметр хранится в `MksAxisConfigurator` как runtime state**
   - `MksAxisConfigurator`:
     - возвращает параметр через `read_parameters()`;
     - принимает его через `apply_parameter_patch()`;
     - отдаёт значение адаптеру через явный getter.
   - Тем самым изменение знака телеметрии происходит без новых abstraction layers и через уже существующий config path.

3. **Bootstrap path MKS runtime config умеет нести этот флаг**
   - В `MksAxisRuntimeConfig` добавлен bool:
     - `telemetry_invert_position_sign`
   - Он прокидывается в `MksAxisAdapterConfig` и дальше в configurator.
   - Даже если основной сценарий — изменение через parameter tree, bootstrap model теперь тоже согласована с новым runtime behavior.

4. **Инверсия применяется только в incoming telemetry decode path**
   - В `MksCanBusManager::process_async_rx_frame()` новый флаг применяется:
     - к decoded `position_deg` из `ReadEncoderAddition`;
     - к decoded `velocity_deg_per_sec` из `ReadMotorSpeed`.
   - Это сознательно сделано именно в драйверном telemetry path, а не в UI и не в motion_core.

#### Что НЕ изменено

- `HardwareInvertDirection` остаётся прежним command/motion-side параметром.
- Генерация motion-команд, relative normalization, software motor-angle limits и command sign semantics не менялись.
- Новый флаг исправляет только **входящую телеметрию**, то есть баг transport feedback, а не кинематику команд.

#### Архитектурный смысл

- Разделены две независимые семантики:
  - **command-side direction policy** (`HardwareInvertDirection`),
  - **feedback-side telemetry correction** (`TelemetryInvertPositionSign`).
- Это минимальный и прагматичный фикс: без новых слоёв, без special-case UI logic, с локальным изменением ровно в том месте, где raw CAN данные превращаются в canonical `TelemetrySnapshot`.

## Update — 2026-04-27 23:09 Asia/Tbilisi

### MKS homing state machine status is now exposed to UI telemetry

После первого внедрения MKS homing state machine добавлена явная индикация её внутреннего procedural state в UI, чтобы оператор видел, что последовательность ещё не завершена даже после окончания hardware-home фазы.

#### Что изменено

1. **В `MksHomingStateMachine` открыт текущий phase-status**
   - Добавлен `current_phase()`.
   - Это позволяет адаптеру публиковать не только булев `active`, но и понятный phase label.

2. **`MksAxisAdapter` теперь ведёт человекочитаемый homing sequence status**
   - Добавлен внутренний runtime status enum:
     - `Idle`
     - `HardwareHomeStart`
     - `WaitingHardwareHomeActive`
     - `WaitingHardwareHomeDone`
     - `MovingToOffset`
     - `ApplyingSetZero`
     - `Completed`
     - `Failed`
   - Этот статус синхронизируется с phase state machine и с one-shot procedural actions (`move-to-offset`, `set-zero`, failure).

3. **Статус прокинут в общий UI telemetry map**
   - `AxisManager::onFastTick()` для MKS axes теперь публикует:
     - `mks_homing_sequence_status`
   - То есть UI получает status через тот же canonical telemetry update path, без нового отдельного API.

4. **Статус отображается в `AxisWorkspace`**
   - В telemetry panel добавлено новое поле:
     - `Homing Sequence:`
   - Оно показывает procedural state именно нашей MKS homing state machine, а не только drive-level `motion_status == 5`.

#### Практический эффект

- Теперь видно различие между:
  - **аппаратным MKS homing статусом** (`Motor Status: Homing`),
  - и **внутренней driver sequence phase** (`Homing Sequence: MovingToOffset`, `ApplyingSetZero`, etc.).
- Это устраняет прежнюю операторскую двусмысленность, когда drive уже вышел из hardware-home, но state machine ещё продолжала post-home sequence.

## Update — 2026-04-27 21:59 Asia/Tbilisi

### MKS homing state machine added: hardware home -> move to configured angle -> final set-zero

По запросу босса для MKS реализован transport-specific homing orchestration без новых общих abstraction layers и без отдельного API для HexaMotion.

#### Что добавлено

1. **Новая MKS homing state machine в driver layer**
   - Добавлены файлы:
     - `drivers/interfaces/mks_can/axis/mks_homing_state_machine.h`
     - `drivers/interfaces/mks_can/axis/mks_homing_state_machine.cpp`
   - Процедура разбита на фазы:
     - `StartHardwareHome`
     - `WaitHomingStatusActive`
     - `WaitHomingStatusInactiveAtZero`
     - `CommandMoveToOffset`
     - `WaitMoveToOffset`
   - Семантика соответствует требованию:
     1. отправить аппаратный `GoHome`;
     2. дождаться завершения hardware-home;
     3. по завершении уйти в программно заданный угол;
     4. по приходу туда выполнить финальный `SetZero`.

2. **State machine встроена в `MksAxisAdapter`**
   - `ServiceCommandType::Home` больше не уходит напрямую в worker как внешний one-shot.
   - Вместо этого home запускает внутреннюю procedural state machine в `MksAxisAdapter`.
   - Во время процедуры:
     - внешние motion-команды блокируются;
     - большинство внешних service-команд тоже блокируются;
     - разрешены только safe/recovery paths (`Disable`, `ClearErrors`, `ResetDrive`).
   - Внутренние procedural actions выполняются тем же существующим contract'ом:
     - hardware home -> через internal `ServiceCommandType::Home`;
     - move-to-offset -> через internal `MotionCommandPoint(Position)`;
     - final zero -> через internal `ServiceCommandType::SetZero`.

3. **HexaMotion использует тот же самый public path**
   - Никакого отдельного HexaMotion-only homing API не добавлялось.
   - `AxisManager::executeAxisOperation()` уже маппил `hal_ipc::ControlOp::Home` в `ServiceCommandType::Home`.
   - Поскольку `RuntimeQueueIngress::enqueueServicePoint()` не режет service-команды по ownership, один и тот же MKS homing теперь можно запускать:
     - из UI,
     - из HexaMotion,
     - из любого IPC клиента,
     через единый canonical service-command path.

4. **Для MKS добавлен common config parameter `HomingOffsetDeg`**
   - В `motion_core::CommonParameter` этот id уже существовал, но MKS-path его не публиковал.
   - Теперь `MksAxisConfigurator`:
     - публикует `CommonParameter::HomingOffsetDeg` в parameter tree;
     - принимает его в `apply_parameter_patch()`;
     - хранит в runtime configurator state;
     - отдаёт в adapter/state machine как post-home target.
   - В MKS dictionary параметр отображается как:
     - `Homing Offset`
     - group = `Common/Homing`
     - unit = `deg`

5. **Детект завершения hardware-home завязан на уже существующий MKS homing status в UI**
   - В проекте уже существовало UI отображение MKS homing как `motion_status == 5` (`"Homing"`).
   - Поэтому state machine не вводит второй competing source of truth.
   - Критерий перехода из hardware-home в post-home move теперь такой:
     - home status был активен (`motion_status_code == 5`),
     - затем стал неактивен,
     - текущая позиция стабилизировалась около нуля,
     - скорость стала близка к нулю.
   - Это сохраняет согласованность между UI и runtime-процедурой.

6. **Обновлён simulator для проверки нового потока**
   - `SimCanPort` теперь эмулирует:
     - `GoHome (0x91)`;
     - `ReadGoBackToZeroStatus (0x3B)`;
     - временно активный homing state;
     - возвращение оси к нулю перед завершением home.
   - Это даёт возможность локально прогонять MKS homing flow без реального железа.

#### Что это означает архитектурно

- Homing для MKS остался **transport-specific behavior** и живёт в MKS driver path, а не в `motion_core`.
- Public orchestration contract не расширялся:
  - по-прежнему используются только `enqueueServicePoint`, `enqueueCommandPoint`, telemetry и estop.
- HexaMotion и UI используют **одну и ту же** команду `Home`, а вся сложность процедуры скрыта внутри адаптера.
- Это соответствует требованию босса:
  - без новых слоёв,
  - без helper API снаружи,
  - с максимально прямой и поддерживаемой семантикой.

## Update — 2026-04-27 20:27 Asia/Tbilisi

### MKS motor-angle limits added as driver-enforced axis config parameters

По запросу босса для MKS добавлена поддержка `motor-angle` лимитов как части axis config и common parameter tree.

#### Что добавлено

1. **Новые common parameters для MKS axis config**
   - `LimitsSoftwareMinDeg`
   - `LimitsSoftwareMaxDeg`

   В MKS dictionary они заведены как:
   - `Motor Angle Min Limit`
   - `Motor Angle Max Limit`
   - unit = `motor_deg`

2. **Поддержка import/export/read/apply через axis config path**
   - `MksAxisConfigurator::read_parameters()` теперь публикует оба лимита.
   - `MksAxisConfigurator::apply_parameter_patch()` теперь принимает оба лимита,
     валидирует их и хранит во внутреннем runtime state configurator-а.
   - За счёт общего `AxisConfig -> ParameterPatch -> apply_parameter_patch()` пути эти параметры автоматически входят в per-axis config import/export flow.

3. **Driver-enforced enforcement path**
   - В `MksAxisAdapter::enqueueCommandPoint()` после normalizing relative command в absolute target
     добавлена проверка motor-side target against configured limits.
   - Проверка делается именно в **моторных углах**:
     - `motor_target_deg = output_target_deg * gear_ratio`
     - затем учитывается `invert_direction`
   - Если target выходит за диапазон `[min_motor_deg, max_motor_deg]`, команда отклоняется до попадания в motion queue.

#### Что это означает архитектурно

- Это **не protocol-native MKS coordinate window**, а осознанный **driver-enforced limit guard**.
- Решение выбрано намеренно, потому что в текущем интегрированном MKS contract уже были:
  - gear ratio,
  - velocity/accel limits,
  - parameter tree import/export,
  но не было надёжно заведённого и используемого native min/max motor-angle feature path.
- Guard живёт в MKS adapter, то есть:
  - выше transport-independent ingress layer не поднимается,
  - ниже UI/HexaMotion остаётся единый minimal contract,
  - относительные команды сначала нормализуются, потом проверяются against limits.

#### Текущая policy

- Policy сейчас простая: **reject** out-of-range motion command.
- Clamp намеренно не включён, чтобы не скрывать операторскую/интеграционную ошибку и не создавать двусмысленного поведения.

## Update — 2026-04-26 19:17 Asia/Tbilisi

### MKS defaults, plotting deadband, and CAN-ID-based axis identity

По требованию босса внесены три прагматичных изменения только в MKS-path, без новых abstraction layers.

1. **Новые дефолты MKS**
   - Для MKS по умолчанию установлены:
     - `speed = 300 rpm`
     - `accel = 80 %`
   - Это синхронизировано на нескольких уровнях, чтобы UI и runtime не расходились:
     - `MksAxisWorkspace` UI defaults;
     - `MksAxisAdapter` fallback defaults;
     - `MksRuntimeConfig` axis defaults;
     - `MksAxisConfigurator` defaults/limits;
     - `MksAxisWorker` and `MksMotionBuildContext` fallback values.
   - Внутренний runtime accel byte теперь соответствует `80 %` (`204` в шкале `0..255`).

2. **Deadband для MKS графика позиции**
   - Чтобы интерфейс не перегружался микрофлуктуациями энкодера, добавлен UI-only deadband для MKS actual position trace:
     - `kScopePositionDeadbandDeg = 0.02`
   - Фильтрация применяется только на plotting path в `MksAxisWorkspace` при обработке batched `position_samples`.
   - Canonical telemetry snapshot и runtime logic не изменялись: фильтр не скрывает данные от motion/runtime/IPC, а только уменьшает шум на графике.
   - Состояние deadband сбрасывается при `disable`, `home`, `set zero`, а также при старте/остановке sine producer.

3. **MKS axis_id теперь соответствует CAN_ID**
   - Для MKS scan больше не используется generic auto-assignment глобального `axis_id`.
   - В `AxisManager::scanMotors()` введена отдельная политика:
     - `axis_id = transport_address = CAN_ID`
   - Добавлена явная валидация конфликтов:
     - CAN ID должен быть в диапазоне `[1..2047]`;
     - если такой `axis_id` уже занят в текущем runtime config, scan отклоняется с понятной ошибкой.
   - Для EtherCAT прежняя логика auto-assignment глобальных `axis_id` оставлена без изменений.

4. **Прокидка batched MKS position samples в UI**
   - `AxisManager::onFastTick()` снова собирает `position_samples` из `MksAxisAdapter::drain_position_samples(...)` и публикует их в telemetry map для workspace.
   - Это сохраняет high-resolution plotting path для MKS и даёт deadband-фильтрации работать на batch samples, а не на coarse single-point telemetry.

### Результат
- MKS operator defaults стали безопаснее и ближе к ожидаемым:
  - `300 rpm / 80 %`.
- Когда MKS мотор стоит, график позиции больше не flood'ится микродрожанием.
- Идентичность MKS оси стала очевидной для оператора:
  - `Axis ID == CAN ID`.


## Update — 2026-04-24 06:32 Asia/Tbilisi

### Restored motion/telemetry semantics after refactor regression

The runtime contract still stays minimal:

1. service queue
2. motion command queue
3. single canonical telemetry snapshot
4. estop
5. control owner switching

But the canonical snapshot must preserve **actual vs target** semantics.
The recent refactor collapsed them and caused three regressions:

1. MKS actual position was overwritten by requested target in CAN telemetry assembly.
2. MKS relative motion (`is_relative`) stopped being normalized to absolute target before driver execution.
3. UI target label/scope trace relied on corrupted telemetry and therefore target was missing or misleading.

### Current required invariants

#### TelemetrySnapshot

`TelemetrySnapshot` remains the only public telemetry object, but now it explicitly carries:

* `position` = actual position
* `velocity` = actual velocity
* `current` = actual current/torque proxy
* `target_position` = commanded/requested target
* `has_target_position` = whether target is explicitly available from transport/runtime

This keeps the architecture minimal while restoring semantic correctness.

#### MKS responsibilities

`MksCanBusManager` must publish:

* actual feedback into `TelemetrySnapshot.position`
* requested/commanded target into `TelemetrySnapshot.target_position`

`MksAxisAdapter` must normalize relative commands locally:

* if `MotionCommandPoint.is_relative == true`, convert it to an absolute target inside the MKS adapter
* use last requested target if present, otherwise latest actual telemetry position as the base

This logic is transport-specific and must not be moved into the common ingress layer, otherwise EtherCAT would risk double relative conversion.

#### EtherCAT responsibilities

`EthercatAxisAdapter` already owns correct relative-motion semantics in its adapter/RT path.
It only needs to expose commanded target explicitly in `TelemetrySnapshot.target_position` so UI and IPC consumers observe the same canonical target source.

#### UI responsibilities

`AxisManager::onFastTick()` must map:

* `actual_position_deg <- TelemetrySnapshot.position`
* `target_position_deg <- TelemetrySnapshot.target_position` when available

Transport workspaces that declare `transportProvidesTargetTrace() == true` must actually render the target trace from that target source.

### Practical consequence

The fix is intentionally narrow and pragmatic:

* no new abstraction layers
* no change to queue ingress ownership policy
* no change to minimal public orchestration contract
* only restoration of semantic correctness inside the existing architecture

По сути у босса сейчас очень жесткое требование:

кроме дерева параметров оставить только:

1. очередь сервисных команд,
2. очередь motion/command points,
3. получение телеметрии,
4. estop,
5. переключение источника управления HexaMotion.

Все остальное — удалить, даже если оно “удобное”.

Ниже целевая форма.

---

### Что должно остаться концептуально

Слои оставить такими:

1. `MainWindow / AxisWorkspace`
2. `AxisManager`
3. `RuntimeQueueIngress`
4. `HalRuntime`
5. `IAxis / IBusManager`
6. драйверы (`EthercatAxisAdapter`, `MksAxisAdapter/MksAxisWorker`)

Но API у этих слоев надо резко сузить.

---

## 1. Минимальный контракт системы

### Единственные разрешенные действия на ось

#### A. Положить сервисную команду в service queue

Все такие вещи, как:

* enable
* disable
* home
* clear errors
* set zero
* reset fault
* mode switch, если это сервисный акт

не должны больше быть отдельными публичными методами.

Они должны стать payload одного типа, например:

```cpp
enum class ServiceCommandType {
    Enable,
    Disable,
    Home,
    ClearErrors,
    SetZero,
    ResetDrive
};

struct ServiceCommandPoint {
    ServiceCommandType type;
    int axis_id;
    // optional payload if needed
};
```

И дальше везде один метод:

```cpp
bool enqueueServicePoint(const ServiceCommandPoint& p);
```

---

#### B. Положить motion point в command queue

Любые:

* move absolute
* move relative
* velocity jog
* stream point
* execute target
* setpoint push

не должны существовать как отдельные методы.

Все это должно быть выражено через единый тип command point:

```cpp
enum class MotionCommandType {
    Position,
    Velocity,
    Stream
};

struct MotionCommandPoint {
    MotionCommandType type;
    int axis_id;
    double value;
    double velocity;
    double acceleration;
    uint64_t timestamp_us;
};
```

И один метод:

```cpp
bool enqueueCommandPoint(const MotionCommandPoint& p);
```

---

#### C. Телеметрия

Один путь чтения:

```cpp
TelemetrySnapshot getTelemetry(int axis_id) const;
```

или

```cpp
const TelemetrySnapshot& telemetry(int axis_id) const;
```

Телеметрия берется из одного canonical snapshot. Не должно быть:

* `readPosition()`
* `readVelocity()`
* `readCurrent()`
* `readFaults()`
* `readStatusWord()`
* `readUiTelemetry()`
* `readHexaMotionTelemetry()`

Все это должно жить внутри одного `TelemetrySnapshot`.

HexaMotion и UI читают один и тот же snapshot.

---

#### D. E-Stop

Отдельный метод допустим как исключение:

```cpp
void estop();
```

или

```cpp
void estopAxis(int axis_id);
void estopAll();
void resetError(int axis_id);
```

Но без дополнительных alias’ов типа:

* `stopNow`
* `quickStop`
* `halt`
* `emergencyDisable`
* `panicStop`

Оставить только одно имя.

---

#### E. Переключение управления HexaMotion

Один явный переключатель ownership/source:

```cpp
void setHexaMotionControlEnabled(bool enabled);
```

или лучше явно:

```cpp
enum class ControlOwner {
    UI,
    HexaMotion
};

void setControlOwner(ControlOwner owner);
ControlOwner controlOwner() const;
```

Это лучше, чем bool, потому что потом не будет двусмысленности.

---

## 2. Что именно надо удалить

Надо удалить все методы, которые являются специализированными обертками над двумя очередями.

### Под нож идут такие категории

#### Из `AxisManager`

Удалить все методы вида:

* `enableAxis`
* `disableAxis`
* `homeAxis`
* `clearAxisErrors`
* `setAxisZero`
* `moveAbsolute`
* `moveRelative`
* `jogPositive`
* `jogNegative`
* `setVelocity`
* `streamPoint`
* `pushTrajectoryPoint`
* `stopMotion`
* `softStop`
* `resume`
* `hold`
* `release`
* любые `helper`/`convenience` методы

Оставить только:

* `enqueueServicePoint`
* `enqueueCommandPoint`
* `getTelemetry`
* `estop`
* `setControlOwner` / `setHexaMotionControlEnabled`
* дерево параметров как есть

---

#### Из `RuntimeQueueIngress`

Оставить только логику маршрутизации входящих команд в 2 очереди и арбитраж между UI и HexaMotion.

Удалить:

* все transport-aware helper’ы,
* все axis-specific sugar API,
* все “понятные UI методы”.

`RuntimeQueueIngress` должен быть тупым шлюзом.

---

#### Из `HalRuntime`

Оставить:

* регистрацию/доступ к осям,
* проброс минимального контракта,
* телеметрию,
* estop.

Удалить все “операционные” helper методы, которые дублируют логику `AxisManager` или драйвера.

---

#### Из `IAxis`

Интерфейс надо тоже сузить. Не должен быть “бог-объектом”.

Примерно так:

```cpp
class IAxis {
public:
    virtual ~IAxis() = default;

    virtual bool enqueueServicePoint(const ServiceCommandPoint& p) = 0;
    virtual bool enqueueCommandPoint(const MotionCommandPoint& p) = 0;

    virtual TelemetrySnapshot telemetry() const = 0;

    virtual void estop() = 0;

    virtual void setControlOwner(ControlOwner owner) = 0;
};
```

Если parameter tree сейчас торчит через `IAxis`, можно не трогать, раз босс сказал оставить его без изменений.

Удалить из `IAxis` все узкоспециализированные команды.

---

## 3. Где должна жить логика ownership UI / HexaMotion

Не в UI. Не в драйвере.

Правильное место — `AxisManager` + `RuntimeQueueIngress`.

Логика такая:

* `ControlOwner::UI`
  UI имеет право класть `MotionCommandPoint` в command queue.
  HexaMotion-команды игнорируются или отклоняются.

* `ControlOwner::HexaMotion`
  HexaMotion имеет право класть `MotionCommandPoint`.
  UI motion commands игнорируются или отклоняются.

* `ServiceCommandPoint` можно либо:

  1. разрешить всегда с обоих источников,
  2. либо также подчинить ownership.

Практически я бы сделал так:

* motion queue подчиняется ownership,
* service queue разрешена всегда,
* `estop` всегда вне ownership и всегда работает.

Это наиболее безопасная и инженерно адекватная модель.

---

## 4. Приоритет исполнения внутри драйвера

Внутри `MksAxisWorker` или любого другого axis worker должно быть очень просто:

1. если `estop` активен — немедленное аварийное действие;
2. если service queue не пуста — обработать service command;
3. иначе если command queue не пуста — обработать motion point;
4. обновить telemetry snapshot.

То есть сервис имеет приоритет над motion.

Примерно:

```cpp
void MksAxisWorker::tick() {
    if (estop_latched_) {
        executeEstop();
        refreshTelemetry();
        return;
    }

    ServiceCommandPoint svc;
    if (service_queue_.try_pop(svc)) {
        executeServiceCommand(svc);
        refreshTelemetry();
        return;
    }

    MotionCommandPoint cmd;
    if (command_queue_.try_pop(cmd)) {
        executeMotionCommand(cmd);
    }

    refreshTelemetry();
}
```

---

## 5. Телеметрия: один источник истины

Ты сам правильно написал: телеметрия уходит и в UI, и в HexaMotion.

Значит архитектурно это не “две телеметрии”, а один snapshot с двумя потребителями.

Нужно оставить один объект, например:

```cpp
struct TelemetrySnapshot {
    int axis_id;
    double position;
    double velocity;
    double current;
    bool enabled;
    bool fault;
    uint64_t timestamp_us;
};
```

Дальше:

* UI читает `TelemetrySnapshot`
* HexaMotion читает тот же `TelemetrySnapshot`

Никаких отдельных методов “телеметрия для UI” и “телеметрия для runtime”.

---

## 6. Что делать с parameter tree

Раз босс сказал не трогать — не трогать.

Это означает:

* не переименовывать,
* не переносить,
* не перепаковывать,
* не делать поверх него helper API,
* не тащить параметрические операции в новый упрощенный контракт.

То есть дерево параметров — это отдельная подсистема, которую не надо “улучшать”.

---

## 7. Как должен выглядеть `AxisManager` после зачистки

Вот практически целевая форма:

```cpp
class AxisManager {
public:
    bool enqueueServicePoint(const ServiceCommandPoint& p);
    bool enqueueCommandPoint(const MotionCommandPoint& p);

    TelemetrySnapshot getTelemetry(int axis_id) const;

    void estop();
    void estopAxis(int axis_id);

    void setControlOwner(ControlOwner owner);
    ControlOwner controlOwner() const;

    ParameterTree& parameterTree();
    const ParameterTree& parameterTree() const;
};
```

Если `parameterTree()` уже есть иначе — оставить как есть, без рефакторинга ради красоты.

Больше публичных методов быть не должно.

---

## 8. Самый важный инженерный момент: кто пишет в очереди

Чтобы не убить простоту и не ломать SPSC-подход, должен быть один canonical producer на ось.

То есть:

* UI не пишет прямо в драйвер,
* HexaMotion не пишет прямо в драйвер,
* оба идут через `RuntimeQueueIngress`,
* `RuntimeQueueIngress` уже один кладет в axis queue.

Иначе быстро появятся:

* гонки,
* странная ownership-логика,
* необходимость mutex,
* дублирование reject/filter logic.

---

## 9. Практический список на выпиливание для джуна

Ему надо дать не абстрактное “упрости”, а checklist.

### Оставить

* `enqueueServicePoint(...)`
* `enqueueCommandPoint(...)`
* `getTelemetry(...)`
* `estop(...)`
* `setControlOwner(...)` или `setHexaMotionControlEnabled(...)`
* parameter tree API без изменений

### Выпилить

* все специализированные axis action methods
* все convenience wrappers
* все driver-specific helper methods, торчащие наружу
* все отдельные telemetry getters по полям
* все UI-only shortcuts
* все HexaMotion-only shortcuts
* все дублирующие stop/halt/disable/pause методы, кроме одного estop
* все helper conversion methods, если они не private/internal necessity

---

## 10. Жесткая формулировка для босса / для задачи в трекер

Можно оформить так:

> Refactor target: collapse the public control surface to the minimum orchestration contract.
> Keep only:
>
> * service queue ingress,
> * motion command queue ingress,
> * telemetry retrieval,
> * estop,
> * HexaMotion control ownership switch,
> * existing parameter tree unchanged.
>
> Remove all specialized helper/convenience methods from `AxisManager`, `RuntimeQueueIngress`, `HalRuntime`, and public driver-facing interfaces.
> All service actions must be represented as `ServiceCommandPoint`.
> All motion actions must be represented as `MotionCommandPoint`.
> Telemetry must be exposed as a single canonical snapshot consumed by both UI and HexaMotion.

---

## 11. Мой вывод по сути

Да, направление босса логичное.

Он не просит “красивую архитектуру”. Он просит:

* убить распухший API,
* оставить только универсальные примитивы,
* запретить helper-методы как класс,
* сохранить параметрическое дерево,
* держать orchestration RuntimeQueueIngress,
* не тащить transport-специфику выше драйвера.

Если делать правильно, то вся система должна ощущаться так:

* есть **две очереди**,
* есть **одна телеметрия**,
* есть **один estop**,
* есть **один ownership switch**,
* все остальное — не API, а внутренние детали.
* дерево параметров и конфигурацию пока не трогаем

---

## Update 2026-04-08 03:14 (UTC+4) — Fix EtherCAT freeze on motion start (PP/CSP)

### Симптом
- По EtherCAT команды Enable/Disable работали.
- При старте движения в `Profile Position` или `CSP` UI выглядел как «зависший», потому что в UI не обновлялась статистика motion queue и не запускался refill-путь для CSP/sine.

### Корень проблемы
- В `AxisManager::onFastTick` для всех осей отдавался пустой `MotionQueueStats` (`size=0/capacity=0`), вместо чтения реальной очереди из адаптера.
- Для EtherCAT это ломало event-driven логику refill в `EthercatAxisWorkspace` (низкий watermark/префилл), поэтому движение не стартовало корректно и создавалось впечатление deadlock.
- Дополнительно в общем UI-path часть команд в CSP отправлялась с `kind=Position`, что не соответствовало ожиданиям EtherCAT runtime (для CSP нужен `Stream`).

### Что изменено
1. **Добавлен runtime-доступ к queue stats у адаптеров**:
   - `EthercatAxisAdapter::query_motion_queue_stats()` (возвращает size/capacity/pushed/dropped/underruns/short_starts).
   - `MksAxisAdapter::query_motion_queue_stats()` проксирует вызов в `MksAxisWorker`.
2. **`AxisManager::onFastTick` теперь публикует реальные stats**:
   - Для MKS — через `MksAxisAdapter`.
   - Для EtherCAT — через `EthercatAxisAdapter`.
3. **UI генерация motion-команд синхронизирована с CSP** в `AxisWorkspace`:
   - Для EtherCAT/CSP команды отправляются как `kind=0` (`Stream`) вместо `Position/Relative Position`.
   - Это применено для абсолютного move, jog, slider-движения и генератора траектории.

### Проверка
- Выполнена сборка:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-08 03:34 (UTC+4) — Fix second EtherCAT Move freeze (command eaten before drive ready)

### Симптом
- При нажатии `Move` на EtherCAT оси визуально происходило «зависание»: команда как будто не исполнялась.
- Это проявлялось как в сценариях вокруг CSP, так и при одиночных one-shot командах движения.

### Корень проблемы
- В `EthercatAxisAdapter::process_cycle()` команда из `motion_queue_` извлекалась (`try_pop`) **до** проверки готовности исполнения (`is_enabled`, `!fault`, `!mode_switch_pending`).
- Если ось в этом цикле была ещё не готова, команда фактически терялась (одноразовый `Move` уже вынут из очереди), что для оператора выглядело как freeze.
- Дополнительно в CSP/CSV ветке инициализация интерполятора на первом активном цикле затирала endpoint, из-за чего первый шаг мог обнуляться.

### Что изменено
1. **Защита от потери one-shot команды**:
   - `motion_queue_.try_pop(...)` теперь выполняется только когда `mode_ready_for_motion == true`.
   - Пока привод/режим не готовы, команда остаётся в очереди и будет обработана в следующем готовом цикле.
2. **Поправлена CSP/CSV интерполяция первого шага**:
   - Сброс endpoint в `pos_deg` выполняется только если привод реально не готов (`!enabled || fault`).
   - При первом готовом цикле интерполятор инициализируется от текущей позиции, но endpoint сохраняется из `cmd_.target_pos_deg`, без потери команды.

### Проверка
- Повторная сборка после правки:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-08 13:31 (UTC+4) — Fail-safe behavior for EtherCAT mode transitions (no UI/telemetry freeze)

### Цель
- Даже при несогласованности режима, reject движения или временной неготовности привода приложение не должно «виснуть».
- Телеметрия и UI должны продолжать работать, а поведение движения должно быть предсказуемым и логируемым.

### Что было причиной деградации
- Жёсткая проверка в `RuntimeQueueIngress::enqueueCommandPoint()` по `telemetry().mode` на этапе ingress.
- При асинхронном mode switch (особенно EtherCAT PP/CSP) это давало ранние reject'ы, и UI producer мог войти в цикл частых повторов.

### Что изменено
1. **Fail-safe ingress policy**
   - В `hal_host_service/runtime_queue_ingress.h` убрана блокирующая проверка совместимости `motion_type` с `telemetry mode` на ingress-уровне.
   - Ingress теперь авторизует источник и передаёт point в ось.
   - Строгое применение/гейтинг остаётся в адаптере оси (RT path), где известна реальная готовность привода.

2. **Анти-flood логирование enqueue reject в UI path**
   - В `ui/qt/mks/axis_manager/axis_manager_motion.cpp` добавлен throttling логов по оси (окно 500 мс) для `motion enqueue failed`.
   - Это защищает UI loop от log storm при повторяющихся reject состояниях.

3. **Явная трассировка mode switch запроса**
   - При `SetOperatingMode` через `enqueueServiceBatch` фиксируется pending-состояние в `AxisManager` и пишется лог `mode switch requested`.

4. **Fail-safe остановка EtherCAT sine producer при drop-росте**
   - В `EthercatAxisWorkspace` добавлен контроль baseline/роста `dropped` из motion queue stats.
   - При детекте новых drops в активном sine-режиме producer автоматически останавливается (через `disableSineUiState()`), пишется диагностический лог.
   - Это предотвращает бесконечный refill/enqueue цикл и псевдо-freeze UI.

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 02:39 (UTC+4) — Фикс применения скорости в EtherCAT manual homing (CSP-фаза)

### Проблема
- После рефакторинга ручного хоуминга желаемая скорость home-команды доходила до `EthercatAxisAdapter`,
  но в RT-цикле затиралась на этапе `target_position_override`.
- В результате в поисковой фазе manual homing (`force_csp_mode`) интерполяция шла не по заданной скорости,
  а по дефолту `kDefaultInterpolationVelocityDegPerSec = 30.0`.

### Что изменено
1. **Точечный фикс в `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`**
   - В блоке `manual_output.target_position_override` убран безусловный сброс:
     - `cmd_.has_target_vel = false`
     - `cmd_.target_vel_deg_s = 0.0`
   - Теперь сброс выполняется **только если не активен** `manual_output.force_csp_mode`.

### Результат
- В CSP-поиске ручного хоуминга скорость из команды (`homing_speed_deg_per_sec`) больше не затирается.
- Желаемая скорость корректно влияет на интерполяцию в manual homing search phase.
- Поведение PP-фазы (`force_pp_mode`) не менялось.

### Проверка
- Выполнена сборка:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 02:27:59 (UTC+4) — Manual Homing: скорость из UI + фиксация финального режима

### Цель
- Убрать hardcoded скорость в EtherCAT manual homing и брать её из UI.
- После завершения homing оставлять привод в том режиме, на котором закончилась процедура (PP), без отката в CSP.

### Что изменено
1. **Скорость homing протянута по всей цепочке UI → runtime → adapter → state machine**
   - `motion_core/include/motion_core/axis_data.h`
     - в `ServiceCommandPoint` добавлено поле `homing_speed_deg_per_sec`.
   - `ui/qt/mks/axis_workspace/axis_workspace.cpp`
     - при `GO HOME` в service-команду добавляется текущая UI-скорость (`profile_speed_rpm` из `speed_spin`).
   - `ui/qt/mks/axis_manager/axis_manager_motion.cpp`
     - для `ServiceCommandType::Home` вычисляется `command.homing_speed_deg_per_sec`:
       - при наличии `target_velocity_deg_per_sec` берётся напрямую,
       - иначе конвертируется из `profile_speed_rpm * 6.0`.
   - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.h`
     - добавлено `cmd_.manual_homing_speed_deg_s` (atomic runtime state).
   - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
     - в обработке `Home` записывается скорость из service-команды в `manual_homing_speed_deg_s`.
     - в `manual_inputs` в state machine передаётся `search_velocity_deg_per_sec`.

2. **State machine перестала использовать жёсткую константу скорости**
   - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.h/.cpp`
     - вход `EthercatManualHomingInputs` расширен полем `search_velocity_deg_per_sec`.
     - `kSearchVelocityDegPerSec` заменён на `kDefaultSearchVelocityDegPerSec` (fallback).
     - в фазе `SearchForSwitch` шаг цели считается из входной скорости, а не из hardcoded значения.

3. **Убран откат в CSP после завершения PP-фазы**
   - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
     - при `manual_output.force_pp_mode` теперь дополнительно фиксируется
       `cmd_.mode_req = ProfilePosition`.
     - это сохраняет PP как базовый requested mode после завершения homing и предотвращает возврат к ранее записанному CSP.

4. **PP return-скорость теперь соответствует homing-скорости из UI**
   - в adapter расчёт `profile_vel_rpm` для PP return выполняется из `manual_inputs.search_velocity_deg_per_sec` (а не из константы).

### Результат
- Search/return скорость manual homing задаётся из UI speed control.
- После завершения homing ось остаётся в `ProfilePosition`, без автоматического возврата в CSP.

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 01:41:43 (UTC+04:00) — EtherCAT Manual Homing: CSP search + move-to-offset + set-zero

### Цель
- Убрать неоднозначное поведение manual homing (поиск в PV + паразитный post-home creep).
- Перевести search phase на CSP.
- Привести семантику к требованию: **после DI3 сначала физически доехать до offset-позиции, затем выполнить set-zero**.

### Что изменено
1. **Переделана state machine manual homing (EtherCAT)**
   - Файлы:
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.h`
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.cpp`
   - Убрана velocity-based семантика (`velocity_override`, `hold_position`, `apply_zero_with_home_shift`).
   - Добавлены входы для процедурной позиции:
     - `cycle_dt_sec`
     - `home_offset_deg`
   - Новые выходы:
     - `force_csp_mode`
     - `target_position_override`
     - `target_position_deg`
     - `apply_set_zero`
   - Новые фазы:
     - `Idle`
     - `SearchForSwitch`
     - `MoveToOffset`
   - Логика:
     - search идет как CSP target ramp: `target += search_speed * dt`;
     - при DI3 фиксируется switch-позиция и вычисляется offset target;
     - после достижения offset (с допуском) выдается `apply_set_zero`.

2. **Интеграция новой CSP homing логики в EthercatAxisAdapter**
   - Файл:
     - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
   - `ServiceCommandType::Home` теперь принудительно запрашивает `CyclicSyncPosition` work mode на start homing.
   - В `process_cycle()`:
     - передаются новые inputs в state machine (`dt`, `home_offset_deg`);
     - при `force_csp_mode` удерживается CSP режим;
     - при `target_position_override` target берется из homing state machine;
     - при `apply_set_zero` инициируется стандартный `set_zero_req`.
   - Убран старый immediate-shift path (`apply_homing_zero_from_current_position`) из homing-процедуры.
   - Motion queue во время homing блокируется через `manual_homing_busy`.

3. **Явная фиксация ManualHoming work mode**
   - `mode_to_work_mode(AxisMode::ManualHoming)` изменен с PV (`3`) на CSP (`8`).

### Результат
- Search phase manual homing теперь позиционная (CSP), а не velocity-based.
- Процедура соответствует целевой последовательности:
  - search до DI3,
  - move to offset,
  - set-zero в offset-точке.
- Убрана прежняя причина неоднозначного пост-хоумингового поведения, связанная с legacy PV-path.

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 02:06:08 (UTC+04:00) — Hybrid Manual Homing: CSP search + PP return-to-offset

### Цель
- По запросу босса перевести **фазу возврата к offset** в `Profile Position`, оставив поисковую фазу по DI3 в CSP.
- Сохранить простую и предсказуемую семантику процедуры: `search -> pause -> return -> set-zero`.

### Что изменено
1. **State machine manual homing расширена для гибридного режима**
   - Файлы:
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.h`
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.cpp`
   - В `EthercatManualHomingOutputs` добавлены поля:
     - `force_pp_mode`
     - `request_pp_setpoint`
   - Добавлен внутренний флаг one-shot PP setpoint:
     - `offset_pp_setpoint_issued_`
   - Поведение фаз:
     - `SearchForSwitch`: как и раньше CSP target ramp;
     - `SwitchDetectedPause`: удержание позиции switch точки 1с в CSP;
     - `MoveToOffset`: теперь выдает `force_pp_mode=true`, target=offset и одноразовый `request_pp_setpoint=true`.

2. **Интеграция PP return-фазы в EtherCAT adapter**
   - Файл:
     - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
   - Добавлена обработка `manual_output.force_pp_mode`:
     - принудительный запрос `ProfilePosition` work mode;
     - отключение `target_vel` пути;
     - установка `profile_vel_rpm` на основе текущей homing скорости (`kSearchVelocityDegPerSec`) с учетом gear ratio.
   - Добавлена обработка `manual_output.request_pp_setpoint`:
     - устанавливается `cmd_.pp_new_setpoint_req = true` (one-shot старт PP перемещения).
   - Для `manual_output.target_position_override` отключение `profile_vel` теперь не выполняется в PP return-фазе.
   - `manual_homing_busy` расширен:
     - учитывает `force_pp_mode` наряду с `active/force_csp_mode`.

### Итоговая семантика manual homing
- `SearchForSwitch` — CSP
- `SwitchDetectedPause` — CSP hold (1s, неблокирующе)
- `MoveToOffset` — PP one-shot move
- `ApplyZero` — set-zero после достижения offset (по текущему допуску)

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 01:52:11 (UTC+04:00) — Неблокирующая пауза 1с после DI3 в EtherCAT Manual Homing

### Цель
- Сделать поведение после срабатывания DI3 однозначным: короткая остановка перед переходом в `move-to-offset`.
- Реализовать ожидание без блокировки RT потока (без `sleep_for` внутри цикла).

### Что изменено
1. **Добавлена отдельная фаза паузы в manual homing state machine**
   - Файлы:
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.h`
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.cpp`
   - Новая фаза:
     - `SwitchDetectedPause`
   - Новое внутреннее состояние:
     - `pause_remaining_sec_`
   - Константа паузы:
     - `kSwitchPauseSec = 1.0`

2. **Логика переходов обновлена**
   - При DI3 в `SearchForSwitch` state machine больше не переходит сразу в `MoveToOffset`.
   - Теперь переход:
     - `SearchForSwitch` -> `SwitchDetectedPause` (1 сек)
     - затем `MoveToOffset`.
   - Во время паузы state machine продолжает выдавать:
     - `force_csp_mode = true`
     - `target_position_override = true`
     - `target_position_deg = switch_position_deg_`
   - Таймер паузы уменьшается на каждом шаге через `dt` (`cycle_dt_sec`), без блокировок.

3. **Сброс состояния дополнен**
   - В `reset()` теперь также сбрасывается `pause_remaining_sec_`.

### Результат
- После DI3 ось удерживает позицию switch-точки 1 секунду в CSP.
- Только после этого стартует фаза доезда до offset.
- RT-цикл остаётся неблокирующим и детерминированным.

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-09 00:51:48 (UTC+4) — EtherCAT: временно заблокирован обычный homing, оставлен только manual DI3

### Цель
- Выполнить требование босса: убрать архитектурные хвосты после рефакторинга и временно отключить стандартный EtherCAT Homing path.
- Оставить единый, простой и предсказуемый путь хоуминга для EtherCAT: **Manual Homing (DI3)**.

### Что изменено
1. **Manual DI3 state machine упрощена до level-based семантики**
   - Файлы:
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.h`
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.cpp`
   - Удалён edge-tracking (`di3_prev_`) и логика по rising-edge.
   - Поведение теперь простое и детерминированное:
     - при `request_start()` и уже активном DI3 — немедленно формируется `hold_position + apply_zero_with_home_shift`;
     - в активном поиске завершение срабатывает по факту `DI3 == 1` (уровень), без зависимости от фронта.

2. **Из EtherCAT adapter удалён стандартный homing path (HM mode 6)**
   - Файлы:
     - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.h`
     - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
   - Удалены поля/ветки, связанные с обычным homing (`homing_req`, `homing_active`, проверки `MaskHomingAttained/MaskHomingError`).
   - `ServiceCommandType::Home` теперь всегда ведёт в manual DI3 path:
     - принудительный `ManualHoming` work-mode,
     - синхронизация режима,
     - clear motion queue,
     - `manual_homing_start_req = true`.
   - `AxisMode::Homing` в `mode_to_work_mode()` теперь заблокирован (`-1`),
     а в `work_mode_to_mode()` убран обратный маппинг `6 -> Homing`.

3. **AxisManager/IPC/UI: запрет обычного EtherCAT Homing и явный manual path**
   - Файлы:
     - `ui/qt/mks/axis_manager/axis_manager_hal_service.cpp`
     - `ui/qt/mks/axis_manager/axis_manager_motion.cpp`
     - `ui/qt/mks/axis_workspace/ethercat_axis_workspace.cpp`
   - `StartManualHoming` больше не отрезан полностью:
     - для EtherCAT маршрутизируется в `ServiceCommandType::Home` (manual DI3 path);
     - для не-EtherCAT возвращает `Unsupported`.
   - `SetAxisMode` для EtherCAT + `AxisMode::Homing` теперь отклоняется с понятной ошибкой.
   - Batch-service путь (`enqueueServiceBatch`) также блокирует попытку установить `Homing` на EtherCAT.
   - В EtherCAT workspace убран пункт режима `"Homing"` из UI-комбобокса, оставлен `"Manual Homing (DI3)"`.

### Результат
- Для EtherCAT остался один путь хоуминга: **Manual DI3**.
- Удалены параллельные/конфликтующие ветки стандартного Homing, которые создавали неоднозначность и регрессы после рефакторинга.
- Контракт стал проще: без fallback’ов и без скрытой зависимости от порядка команд для запуска хоуминга.

---

## Update 2026-04-09 00:22 (UTC+4) — Аудит и зачистка manual homing DI3 после неудачного рефакторинга

### Проблема
- После рефакторинга в системе одновременно существовали две реализации manual homing:
  1. рабочая и актуальная в `EthercatAxisAdapter` + `ethercat_manual_homing_state_machine`;
  2. legacy-ветка `ManualHomingOrchestrator` в `motion_core`, которая больше не входила в минимальный публичный контракт.
- Это создавало split-brain архитектуру: непонятно, какой путь является каноническим.

### Что изменено
1. **Единый канонический path manual homing оставлен в EtherCAT драйвере**
   - Сохранён только путь через сервисные команды:
     - `SetOperatingMode(ManualHoming)`
     - затем `Home`
   - Фактическое выполнение DI3-homing остаётся в:
     - `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp`
     - `drivers/interfaces/ethercat/manual_homing/ethercat_manual_homing_state_machine.*`

2. **Удалён orphaned orchestrator путь из motion_core**
   - Из `motion_core/src/hal_runtime.cpp` удалены:
     - include `manual_homing_orchestrator.h`
     - запуск/останов orchestrator в `start()/stop()`
     - методы `start_manual_homing()/stop_manual_homing()`
   - `HalRuntime` приведён к консистентному minimal contract.

3. **Удалены неиспользуемые файлы и сборочные ссылки**
   - Удалены файлы:
     - `motion_core/include/motion_core/manual_homing_orchestrator.h`
     - `motion_core/src/manual_homing_orchestrator.cpp`
   - В `motion_core/CMakeLists.txt` удалён `src/manual_homing_orchestrator.cpp` из:
     - `motion_core`
     - `motion_core_no_hw`

### Результат
- В проекте остался один понятный и поддерживаемый DI3 manual homing pipeline без дублирования логики.
- Контракт `HalRuntime` и фактическая реализация снова согласованы.


---

## Update 2026-04-08 18:44 (UTC+4) — Fix hard telemetry freeze on any motion/enable action

### Симптом (по полевому фидбеку)
- Любое действие `jog/move`, а также `enable/disable` приводило к остановке обновления телеметрии (визуально «зависал» UI).

### Найденные проблемы
1. **RT-thread log flood в EtherCAT adapter**
   - В `EthercatAxisAdapter::process_cycle()` mode-switch progress печатался через `std::fprintf(stderr, ...)`.
   - Этот путь выполняется в RT-цикле и при частых mode transitions создавал блокирующий I/O pressure.

2. **Лишний gating по `mode_switch_pending_` в motion dispatch**
   - Обработка motion queue блокировалась условием `!mode_switch_pending_`.
   - В реальном асинхронном переключении это могло удерживать producer/consumer в конфликтном состоянии и провоцировать drop/freeze-паттерн.

3. **Sine guard не переармировался после смены режима**
   - После drop-detection guard останавливал sine producer корректно, но не имел явного auto re-arm при последующем реальном изменении режима.

### Что изменено
1. **Убран RT stderr logging для mode-switch trace**
   - `log_mode_switch_event(...)` в `drivers/interfaces/ethercat/adapter/ethercat_axis_adapter.cpp` переведён в no-op.
   - Это исключает потенциально блокирующий stdout/stderr I/O из RT цикла.

2. **Смягчён mode-ready критерий в RT motion dispatch**
   - В `EthercatAxisAdapter::process_cycle()`:
     - было: `is_enabled && !is_faulted && !mode_switch_pending_`
     - стало: `is_enabled && !is_faulted`
   - Команды теперь не подвисают из-за внутреннего pending-флага, при этом валидация по активному mode (`command_matches_mode`) остаётся.

3. **Добавлен auto re-arm для EtherCAT sine drop-guard**
   - В `EthercatAxisWorkspace` добавлено поле `last_driver_mode_`.
   - При изменении telemetry-mode guard сбрасывается (`sine_stopped_due_to_drops_ = false`, baseline reset), логируется `sine producer guard re-armed`.

### Проверка
- Сборка после фиксов:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-08 20:53 (UTC+4) — Кондовое упрощение AxisManager и устранение re-entrant deadlock

### Проблема
- В `AxisManager` был re-entrant deadlock в цепочке motion authorize:
  - `RuntimeQueueIngress state_provider` брал `control_state_mutex_`;
  - под этим lock вызывался `host_service_->state_snapshot()`;
  - `state_snapshot()` вызывал provider из `AxisManager`, который снова пытался взять `control_state_mutex_`.

### Что изменено
1. **Устранён deadlock-путь (без внешних вызовов под lock)**
   - В `ui/qt/mks/axis_manager/axis_manager_core.cpp` в provider для `RuntimeQueueIngress`:
     - чтение `control_source_`/`estop_active_` остаётся под `control_state_mutex_`;
     - вызов `host_service_->state_snapshot()` перенесён строго **после unlock**.

2. **Удалены лишние passthrough-wrapper методы AxisManager**
   - Удалены private методы:
     - `findAxis(std::uint16_t)`
     - `listAxes()`
   - Во всех местах использованы прямые вызовы:
     - `unified_runtime_.find_axis(...)`
     - `unified_runtime_.list_axes()`
   - Затронуты файлы:
     - `ui/qt/mks/axis_manager/axis_manager.h`
     - `ui/qt/mks/axis_manager/axis_manager_core.cpp`
     - `ui/qt/mks/axis_manager/axis_manager_params.cpp`

### Результат
- Убран общий cross-transport фриз на `Move/Jog`, вызванный re-entrant lock path.
- Код AxisManager стал проще: меньше прокладок и меньше неявных уровней вызовов.

### Проверка
- Выполнена сборка:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target mks_can_cli`
  - `Built target motor_tester_gui`

---

## Update 2026-04-08 23:25 (UTC+4) — Простая глобальная нумерация осей для mixed MKS/EtherCAT

### Цель
- Убрать неоднозначность `axis_id` без диапазонов вида `10000+` и без дополнительных слоёв.
- Обеспечить одновременную работу mixed runtime (MKS + EtherCAT) за счёт явного разделения:
  - `axis_id` = глобальный ID приложения,
  - `transport_address` = локальный адрес на конкретной шине.

### Что изменено
1. **Назначение глобальных `axis_id` после scan в AxisManager**
   - В `ui/qt/mks/axis_manager/axis_manager_core.cpp` добавлены простые helper-функции:
     - поиск следующего свободного `axis_id`;
     - переназначение `axis_id` для только что обнаруженных осей.
   - В `scanMotors()` и `scanEthercatMotors()` теперь найденные оси получают новые глобальные ID последовательно (1..N с пропусками по занятым), а `transport_address` остаётся неизменным.

2. **Явная ранняя проверка дубликатов `axis_id` при загрузке конфига**
   - В `ui/qt/mks/axis_manager/axis_manager_config.cpp` добавлена валидация `has_duplicate_axis_ids(...)`.
   - При дублировании загрузка прерывается с понятным сообщением:
     - `HAL config load failed: duplicate global axis_id=<id>`.

### Результат
- Устранена главная причина смешанного конфликта идентичности осей между transport'ами.
- Решение «кандовое»: без новых абстракций, без изменения публичного API управления осями.
- По запросу на упрощение лишние публичные методы дополнительно не вводились и не требовались к удалению в рамках этого фикса (удаление методов: `0`).

### Проверка
- Сборка после изменений:
  - `cmake --build /home/rdt/Desktop/Ethercat/MotorTester_v1/MotorTester_app/build -j6`
  - `Built target motor_tester_gui`

