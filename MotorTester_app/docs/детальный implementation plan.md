Ниже — **детальный implementation plan**, исходя из твоего решения: **не плодить новый wrapper, а сделать `AxisControlService` канонической headless API-точкой** для:
- вашего Qt GUI,
- CLI,
- и второй команды.

---

# 0. История и цель рефакторинга

## Что есть сейчас
Сейчас проект уже имеет хороший базовый каркас:
- `IAxis` как общий контракт оси,
- `AxisControlService` как общий сервис управления осью,
- MKS/EtherCAT adapters,
- runtime factories,
- Qt GUI как средство конфигурирования и теста.

Но логика пока разделена неидеально:
- часть orchestration живёт в `AxisControlService`,
- часть lifecycle/config/runtime flow — в `AxisManager`,
- часть parameter UX и export/import — в GUI,
- transport-specific policy местами протекает вверх.

## Что хочет босс / вторая команда
Нужен сценарий, где:
1. GUI остаётся **основным инструментом настройки и отладки**;
2. через GUI можно:
   - поднять mixed system (`CAN + EtherCAT`),
   - сконфигурировать оси,
   - сохранить полные конфиги,
   - оттестировать цикл управления;
3. вторая команда потом берёт **тот же код** и использует только базовые headless функции:
   - загрузить HAL config,
   - загрузить осевые конфиги,
   - применить,
   - стартовать runtime,
   - слать target position,
   - читать feedback/status/errors.

## Главная архитектурная цель
Сделать так, чтобы:
- **GUI был клиентом того же production-path**, что и вторая команда;
- **`AxisControlService` стал каноническим API** для runtime/config/control/feedback;
- `AxisManager` остался тонким Qt bridge;
- конфиги были простыми, полными и переносимыми.

---

# 1. Целевая архитектура

## 1.1. Верхнеуровневая схема

```text
Qt GUI
  -> AxisManager (QObject bridge only)
    -> AxisControlService
      -> Runtime/Bus orchestration
      -> IAxis registry
      -> Config apply/export
      -> Control commands
      -> Feedback readout
        -> MksAxisAdapter / EthercatAxisAdapter
          -> MksCanBusManager / EthercatBusManager
```

И параллельно:

```text
Second team HAL code
  -> AxisControlService
      -> same runtime/config/control/feedback path
```

## 1.2. Жёсткое правило новой архитектуры
`AxisControlService` становится **единственной канонической headless API-точкой**.

### В нём должно жить:
- runtime lifecycle,
- axis registry,
- config apply/export,
- axis control,
- unified feedback/status.

### В нём не должно жить:
- Qt,
- `QObject`,
- `QVariant`,
- `QTreeWidget`,
- UI HTML descriptions,
- графики, таймеры UI,
- GUI-specific polling/presentation.

## 1.3. Роль `AxisManager` после рефакторинга
`AxisManager` остаётся, но становится тонким:
- bridge between Qt and `AxisControlService`;
- signals/slots;
- `QVariant` conversion;
- UI-friendly async proxy;
- возможно UI telemetry polling timer.

Он **не должен больше быть владельцем business logic**.

---

# 2. Целевая история использования для двух команд

## 2.1. Ваш GUI workflow
1. Открыть HAL runtime из master-config.
2. Просканировать / подцепить оси.
3. Для каждой оси:
   - считать полное дерево параметров;
   - изменить нужные значения;
   - применить;
   - сохранить полный осевой config.
4. Сохранить master HAL config.
5. Через тот же runtime path прогнать тесты движения, feedback, errors.
6. Зафиксировать рабочие конфиги в git.

## 2.2. Workflow второй команды
1. Подтянуть из git актуальную библиотеку.
2. Прочитать master HAL config.
3. Поднять runtime.
4. Загрузить осевые конфиги.
5. Применить конфиг каждой оси.
6. Стартовать runtime.
7. В цикле:
   - set target position,
   - read actual position,
   - read state/status/faults.

Важно: **никакой второй архитектуры**, только другой клиент к тому же API.

---

# 3. Модель конфигов

Нужно 2 уровня конфигов.

---

## 3.1. Осевой конфиг: `AxisConfig`
Это **полный снимок конфигурации оси**, не просто случайный patch.

### Назначение
- reproducible setup;
- передача между командами;
- отсутствие «скрытых» ручных шагов.

### Структура (логически)
```text
AxisConfig
  version
  axis_id
  axis_name
  transport_kind
  hardware_profile / model
  startup_policy
  persistable_parameters[]
  extra_runtime_metadata
```

### Что должно входить
- все **persistable writable parameters**;
- transport-specific параметры;
- common hardware metadata:
  - gear ratio,
  - encoder resolution,
  - units scaling;
- возможно startup mode / default enable policy / homing policy.

### Что не должно входить
- runtime-only telemetry;
- transient status;
- read-only live diagnostics;
- volatile errors;
- значения, которые отражают только текущее состояние шины, а не конфиг.

### Важный принцип
Нужна **явная фильтрация persistable-параметров**.
Иначе “полный snapshot” будет грязным и опасным.

---

## 3.2. Master HAL config: `HalRuntimeConfig`
Это “царь-конфиг”, который описывает весь runtime.

### Назначение
- описать topology;
- описать buses;
- описать cycle times;
- связать axes с их axis-config files.

### Структура (логически)
```text
HalRuntimeConfig
  version
  runtime
    dispatch_period_ms
    telemetry_period_ms
    startup_policy
  mks_buses[]
  ethercat_buses[]
  axes[]
    axis_id
    transport_kind
    bus_ref
    config_file
    startup_mode
    enable_on_start
```

### Что в нём должно быть
- времена циклов;
- CAN bus settings;
- EtherCAT interface/master settings;
- список всех осей;
- тип оси;
- привязка оси к bus/master;
- путь к axis config file.

### Сценарий mixed runtime
Да, этот формат должен поддерживать, например:
- 3 CAN оси,
- 3 EtherCAT оси,
- и один общий runtime lifecycle.

---

# 4. Что нужно изменить в коде концептуально

## 4.1. Расширить `AxisControlService`
Он уже умеет:
- registry,
- axis commands,
- telemetry,
- parameter read/write,
- motion queue.

Теперь ему нужно добавить **system/runtime/config orchestration**.

### Новые логические обязанности
1. runtime open/close/start/stop;
2. build runtime from structured config;
3. attach adapters/bus managers;
4. apply full axis config;
5. export full axis config;
6. unified feedback read.

---

## 4.2. Не мешать JSON parsing с core orchestration
Рекомендация:
- **JSON loading/saving держать отдельно**;
- `AxisControlService` должен работать со структурами `AxisConfig` / `HalRuntimeConfig`.

То есть:
- `load_axis_config_from_file(path) -> AxisConfig`
- `save_axis_config_to_file(path, AxisConfig)`
- `load_hal_runtime_config_from_file(path) -> HalRuntimeConfig`
- `save_hal_runtime_config_to_file(path, HalRuntimeConfig)`

А уже потом:
- `AxisControlService::apply_axis_config(axis_id, cfg)`
- `AxisControlService::open_runtime(cfg)`

Это лучший баланс простоты и чистоты.

---

# 5. Конкретная целевая API-модель

## 5.1. Новые DTO / модели
Нужно ввести:
- `AxisPersistedParameter`
- `AxisConfig`
- `HalBusConfigMks`
- `HalBusConfigEthercat`
- `HalAxisRuntimeEntry`
- `HalRuntimeConfig`
- `AxisFeedback`
- `AxisStatusSnapshot`

---

## 5.2. Что добавить в `AxisControlService`
Примерно такой набор:

### Runtime lifecycle
- `Result<void> open_runtime(const HalRuntimeConfig&)`
- `Result<void> close_runtime()`
- `Result<void> start_runtime()`
- `Result<void> stop_runtime()`
- `Result<std::vector<AxisInfo>> list_axes() const`

### Axis config lifecycle
- `Result<AxisConfig> export_axis_config(AxisId axis_id) const`
- `Result<void> apply_axis_config(AxisId axis_id, const AxisConfig& cfg)`
- `Result<void> apply_all_axis_configs(const HalRuntimeConfig&, const AxisConfigRepository&)`

### Feedback lifecycle
- `Result<AxisFeedback> read_axis_feedback(AxisId axis_id) const`
- `Result<AxisStatusSnapshot> read_axis_status(AxisId axis_id) const`

### Optional convenience
- `Result<void> configure_and_start(const HalRuntimeConfig&, const AxisConfigRepository&)`

---

## 5.3. Что оставить как есть
Оставить существующие axis-level calls:
- `enable_axis`
- `set_axis_mode`
- `move_absolute`
- `move_relative`
- `submit_command`
- `read_telemetry`
- `list_parameters`
- `read_parameters`
- `apply_parameter_patch`

То есть новый API должен быть **additive**, не ломающим старый.

---

# 6. Изменения по слоям

## 6.1. `motion_core`
### Добавить
- config DTO headers/cpp;
- feedback/status DTO;
- расширенный `AxisControlService`;
- transport-agnostic runtime orchestration logic.

### Оставить Qt-free
Критично: никакого Qt.

---

## 6.2. `drivers/interfaces/mks`
### Нужно будет поддержать
- экспорт persistable parameters в `AxisConfig`;
- применение полного `AxisConfig`;
- возможно capability/query для persistable tree.

### Вероятная доработка
Разделить параметры на:
- persistable config,
- runtime-only,
- telemetry/read-only.

---

## 6.3. `drivers/interfaces/ethercat`
То же самое, но с EtherCAT-спецификой.

Особенно важно:
- формализовать, какие параметры persistable;
- убрать ad-hoc startup special cases из `AxisManager` и перевести их в config/runtime policy.

---

## 6.4. `ui/qt/mks/AxisManager`
После рефакторинга он должен:
- вызывать новый API `AxisControlService`;
- не знать, как строится runtime;
- не владеть transport-specific startup orchestration;
- не быть местом применения policy.

### Он должен остаться для GUI
Потому что нужен Qt bridge, но не больше.

---

## 6.5. `AxisWorkspace`
Остаётся UI-инструментом:
- показать parameter tree;
- позволить редактировать и применять;
- export/import axis config;
- запускать тесты движения;
- отображать feedback.

Но конечный путь должен идти через канонический API.

---

# 7. Пошаговый план рефакторинга

Ниже — безопасный порядок, чтобы **не сломать всё**.

---

## Phase 1 — Зафиксировать целевую архитектуру и модели
### Цель
Сначала описать архитектуру и DTO, ничего не ломая в runtime.

### Шаги
1. Обновить `ARCHITECTURE.md` новой целевой схемой.
2. Ввести DTO:
   - `AxisConfig`
   - `HalRuntimeConfig`
   - `AxisFeedback`
3. Ввести schema versioning.
4. Формально описать:
   - persistable parameters,
   - runtime-only parameters.

### Результат
Есть контракт, которым смогут пользоваться и GUI, и вторая команда.

### Риск
Низкий.

---

## Phase 2 — Добавить config models и JSON support
### Цель
Сделать переносимые конфиги.

### Шаги
1. Добавить shared config module, например:
   - `motion_core/include/motion_core/config/axis_config.h`
   - `motion_core/include/motion_core/config/hal_runtime_config.h`
   - `motion_core/src/config/...`
2. Реализовать load/save JSON.
3. Сделать sample configs.
4. Сделать unit tests на parse/save/roundtrip.

### Результат
Конфиги существуют как стабильный API/format.

### Риск
Низкий.

---

## Phase 3 — Научить adapters формировать persistable configs
### Цель
Сделать осевой config воспроизводимым.

### Шаги
1. Для MKS определить persistable subset параметров.
2. Для EtherCAT определить persistable subset параметров.
3. Добавить методы экспорта/применения полного осевого конфига.
4. Добавить verification after apply.

### Результат
Ось можно:
- считать в полный config,
- сохранить,
- заново поднять и применить.

### Риск
Средний, потому что легко перепутать runtime-only и persistable параметры.

---

## Phase 4 — Расширить `AxisControlService`
### Цель
Сделать его канонической headless API-точкой.

### Шаги
1. Добавить методы runtime lifecycle.
2. Добавить методы apply/export axis config.
3. Добавить unified feedback/status API.
4. Подвязать runtime factories внутрь orchestration flow.
5. Сохранить старый API совместимым.

### Результат
Вторая команда уже может работать через `AxisControlService` напрямую.

### Риск
Средний.

### Как снизить риск
- не удалять старые methods;
- добавлять новый путь рядом;
- покрыть integration tests.

---

## Phase 5 — Перевести `AxisManager` на новый канонический path
### Цель
GUI должен использовать тот же код, что и вторая команда.

### Шаги
1. Внутри `AxisManager` заменить ручной runtime flow на вызовы `AxisControlService`.
2. Перенести special-case startup/config logic из `AxisManager` в сервис.
3. Оставить в `AxisManager` только:
   - Qt bridge,
   - QVariant conversion,
   - telemetry signal emission.

### Результат
GUI реально тестирует production headless path.

### Риск
Средний.

---

## Phase 6 — GUI export/import full configs
### Цель
Сделать GUI не только редактором patch, а редактором полноценных конфигов.

### Шаги
1. Добавить export full axis config.
2. Добавить import/apply axis config.
3. Добавить HAL master-config editor/import/export.
4. Добавить кнопки:
   - Export Axis Config
   - Import Axis Config
   - Export HAL Config
   - Load HAL Config

### Результат
GUI становится полноценным инструментом подготовки handoff-артефактов.

### Риск
Низко-средний.

---

## Phase 7 — Mixed runtime scenario validation
### Цель
Подтвердить сценарий `3 CAN + 3 EtherCAT`.

### Шаги
1. Подготовить sample HAL config для mixed runtime.
2. Подготовить 6 axis configs.
3. Проверить в GUI:
   - open runtime,
   - apply configs,
   - start,
   - move axes,
   - read feedback/errors.
4. Проверить headless sample для второй команды.

### Результат
Реальный proof, что reuse работает.

### Риск
Средний, потому что mixed startup/policy — самый чувствительный сценарий.

---

## Phase 8 — Handoff package for second team
### Цель
Сделать встраивание максимально простым.

### Шаги
1. Подготовить README для integration.
2. Дать sample `headless_main.cpp`.
3. Дать sample configs.
4. Описать release process и совместимость версий config schema.

### Результат
Вторая команда реально может взять из git и интегрировать.

### Риск
Низкий.

---

# 8. Что именно переносить из `AxisManager`

## Переносить в `AxisControlService`
- runtime startup/shutdown orchestration;
- config apply sequence;
- verification after patch/config;
- transport-neutral feedback read;
- startup policy для runtime;
- attach/register axis runtime path.

## Оставлять в `AxisManager`
- `QObject`;
- signals/slots;
- `QVariantList`/`QVariantMap` conversion;
- UI timers for update emission;
- вызовы из GUI-кнопок.

---

# 9. Что делать с parameter tree

## Сейчас
Сейчас это плоский `QTreeWidget`, не настоящее дерево.

## В новом плане
Это нормально оставить на первом этапе.

### Почему
Потому что главная цель сейчас:
- reproducible configs,
- common backend path,
- handoff второй команде.

А не идеальная GUI-модель.

## Что нужно добавить минимально
- метку `persistable` для parameter descriptors;
- export только persistable subtree;
- import/apply только persistable subtree.

То есть не надо сразу переписывать UI model в реальное дерево.

---

# 10. Главные риски и как их контролировать

## Риск 1 — сломать GUI runtime
### Митигировать
- additive refactor;
- новый API рядом со старым;
- migration phase;
- сборка/ручная проверка после каждого этапа.

## Риск 2 — сохранить в осевой config мусор
### Митигировать
- ввести `persistable` policy;
- whitelist параметров;
- verification after apply.

## Риск 3 — mixed runtime startup policy станет хаотичным
### Митигировать
- явно формализовать `startup_policy`;
- не держать hidden special cases в GUI.

## Риск 4 — `AxisControlService` превратится в god object
### Митигировать
- не тащить туда Qt;
- не тащить туда JSON parser напрямую;
- не тащить presentation logic;
- держать в нём только orchestration + runtime/config/control/feedback.

---

# 11. Definition of Done
Рефакторинг можно считать завершённым, когда:

1. GUI запускает runtime через канонический path `AxisControlService`.
2. GUI умеет экспортировать **полный axis config**.
3. GUI умеет экспортировать **HAL runtime config**.
4. Есть сценарий `3 CAN + 3 EtherCAT` из одного master-config.
5. Вторая команда может headless:
   - загрузить configs,
   - стартовать runtime,
   - управлять осями,
   - читать feedback.
6. Для этого им не нужен Qt.
7. При этом используется тот же код, что и тестировался в GUI.

---

# 12. Моя рекомендация по внедрению

## Самый безопасный порядок
1. **Сначала**: config DTO + JSON + persistable model.
2. **Потом**: расширение `AxisControlService`.
3. **Потом**: перевод `AxisManager` на новый путь.
4. **Потом**: GUI export/import HAL configs.
5. **Потом**: mixed runtime validation.
6. **Потом**: handoff package второй команде.

Не наоборот.

---

Есть небольшой subset параметров, которые привод читает до того, как хост успел что-либо послать:

Параметр	Почему в EEPROM
CAN Node ID (MKS)	Нужен для коммуникации до первой команды
CAN Baudrate (MKS)	Аналогично
EtherCAT Station Address	Нужен при ENI scan
Всё остальное — PID, gear ratio, скорости, ускорения, encoder settings, режимы — можно и нужно применять из файла при каждом старте.

Да, ваш замысел правильный: **GUI должен остаться инструментом конфигурирования и теста, а `AxisControlService` — стать единой канонической базой, которую потом использует и вторая команда**. Лучший путь — не строить новый wrapper, а поэтапно расширить `AxisControlService` до runtime/config/control/feedback API, ввести два уровня конфигов (**полный осевой config** и **master HAL config**), перевести GUI на этот же path и только после этого делать handoff второй команде. Если делать это поэтапно, additive-style, то риск “сломать всё” нормальный и контролируемый.

Если хочешь, следующим сообщением я могу превратить этот план в **совсем прикладной backlog по файлам**:
- какие заголовки и cpp добавить,
- какие методы именно добавить в `axis_control_service.h/.cpp`,
- что вырезать из `AxisManager`,
- и в каком exact порядке это внедрять в кодовой базе. Если захотите реализацию — попросите меня **toggle to Act mode**.

По сути у вас теперь появляется очень чистая и правильная модель конфигов:

# 1. Разделение конфигурации на 2 класса

## A. Boot / identity parameters
Это то, что должно существовать **до первого сообщения от хоста**:
- `CAN Node ID`
- `CAN Baudrate`
- `EtherCAT Station Address`

Это действительно **не runtime-config**, а **identity / transport bootstrap config**.

### Вывод
Их **не надо считать частью обычного осевого runtime-конфига**.
Они должны жить отдельно, потому что без них сам runtime может не подняться.

---

## B. Runtime-applied parameters
Это всё остальное:
- PID
- gear ratio
- speed limits
- accel
- encoder settings
- modes
- software scaling
- safety/runtime settings

Это как раз идеально подходит под модель:
- GUI сохранил в файл,
- `AxisControlService` при старте применил,
- потом система начала работать.

### Вывод
Именно это и должно быть **основой axis config file**.

---

# 2. Что это меняет в новой архитектуре
Это значит, что у вас не один “полный конфиг”, а фактически **двухфазная конфигурация**:

## Фаза 1 — bootstrap
Нужно сначала суметь найти и открыть устройство/ось.
Для этого нужны:
- bus config,
- addressing,
- transport identity.

## Фаза 2 — runtime apply
После того как ось уже доступна по шине:
- читаем axis config file,
- применяем runtime parameters,
- верифицируем,
- стартуем рабочий цикл.

И это вообще очень здравая промышленная модель.

---

# 3. Как я бы теперь переопределил конфиги

## 3.1. `HalRuntimeConfig` становится главным bootstrap-файлом
Именно в нём должны жить:
- transport topology,
- cycle periods,
- buses,
- identity / addressing,
- axis type mapping,
- path к axis runtime-config file.

То есть для MKS:
- какой bus,
- какой bitrate,
- какой CAN node id у каждой оси.

Для EtherCAT:
- какой интерфейс/master,
- какая station address / slave position.

### Иными словами
Все параметры, без которых ось **нельзя обнаружить / открыть / привязать**, должны быть в HAL config.

---

## 3.2. `AxisConfig` становится чистым runtime-config файлом
И в него теперь должны входить только параметры, которые применяются **после открытия оси**:
- motion settings,
- PID,
- gains,
- accel/vel,
- scaling,
- encoder settings,
- homing/runtime policies,
- режимы и прочее.

### Это очень хорошо, потому что
- осевой конфиг становится transport-agnostic по жизненному циклу;
- он не содержит identity bootstrap, который опасно менять случайно;
- вторая команда не сможет сломать адресацию, случайно применив “полный snapshot”.

---

# 4. Что это значит для GUI
Это тоже упрощает GUI.

## GUI должен уметь делать 2 вещи
### 1. Настроить bootstrap identity
Это отдельный provisioning/setup сценарий:
- задать CAN ID,
- baudrate,
- EtherCAT station address,
- сохранить это в HAL config.

### 2. Настроить runtime parameters
Это текущий parameter tree / config tab:
- считать параметры,
- отредактировать,
- сохранить axis config,
- применить и протестировать.

То есть GUI остаётся единым инструментом, но у него теперь **два осмысленных режима**:
- provisioning,
- runtime tuning/configuration.

---

# 5. Как это влияет на `AxisControlService`
И это ещё один аргумент, почему именно `AxisControlService` надо расширять.

## Его новая роль становится очень ясной
`AxisControlService` **не должен управлять bootstrap identity как обычными runtime parameters**.

Он должен:
1. принять уже готовый `HalRuntimeConfig`;
2. открыть runtime по bootstrap settings;
3. поднять/зарегистрировать оси;
4. применить к ним `AxisConfig` runtime parameters;
5. дальше работать как control/feedback service.

### То есть он управляет
- runtime orchestration,
- runtime config application,
- axis control,
- feedback.

### Но не обязан напрямую заниматься
- low-level EEPROM provisioning CAN ID / baud / station address.

Это лучше держать как отдельный provisioning use-case.

---

# 6. Как теперь выглядит правильный старт системы

## Полный boot sequence
1. Загрузить `HalRuntimeConfig`.
2. Открыть MKS/EtherCAT buses по bootstrap settings.
3. Найти/привязать axes по `node id / station address`.
4. Создать и зарегистрировать `IAxis` adapters.
5. Для каждой оси загрузить её `AxisConfig`.
6. Применить runtime parameters.
7. Верифицировать apply.
8. Включить runtime loop.
9. Начать позиционное управление.

Это очень понятный и воспроизводимый pipeline.

---

# 7. Самое важное преимущество для второй команды
Вот теперь handoff становится реально безопасным.

## Что они получают
### 1. `HalRuntimeConfig`
Где уже описано:
- какие оси существуют,
- где они на шине,
- какие bus settings,
- какие axis config files привязаны.

### 2. `AxisConfig` на каждую ось
Где только runtime tuning/configuration.

### 3. `AxisControlService`
Который делает:
- bootstrap open,
- axis attach,
- runtime config apply,
- control,
- feedback.

### Что это значит practically
Они не редактируют руками transport identity.
Они просто берут утверждённые конфиги и запускают систему.

Это именно то, что вам нужно.

---

# 8. Как скорректировать план рефакторинга с учётом этого
Теперь я бы формулировал план так:

## Шаг 1. Явно разделить config domains
Ввести три понятия:
1. `Bootstrap/identity config`
2. `Axis runtime config`
3. `HAL runtime config`

Но физически файлов достаточно двух:
- `HalRuntimeConfig` содержит bootstrap + topology;
- `AxisConfig` содержит runtime params.

---

## Шаг 2. Ввести признак параметра
Для descriptors/parameter catalog нужно добавить что-то вроде:
- `persist_scope = RuntimeConfig | BootstrapIdentity | VolatileTelemetry | ReadOnlyStatus`

Даже если это будет не прямо enum в UI, а внутренняя policy-таблица.

Это нужно, чтобы:
- не экспортировать мусор,
- не писать в axis config bootstrap identity,
- не путать read-only и persistable параметры.

---

## Шаг 3. GUI разделить на provisioning и runtime config
Не обязательно визуально сразу сильно менять UI.
Можно сначала логически:
- вкладка параметров экспортирует только runtime config;
- provisioning identity живёт отдельно и сохраняется в HAL config.

---

## Шаг 4. `AxisControlService` расширить именно как runtime orchestrator
Он должен:
- открыть runtime по HAL config;
- применить axis runtime configs;
- начать работу.

---

## Шаг 5. Вторая команда использует только approved configs
Очень важный процессный момент:
- GUI — место создания и отладки конфигов;
- git — источник утверждённых конфигов;
- вторая команда — только потребитель утверждённых конфигов.

Это снижает риск ошибок сильнее любого рефакторинга.

---

# 9. Ответ в лоб на твой тезис
> Всё остальное — PID, gear ratio, скорости, ускорения, encoder settings, режимы — можно и нужно применять из файла при каждом старте.

Да. Я полностью согласен.

Это и есть правильная граница.

### Значит:
- `CAN ID / baud / station address` → **HAL/master/bootstrap config**
- всё остальное → **axis runtime config**

И это делает всю архитектуру намного чище.

---

# 10. Мой обновлённый вывод
С этим уточнением твоя целевая схема становится очень сильной:

## Правильная модель
- GUI = provisioning + runtime config + тест;
- `AxisControlService` = единая headless orchestration/control API;
- `HalRuntimeConfig` = topology + bootstrap;
- `AxisConfig` = runtime-applied full config;
- вторая команда = использует те же `AxisControlService + configs`, но без GUI.

## И это уже не выглядит как overengineering
Наоборот — это нормальная промышленная схема:
- identity отдельно,
- runtime config отдельно,
- orchestration отдельно,
- GUI как tool, а не как production logic owner.

Если хочешь, я следующим сообщением могу **переписать предыдущий implementation plan уже в окончательной версии с этим разделением**:
- bootstrap config,
- axis runtime config,
- изменения в `AxisControlService`,
- изменения в GUI,
- exact migration order.