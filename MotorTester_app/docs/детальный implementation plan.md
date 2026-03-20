Ниже — **детальный implementation plan**, исходя из твоего решения: **не плодить новый wrapper, а сделать `Модуль` канонической headless API-точкой** для:
- вашего Qt GUI,
- CLI,
- и второй команды.

---

# 0. История и цель рефакторинга

## Что есть сейчас
Сейчас проект уже имеет хороший базовый каркас:
- `IAxis` как общий контракт оси,
- сервис управления осью,
- MKS/EtherCAT adapters,
- runtime factories,
- Qt GUI как средство конфигурирования и теста.

Но логика пока разделена неидеально:
- часть orchestration живёт своей жизнью,
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
- **было одно каноническое API** для runtime/config/control/feedback;
- `AxisManager` остался тонким Qt bridge;
- конфиги были простыми, полными и переносимыми.

---

# 1. Целевая архитектура

## 1.1. Верхнеуровневая схема



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
