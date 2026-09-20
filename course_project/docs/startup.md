# Збірка та запуск проекту НРК Розміновщик

## Збірка

```bash
cd course_project/robot_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
colcon build
source install/setup.bash
```

> **ВАЖЛИВО:** Після `colcon build` обов'язково виконайте `source install/setup.bash`!

## Критичні нюанси запуску

### 1. Source workspace в КОЖНОМУ терміналі

Кожен новий термінал потребує повторного сурсу workspace:

```bash
cd ~/cpp-miltech-study/course_project/robot_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash
```

**Без цього:**
- `ros2 interface show perimeter_msgs/msg/PerimeterStatus` → `The message type is invalid`
- `ros2 topic echo /perimeter/status` → `The message type is invalid`
- `ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode ...` → `The passed service type is invalid`

### 2. Shell замість bash

Якщо ваш shell за замовчуванням `/bin/sh` (не bash), використовуйте:

```bash
# Замість 'source' використовуйте '.' (крапка з пробілом)
. install/setup.bash

# Або запустіть через bash
bash -c "source install/setup.bash && ros2 topic list"
```

### 3. Fallback до дефолтної конфігурації

Якщо файл сценарію не знайдено, система використовує дефолтну конфігурацію:

```
[PerimeterLoader] Failed to open file: perimeter_miner/config/training_ground.yaml
[WARN] [miner_node]: Failed to load config from any path, using defaults
```

Це не помилка — робот продовжить роботу з дефолтним периметром (квадрат 20x20m).

### 4. Перевірка що типи зареєстровані

Після `source install/setup.bash`:

```bash
# Повинен повернути вміст .msg файлу
ros2 interface show perimeter_msgs/msg/PerimeterStatus

# Повинен повернути вміст .srv файлу
ros2 interface show perimeter_msgs/srv/SwitchMode
```

Якщо обидва повертають вміст — все працює.

## Параметри конфігурації

### Нові параметри (2026-09-19)

| Параметр | Тип | За замовчуванням | Опис |
|----------|-----|------------------|------|
| `scenario_file` | `string` | `training_ground.yaml` | Файл сценарію периметру |
| `config_search_paths` | `string[]` | `[]` | Додаткові шляхи для пошуку конфігурацій |

### Приклад використання config_search_paths:

```bash
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=patrol_alpha.yaml \
    config_search_paths:=["/home/user/custom_configs", "/opt/miltech/configs"]
```

## Запуск

### Базовий запуск з мінесимуляцією

> **ВАЖЛИВО:** `enable_fake_odom:=true` обов'язковий для роботи без Gazebo!

```bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true
```

### Запуск з HTTP звітністю

```bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_reporter:=true \
    api_endpoint:=http://localhost:8080
```

#### Тестування HTTP API

**Крок 1:** Запустіть локальний HTTP сервер (термінал 1):

```bash
cd course_project
python3 http_server.py
```

Або використайте вбудований Python сервер:

```bash
python3 -m http.server 8080 &
```

**Крок 2:** Запустіть систему з звітами (термінал 2):

```bash
cd course_project/robot_ws
source install/setup.bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_reporter:=true \
    api_endpoint:=http://localhost:8080
```

**Крок 3:** Перевірте через curl (термінал 3):

```bash
# Тестовий movement report
curl -X POST http://localhost:8080/api/v1/movement \
  -H "Content-Type: application/json" \
  -d '{"mode":0,"waypoint_index":3,"target_x":20.0,"target_y":20.0,"current_x":18.5,"current_y":19.2,"lateral_error":0.7,"speed":1.5,"mine_detected":false,"timestamp":"1700000000.123456"}'

# Тестовий mine detection
curl -X POST http://localhost:8080/api/v1/mine/detected \
  -H "Content-Type: application/json" \
  -d '{"mine_id":42,"x":15.5,"y":25.3,"type":"anti-tank","confidence":0.95,"detected_at":"1700000000.654321"}'

# Тестовий clearance report
curl -X POST http://localhost:8080/api/v1/mine/cleared \
  -H "Content-Type: application/json" \
  -d '{"mine_id":42,"x":15.5,"y":25.3,"method":"disposal","success":true,"details":"Clearance completed successfully","timestamp":"1700000001.123456"}'

# Тестовий mission summary
curl -X POST http://localhost:8080/api/v1/mission/summary \
  -H "Content-Type: application/json" \
  -d '{"scenario_name":"training_ground","result":"SUCCESS","reason":"All waypoints completed and all mines cleared","total_waypoints":4,"waypoints_completed":4,"mines_detected":3,"mines_cleared":3,"mission_duration":120.5,"coverage_percent":100.0,"start_time":"1700000000","end_time":"1700000120"}'
```

### Ендпоїнти HTTP API

| Метод | Шлях | Повідомлення | Призначення |
|-------|------|-------------|-------------|
| POST | `/api/v1/movement` | `PerimeterStatus` | Періодичний звіт позиції (кожні 5с) |
| POST | `/api/v1/mine/detected` | `MineDetection` | Детекція міни |
| POST | `/api/v1/mine/cleared` | `ClearanceReport` | Розмінування |
| POST | `/api/v1/mission/summary` | `MissionSummary` | Кінець місії |

### Формат JSON звітів

**Movement Report:**
```json
{
    "mode": 0,
    "waypoint_index": 3,
    "target_x": 20.0,
    "target_y": 20.0,
    "current_x": 18.5,
    "current_y": 19.2,
    "lateral_error": 0.7,
    "speed": 1.5,
    "mine_detected": false,
    "timestamp": "1700000000.123456"
}
```

**Mine Detection:**
```json
{
    "mine_id": 42,
    "x": 15.5,
    "y": 25.3,
    "type": "anti-tank",
    "confidence": 0.95,
    "detected_at": "1700000000.654321"
}
```

**Mission Summary:**
```json
{
    "scenario_name": "training_ground",
    "result": "SUCCESS",
    "reason": "All waypoints completed and all mines cleared",
    "total_waypoints": 4,
    "waypoints_completed": 4,
    "mines_detected": 3,
    "mines_cleared": 3,
    "mission_duration": 120.5,
    "coverage_percent": 100.0,
    "start_time": "1700000000",
    "end_time": "1700000120"
}
```

### Запуск з Gazebo симуляцією

> **Примітка:** З Gazebo `enable_fake_odom` НЕ потрібен — Gazebo генерує одометрію.

```bash
ros2 launch perimeter_miner system.launch.py \
    use_gazebo:=true \
    simulate_mines:=true
```

### Запуск з телеоперацією (клавіатура)

```bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_teleop:=true \
    teleop_input_type:=keyboard
```

### Запуск з телеоперацією (gamepad)

```bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_teleop:=true \
    teleop_input_type:=joy
```

### Повний запуск (все разом)

> **Примітка:** З Gazebo `enable_fake_odom` НЕ потрібен.

```bash
ros2 launch perimeter_miner system.launch.py \
    use_gazebo:=true \
    simulate_mines:=true \
    enable_reporter:=true \
    api_endpoint:=http://localhost:8080 \
    enable_teleop:=true \
    teleop_input_type:=keyboard
```

## Перемикання режимів під час роботи

| Режим | ID | Опис |
|-------|----|------|
| AUTONOMOUS | 0 | Патрулювання периметру / покриття області |
| TELEOP | 1 | Телеоперація (оператор має пріоритет) |
| HOLD | 2 | Утримання позиції (при детекції міни) |
| AREA_COVERAGE | 3 | Покриття області (zigzag pattern) |

```bash
# AUTONOMOUS → TELEOP (перехоплення оператором)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 1}"

# TELEOP → HOLD (утримання позиції)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 2}"

# HOLD → AUTONOMOUS (повернення до автономного режиму)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 0}"

# AUTONOMOUS → AREA_COVERAGE (покриття області зигзагом)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 3}"
```

> **Примітка:** Перемикання в режим HOLD відбувається автоматично при детекції міни. Повернення до AUTONOMOUS — коли robot at hold position + clearance confirmed. TELEOP має timeout 30s — після цього автоматичне повернення до AUTONOMOUS.

## Розмінування

```bash
# Активація розмінування міни
ros2 service call /control/trigger_clearance perimeter_msgs/srv/TriggerClearance \
    "{mine_id: 1, method: \"disposal\"}"
```

## Окремі пакети

```bash
# Mine spawner (симуляція мін)
ros2 launch mine_simulator mine_spawner.launch.py

# HTTP reporter (звітність)
ros2 launch http_reporter reporter.launch.py api_endpoint:=http://localhost:8080

# Teleop operator (телеоперація)
ros2 launch teleop_operator teleop.launch.py input_type:=keyboard
```

## Тести

### Unit тести

```bash
cd course_project/robot_ws
colcon test
colcon test-result --all
```

### Інтеграційні тести

```bash
./run_all_tests.sh
```

### Окремий сценарій

```bash
./run_test.sh training_ground.yaml
./run_test.sh patrol_alpha.yaml
./run_test.sh large_patrol.yaml
```

## Перегляд топіків

### Список всіх топіків
```bash
ros2 topic list
```

### Основні топіки

| Топік | Повідомлення | Призначення | Частота |
|-------|-------------|-------------|---------|
| `/control/cmd_vel` | `TwistStamped` | Команди руху робота | 50 Hz |
| `/perimeter/status` | `PerimeterStatus` | Статус патрулювання | 2 Hz |
| `/mines/detected` | `MineDetection` | Детекція мін | Подійна |
| `/mines/cleared` | `ClearanceReport` | Звіт про розмінування | Подійна |
| `/mission/summary` | `MissionSummary` | Підсумок місії | Подійна (кінець) |
| `/control/status` | `PerimeterStatus` | Синхронізація режиму | 2 Hz |
| `/robot/position` | `std_msgs/String` | Позиція ("x,y") | Подійна |
| `/reporter/status` | `std_msgs/String` | Статус HTTP репортера | Подійна |
| `/teleop/cmd` | `std_msgs/Empty` | Сигнал телеоперації | Подійна |
| `/odom` | `nav_msgs/Odometry` | Одометрія (вхід) | Залежить від симуляції |

### Перегляд топіків
```bash
# Статус патрулювання
ros2 topic echo /perimeter/status

# Детекція мін
ros2 topic echo /mines/detected

# Розмінування (НОВЕ)
ros2 topic echo /mines/cleared

# Підсумок місії (НОВЕ)
ros2 topic echo /mission/summary

# Команди руху
ros2 topic echo /control/cmd_vel
```

## Rosbag запис

### Запис всіх топіків

```bash
ros2 bag record -o my_mission_bag /perimeter/status /mines/detected /control/cmd_vel
```

### Запуск з run_test.sh (автоматичний)

Скрипт автоматично запускає rosbag запис під час інтеграційних тестів.
Результати зберігаються в `bags/<scenario_name>/`.

## Конфігурація периметрів

| Файл | Опис |
|------|------|
| [`training_ground.yaml`](../robot_ws/src/perimeter_miner/config/training_ground.yaml) | Квадратний **замкнений** периметр 20x20m, **4 waypoint**, 3 міни |
| [`patrol_alpha.yaml`](../robot_ws/src/perimeter_miner/config/patrol_alpha.yaml) | Лінійний **розімкнений** патруль, **4 waypoint**, 3 міни |
| [`large_patrol.yaml`](../robot_ws/src/perimeter_miner/config/large_patrol.yaml) | Великий **замкнений** периметр (овальний), **15 waypoint**, 8 мін |
| [`training_ground_coverage.yaml`](../robot_ws/src/perimeter_miner/config/training_ground_coverage.yaml) | Покриття площі зигзагом (boustrophedon), pass_spacing=2m |

> **Примітка:** `start_corner` в `training_ground_coverage.yaml` ще не реалізований — завжди використовується BL (bottom-left).

```bash
# Запуск з конкретним сценарієм (без Gazebo)
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=patrol_alpha.yaml \
    simulate_mines:=true \
    enable_fake_odom:=true
```

```bash
# Запуск з конкретним сценарієм (з Gazebo — fake_odom не потрібен)
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=patrol_alpha.yaml \
    use_gazebo:=true \
    simulate_mines:=true
```

## Покриття площі (Area Coverage)

Для покриття всього периметру (а не тільки краю), використовуйте режим AREA_COVERAGE:

```bash
# Запуск з покриттям площі зигзагом
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=training_ground_coverage.yaml \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_coverage:=true
```

### Як це працює:

Алгоритм **boustrophedon** (зигзаг) генерує паралельні лінії сканування:

```
┌─────────────────────────────┐
│ → → → → → → → → → → → →   │  Pass 1 (left→right)
├─────────────────────────────┤
│ ← ← ← ← ← ← ← ← ← ← ← ←   │  Pass 2 (right→left)
├─────────────────────────────┤
│ → → → → → → → → → → → →   │  Pass 3 (left→right)
├─────────────────────────────┤
│ ← ← ← ← ← ← ← ← ← ← ← ←   │  Pass 4 (right→left)
└─────────────────────────────┘
```

### Налаштування coverage конфігурації:

```yaml
# training_ground_coverage.yaml
name: training_ground_coverage
type: coverage

# Бounding box області
min_x: 0.0
min_y: 0.0
max_x: 20.0
max_y: 20.0

# Параметри покриття
pass_spacing: 2.0       # відстань між паралельними лініями (м)
coverage_speed: 1.0     # швидкість під час покриття (м/с)
turn_arcs: 1.0          # радіус розвороту на кінцях ліній

# Напрямок сканування: 'X'=горизонтально, 'Y'=вертикально
scan_direction: X

# Кут початку: BL=зліва-знизу, BR=справа-знизу, TL=зліва-зверху, TR=справа-зверху
start_corner: BL
```
