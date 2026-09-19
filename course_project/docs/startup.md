# Збірка та запуск проекту НРК Розміновщик

## Збірка

```bash
cd course_project/robot_ws
colcon build --packages-select perimeter_msgs perimeter_miner mine_simulator http_reporter teleop_operator
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

```bash
ros2 launch perimeter_miner system.launch.py simulate_mines:=true
```

### Запуск з HTTP звітністю

```bash
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
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

```bash
ros2 launch perimeter_miner system.launch.py \
    use_gazebo:=true \
    simulate_mines:=true
```

### Запуск з телеоперацією (клавіатура)

```bash
ros2 launch perimeter_miner system.launch.py \
    enable_teleop:=true \
    teleop_input_type:=keyboard
```

### Запуск з телеоперацією (gamepad)

```bash
ros2 launch perimeter_miner system.launch.py \
    enable_teleop:=true \
    teleop_input_type:=joy
```

### Повний запуск (все разом)

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

```bash
# AUTONOMOUS → TELEOP (перехоплення оператором)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 1}"

# TELEOP → HOLD (утримання позиції)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 2}"

# HOLD → AUTONOMOUS (повернення до автономного режиму)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode "{mode: 0}"
```

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
colcon test --packages-select perimeter_msgs perimeter_miner mine_simulator http_reporter teleop_operator
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

| Топік | Повідомлення | Призначення |
|-------|-------------|-------------|
| `/perimeter/status` | `PerimeterStatus` | Статус патрулювання |
| `/mines/detected` | `MineDetection` | Детекція мін |
| `/mines/cleared` | `ClearanceReport` | Звіт про розмінування (НОВЕ) |
| `/mission/summary` | `MissionSummary` | Підсумок місії (НОВЕ) |
| `/control/cmd_vel` | `TwistStamped` | Команди руху |
| `/teleop/cmd` | `Empty` | Сигнал телеоперації (НОВЕ) |

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
| `training_ground.yaml` | Квадратний периметр 20x20m, 4 waypoint, 3 міни |
| `patrol_alpha.yaml` | Лінійний патруль, 4 waypoint, 3 міни |
| `large_patrol.yaml` | Великий замкнений периметр, 16 waypoint, 8 мін |

```bash
# Запуск з конкретним сценарієм
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=patrol_alpha.yaml \
    simulate_mines:=true
```

## Випадки використання

### Сценарій 1: Тестування автономного патрулювання

```bash
cd course_project/robot_ws
colcon build --packages-select perimeter_msgs perimeter_miner mine_simulator
source install/setup.bash
ros2 launch perimeter_miner system.launch.py simulate_mines:=true
```

### Сценарій 2: Тестування з HTTP звітністю

```bash
# Запуск локального серверу для прийому звітів (опціонально)
python3 -m http.server 8080 &

ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_reporter:=true \
    api_endpoint:=http://localhost:8080
```

### Сценарій 3: Повне тестування з Gazebo

```bash
ros2 launch perimeter_miner system.launch.py \
    use_gazebo:=true \
    simulate_mines:=true
```

### Сценарій 4: Телеоперація

```bash
# Запуск телеоперації в одному терміналі
ros2 launch teleop_operator teleop.launch.py input_type:=keyboard

# Запуск основного контролера в іншому терміналі
ros2 launch perimeter_miner system.launch.py enable_teleop:=true
```
