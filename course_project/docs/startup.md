# Збірка та запуск проекту НРК Розміновщик

## Збірка

```bash
cd course_project/robot_ws
colcon build --packages-select perimeter_msgs perimeter_miner mine_simulator http_reporter teleop_operator
source install/setup.bash
```

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
