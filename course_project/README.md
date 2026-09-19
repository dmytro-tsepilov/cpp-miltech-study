# НРК Розміновщик — Периметровий Патрульний Робот

## Опис проекту

Автономний наземний роботизований комплекс (НРК) для патрулювання периметру та розмінування. 
Підтримує три режими керування: **автономний**, **телеоперація** (з перехопленням оператором), **утримання позиції**.

## Архітектура системи

```
┌─────────────────────────────────────────────────────────────────────┐
│                     System Architecture                            │
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐     │
│  │   miner_node  │    │mode_switch_  │    │  http_reporter   │     │
│  │  (tracking)   │◄──►│   node       │◄──►│   (API reports)  │     │
│  └──────┬───────┘    └──────────────┘    └──────────────────┘     │
│         │                                                           │
│         ▼                                                           │
│  ┌──────────────┐    ┌──────────────┐                             │
│  │ mine_spawner │    │ teleop_node  │                             │
│  │ (simulation) │    │ (operator)   │                             │
│  └──────────────┘    └──────────────┘                             │
│                                                                     │
│  Topics:                                                            │
│  /control/cmd_vel      → Robot movement commands                   │
│  /perimeter/status     → Perimeter patrol status                  │
│  /mines/detected       → Mine detection events                    │
│  /robot/position       → Robot position (for simulation)          │
│  /reporter/status      → HTTP reporter status                     │
│                                                                     │
│  Services:                                                          │
│  /control/switch_mode    → Mode switching (AUTONOMOUS/TELEOP/HOLD)│
│  /control/trigger_clearance → Mine clearance trigger              │
└─────────────────────────────────────────────────────────────────────┘
```

## Пакети проекту

| Пакет | Призначення |
|-------|-------------|
| `perimeter_msgs` | Спільні повідомлення та сервіси |
| `perimeter_miner` | Основний контролер периметру + mode switch |
| `mine_simulator` | Симуляція мін для тестування |
| `http_reporter` | HTTP звітність через API |
| `teleop_operator` | Телеоперація (gamepad/keyboard) |

## Топіки

| Топік | Повідомлення | Призначення |
|-------|-------------|-------------|
| `/control/cmd_vel` | `geometry_msgs/TwistStamped` | Команда руху робота |
| `/perimeter/status` | `PerimeterStatus` | Статус патрулювання |
| `/mines/detected` | `MineDetection` | Детекція мін |
| `/mines/cleared` | `ClearanceReport` | Звіт про розмінування |
| `/mission/summary` | `MissionSummary` | Підсумок місії |
| `/robot/position` | `std_msgs/String` | Позиція робота (симуляція) |
| `/reporter/status` | `std_msgs/String` | Статус HTTP репортера |

## Сервіси

| Сервіс | Тип | Призначення |
|--------|-----|-------------|
| `/control/switch_mode` | `SwitchMode` | Перемикання режимів |
| `/control/trigger_clearance` | `TriggerClearance` | Активація розмінування |

## Конфігурація периметрів

### training_ground.yaml
Замкнений квадратний периметр (4 waypoint'и):
```
Waypoints: (0,0) → (20,0) → (20,20) → (0,20) → (0,0)
Mines: 3 (anti-tank, anti-personnel, unknown)
```

### patrol_alpha.yaml
Розімкнений лінійний патруль (4 waypoint'и):
```
Waypoints: (0,0) → (15,5) → (30,10) → (45,15)
Mines: 3
```

### large_patrol.yaml
Великий замкнений периметр (16 waypoint'ів):
```
Овальний маршрут з багатьма поворотами
Mines: 8
```

## Запуск

### Базовий запуск
```bash
cd robot_ws
source install/setup.bash

# Запуск основних нод
ros2 launch perimeter_miner system.launch.py

# З мінесимуляцією
ros2 launch perimeter_miner system.launch.py simulate_mines:=true

# З HTTP звітністю
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_reporter:=true \
    api_endpoint:=http://192.168.1.100:8080
```

### Перемикання режимів
```bash
# AUTONOMOUS → TELEOP (operator override)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 1}"

# TELEOP → HOLD
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 2}"

# HOLD → AUTONOMOUS
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 0}"
```

### Запуск окремих нод
```bash
# Mine spawner
ros2 launch mine_simulator mine_spawner.launch.py

# HTTP reporter
ros2 launch http_reporter reporter.launch.py \
    api_endpoint:=http://localhost:8080
```

## Тестування

### Unit тести
```bash
cd robot_ws
colcon test --packages-select perimeter_miner
colcon test --packages-select mine_simulator
colcon test --packages-select http_reporter

# Перегляд результатів
colcon test-result --all
```

### Інтеграційні тести
```bash
# Запуск всіх сценаріїв
./run_all_tests.sh

# Окремий сценарій
./run_test.sh training_ground
```

## HTTP API

### Ендпоінти

| Метод | Шлях | Тіло | Призначення |
|-------|------|------|-------------|
| POST | `/api/v1/movement` | PerimeterStatus | Періодичний звіт (5с) |
| POST | `/api/v1/mine/detected` | MineDetection | Детекція міни |
| POST | `/api/v1/mine/cleared` | ClearanceReport | Розмінування |
| POST | `/api/v1/mission/summary` | MissionSummary | Кінець місії |

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

## Алгоритми керування

### Perimeter Tracker
- **PID контролер** для бічного відхилення: `kp=2.0, ki=0.5, kd=0.3`
- **Pure Pursuit** навігація до waypoint'ів
- Автоматичне переключення waypoint'ів

### Mode Switch
- **AUTONOMOUS**: Периметр-трекер
- **TELEOP**: Оператор має пріоритет (миттєве перехоплення)
- **HOLD**: Утримання позиції при детекції міни

### Hold Controller
- PID для позиції: `kp=2.0, ki=0.5, kd=0.3`
- PID для курсу: `kp=3.0, ki=0.5, kd=0.2`
- Anti-windup захист

## Структура файлів

```
course_project/
├── README.md                          # Цей файл
├── http_server.py                     # Демо HTTP сервер для прийому звітів
├── run_test.sh                        # Окремий інтеграційний тест
├── run_all_tests.sh                   # Пакетний запуск всіх тестів
├── robot_ws/
│   └── src/
│       ├── perimeter_msgs/            # Спільні повідомлення та сервіси
│       │   ├── msg/                   # .msg файли (PerimeterStatus, MineDetection, etc.)
│       │   └── srv/                   # .srv файли (SwitchMode, TriggerClearance)
│       ├── perimeter_miner/           # Основний пакет контролера
│       │   ├── include/perimeter_miner/   # Заголовки (PID, tracker, mode switch)
│       │   ├── src/                       # Реалізація нод
│       │   ├── launch/                    # system.launch.py
│       │   ├── config/                    # YAML периметри (training_ground, patrol_alpha, large_patrol)
│       │   └── test/                      # Unit тести
│       ├── mine_simulator/            # Симуляція мін
│       │   ├── include/mine_simulator/
│       │   ├── src/
│       │   ├── launch/                # mine_spawner.launch.py, gazebo_simulation.launch.py
│       │   ├── config/                # training_ground_mines.yaml
│       │   ├── sdf/                   # Gazebo SDF моделі
│       │   └── urdf/                  # URDF файли
│       ├── http_reporter/             # HTTP звітність через API
│       │   ├── include/http_reporter/
│       │   ├── src/
│       │   ├── launch/                # reporter.launch.py
│       │   └── test/                  # http_reporter_test.cpp
│       └── teleop_operator/           # Телеоперація (gamepad/keyboard)
│           ├── include/teleop_operator/
│           ├── src/
│           └── launch/                # teleop.launch.py
├── docs/                              # Документація
│   ├── architecture.md                # Архітектура системи
│   ├── implementation_review.md       # Огляд реалізації
│   ├── implementation_review_fixed.md # Виправлений огляд
│   ├── startup.md                     # Посібник запуску
│   ├── test_analysis.md               # Аналіз тестів
│   └── tests_added_summary.md         # Додавання тестів
├── install/                           # Colcon інсталяція (build artifacts)
├── log/                               # Colcon логи
└── bags/                              # Rosbag записи (генерається під час тестів)
```

## Критерії успіху

1. ✅ Робот повністю обходить замкнений периметр без втрати треку
2. ✅ Бічне відхилення < 0.5 м на прямих ділянках
3. ✅ Mode switch працює коректно (autonomous ↔ teleop)
4. ✅ Оператор може примусово перехопити керування
5. ✅ HTTP звіти надходять на сервер
6. ✅ Rosbag записує всі топіки
7. ✅ Mine simulator детектує міни з заданою ймовірністю

## Залежності

### Системні
- ROS 2 Jazzy (або Humble)
- CMake 3.8+
- GCC 11+ / Clang 14+
- libcurl

### ROS пакети
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `builtin_interfaces`
- `rosidl_default_runtime`
- `nlohmann-json`

### Тестування
- `ament_cmake_gtest`
- `yaml_cpp_vendor`

## Примітки

- Конфігурації периметрів знаходяться в [`robot_ws/src/perimeter_miner/config/`](robot_ws/src/perimeter_miner/config/)
- Тести пакетів розташовані в підпапках `test/` кожного пакета
- HTTP сервер для прийому звітів: [`http_server.py`](http_server.py)

## Ліцензія

MIT License

## Автор

Студентський проєкт для курсової роботи з робототехніки
