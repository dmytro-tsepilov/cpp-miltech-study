# НРК Розміновщик — Периметровий Патрульний Робот

## Опис проекту

Автономний наземний роботизований комплекс (НРК) для патрулювання периметру та розмінування. 
Підтримує чотири режими керування: **автономний**, **телеоперація** (з перехопленням оператором), **утримання позиції**, **покриття області** (boustrophedon/zigzag pattern).

## Архітектура системи

```
┌─────────────────────────────────────────────────────────────────────┐
│                     System Architecture                            │
│                                                                     │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐  │
│  │   miner_node  │◄──►│ mode_switch_node │◄──►│  http_reporter   │  │
│  │  (tracker +   │    │  (service +      │    │   (API reports)  │  │
│  │   controller) │    │   topic sync)    │    └──────────────────┘  │
│  └──────┬───────┘    └──────────────────┘                         │
│         │                                                          │
│         ▼                                                          │
│  ┌──────────────┐    ┌──────────────┐                             │
│  │ mine_spawner │    │ teleop_node  │                             │
│  │ (simulation) │    │ (operator)   │                             │
│  └──────────────┘    └──────────────┘                             │
│                                                                     │
│  Communication:                                                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Topics:                                                    │   │
│  │  /control/cmd_vel      → Robot movement commands           │   │
│  │  /perimeter/status     → Perimeter patrol status (500ms)   │   │
│  │  /mines/detected       → Mine detection events             │   │
│  │  /mines/cleared        → Clearance reports                 │   │
│  │  /mission/summary      → Mission summary on completion     │   │
│  │  /control/status       → Mode status sync (topic-based)    │   │
│  │  /robot/position       → Robot position (simulation)       │   │
│  │  /reporter/status      → HTTP reporter status              │   │
│  │  /teleop/cmd           → Teleop input signal               │   │
│  │  /odom                 → Odometry input                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  Services:                                                         │
│  /control/switch_mode    → Mode switching (AUTONOMOUS/TELEOP/HOLD/ │
│                            AREA_COVERAGE)                          │   │
│  /control/trigger_clearance → Mine clearance trigger              │
│                                                                     │
│  Data Flow:                                                        │
│  mine_spawner ──(/mines/detected)──► miner_node                   │
│  mode_switch_node ──(/control/status)──► miner_node               │
│  miner_node ──(/perimeter/status)──► http_reporter                │
│  miner_node ──(/mines/cleared)──► http_reporter                   │
│  miner_node ──(/mission/summary)──► http_reporter                 │
│  miner_node ──(/control/cmd_vel)──► [robot controller]            │
└─────────────────────────────────────────────────────────────────────┘
```

## Пакети проекту

| Пакет | Призначення | Ключові файли |
|-------|-------------|---------------|
| [`perimeter_msgs`](robot_ws/src/perimeter_msgs/) | Спільні повідомлення та сервіси | `PerimeterStatus.msg`, `MineDetection.msg`, `SwitchMode.srv` |
| [`perimeter_miner`](robot_ws/src/perimeter_miner/) | Основний контролер периметру + mode switch | [`miner_node.cpp`](robot_ws/src/perimeter_miner/src/miner_node.cpp), [`mode_switch_node.cpp`](robot_ws/src/perimeter_miner/src/mode_switch_node.cpp) |
| [`mine_simulator`](robot_ws/src/mine_simulator/) | Симуляція мін для тестування | [`mine_spawner_node.cpp`](robot_ws/src/mine_simulator/src/mine_spawner_node.cpp) |
| [`http_reporter`](robot_ws/src/http_reporter/) | HTTP звітність через API | [`reporter_node.cpp`](robot_ws/src/http_reporter/src/reporter_node.cpp) |
| [`teleop_operator`](robot_ws/src/teleop_operator/) | Телеоперація (gamepad/keyboard) | [`teleop_driver.cpp`](robot_ws/src/teleop_operator/src/teleop_driver.cpp) |

## Топіки

| Топік | Повідомлення | Призначення | Частота |
|-------|-------------|-------------|---------|
| `/control/cmd_vel` | `geometry_msgs/TwistStamped` | Команда руху робота | 50 Hz |
| `/perimeter/status` | `PerimeterStatus` | Статус патрулювання | 2 Hz |
| `/mines/detected` | `MineDetection` | Детекція міни | Подійна |
| `/mines/cleared` | `ClearanceReport` | Звіт про розмінування | Подійна |
| `/mission/summary` | `MissionSummary` | Підсумок місії | Подійна (кінець) |
| `/control/status` | `PerimeterStatus` | Синхронізація режиму | 2 Hz |
| `/robot/position` | `std_msgs/String` | Позиція робота ("x,y" формат) | Подійна |
| `/reporter/status` | `std_msgs/String` | Статус HTTP репортера | Подійна |
| `/teleop/cmd` | `std_msgs/Empty` | Сигнал телеоперації | Подійна |
| `/odom` | `nav_msgs/Odometry` | Одометрія (вхід) | Залежить від симуляції |

## Сервіси

| Сервіс | Тип | Призначення |
|--------|-----|-------------|
| `/control/switch_mode` | [`SwitchMode`](robot_ws/src/perimeter_msgs/srv/SwitchMode.srv) | Перемикання режимів (0=AUTONOMOUS, 1=TELEOP, 2=HOLD, 3=AREA_COVERAGE) |
| `/control/trigger_clearance` | [`TriggerClearance`](robot_ws/src/perimeter_msgs/srv/TriggerClearance.srv) | Активація розмінування |

## Повідомлення (Message Types)

### PerimeterStatus
```
uint8 mode                 # 0=AUTONOMOUS, 1=TELEOP, 2=HOLD, 3=AREA_COVERAGE
uint32 waypoint_index      # Поточний індекс waypoint (0-based)
float64 target_x/y         # Цільова позиція (метри)
float64 current_x/y        # Поточна позиція робота (метри)
float64 lateral_error      # Бічне відхилення від шляху (метри)
float32 speed              # Поточна швидкість (м/с)
bool mine_detected         # Чи детектовано міну
builtin_interfaces/Time timestamp
```

### MineDetection
```
int32 mine_id              # Унікальний ID міни
float64 x/y                # Позиція (метри)
string type                # "anti-tank" | "anti-personnel" | "unknown"
float32 confidence         # Достовірність детекції 0.0 - 1.0
builtin_interfaces/Time detected_at
```

### MissionSummary
```
string scenario_name       # Назва сценарію
string result              # "SUCCESS" | "FAILED" | "ABORTED"
string reason              # Причина результату
uint32 total_waypoints     # Загальна кількість waypoint
uint32 waypoints_completed # Виконано waypoint
uint32 mines_detected      # Детековано мін
uint32 mines_cleared       # Розміновано мін
float64 mission_duration   # Тривалість місії (сек)
float64 coverage_percent   # Покриття периметру (0-100)
builtin_interfaces/Time start_time/end_time
```

### ClearanceReport
```
int32 mine_id              # ID розмінованої міни
float64 x/y                # Позиція (метри)
string method              # "disposal" | "marking" | "neutralization"
bool success               # Чи успішне розмінування
string details             # Додаткові деталі
builtin_interfaces/Time timestamp
```

## Конфігурація периметрів

### training_ground.yaml (Замкнений квадрат)
```yaml
name: training_ground
closed_loop: true
tolerance: 0.5
max_speed: 2.0
min_turn_radius: 1.0

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  - {x: 20.0, y: 0.0, heading: 1.5708, radius: 1.0}
  - {x: 20.0, y: 20.0, heading: 3.1416, radius: 1.0}
  - {x: 0.0, y: 20.0, heading: -1.5708, radius: 1.0}

mines:
  - {id: 1, x: 10.0, y: 5.0, type: "anti-tank"}
  - {id: 2, x: 15.0, y: 15.0, type: "anti-personnel"}
  - {id: 3, x: 5.0, y: 10.0, type: "unknown"}
```

### patrol_alpha.yaml (Розімкнений лінійний патруль)
```yaml
name: patrol_alpha
closed_loop: false
tolerance: 0.8
max_speed: 1.5
min_turn_radius: 1.5

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.5}
  - {x: 15.0, y: 5.0, heading: 0.349, radius: 1.5}
  - {x: 30.0, y: 10.0, heading: 0.349, radius: 1.5}
  - {x: 45.0, y: 15.0, heading: 0.349, radius: 1.5}

mines:
  - {id: 1, x: 10.0, y: 3.0, type: "anti-tank"}
  - {id: 2, x: 25.0, y: 8.0, type: "anti-personnel"}
  - {id: 3, x: 40.0, y: 13.0, type: "unknown"}
```

### large_patrol.yaml (Великий замкнений периметр)
```yaml
name: large_patrol
closed_loop: true
tolerance: 1.0
max_speed: 2.0
min_turn_radius: 2.0

waypoints: [15 waypoint по колу]
mines: [8 мін різних типів]
```

### training_ground_coverage.yaml (Покриття області)
```yaml
name: training_ground_coverage
type: coverage

min_x: 0.0
min_y: 0.0
max_x: 20.0
max_y: 20.0

pass_spacing: 2.0        # Відстань між паралельними проходами (м)
coverage_speed: 1.0      # Швидкість під час покриття (м/с)
turn_arcs: 1.0           # Довжина U-turn (м)
scan_direction: X        # 'X'=горизонтальні, 'Y'=вертикальні проходи
start_corner: BL         # Кут початку (BL/BR/TL/TR)
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

# З fake odometry (без Gazebo)
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true

# З HTTP звітністю
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_reporter:=true \
    api_endpoint:=http://192.168.1.100:8080

# З телеоперацією
ros2 launch perimeter_miner system.launch.py \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_teleop:=true \
    teleop_input_type:=keyboard

# З покриттям області (zigzag pattern)
ros2 launch perimeter_miner system.launch.py \
    scenario_file:=training_ground_coverage.yaml \
    enable_coverage:=true \
    simulate_mines:=true \
    enable_fake_odom:=true
```

### Перемикання режимів
```bash
# AUTONOMOUS → TELEOP (operator override)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 1}"

# TELEOP → HOLD (зупинка при детекції міни)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 2}"

# HOLD → AUTONOMOUS (повернення до патрулювання)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 0}"

# AUTONOMOUS → AREA_COVERAGE (покриття області)
ros2 service call /control/switch_mode perimeter_msgs/srv/SwitchMode \
    "{mode: 3}"
```

### Запуск окремих нод
```bash
# Mine spawner
ros2 launch mine_simulator mine_spawner.launch.py

# HTTP reporter
ros2 launch http_reporter reporter.launch.py \
    api_endpoint:=http://localhost:8080

# Fake odometry publisher
ros2 launch mine_simulator fake_odom_publisher.launch.py

# Teleoperation
ros2 launch teleop_operator teleop.launch.py input_type:=keyboard
```

## Тестування

### Unit тести
```bash
cd robot_ws

# Перевірка тестів пакетів
colcon test --packages-select perimeter_miner mine_simulator http_reporter

# Перегляд результатів
colcon test-result --all

# Запуск окремих тестів
colcon test --packages-select perimeter_miner --ctest-args -R HoldControllerPIDTest
colcon test --packages-select perimeter_miner --ctest-args -R ModeSwitchTest
colcon test --packages-select perimeter_miner --ctest-args -R PerimeterTrackerTest
colcon test --packages-select mine_simulator --ctest-args -R MineDefinitionTest
```

### Інтеграційні тести
```bash
# Запуск всіх сценаріїв
./run_all_tests.sh

# Окремий сценарій
./run_test.sh training_ground.yaml
./run_test.sh patrol_alpha.yaml
./run_test.sh large_patrol.yaml
```

## HTTP API

### Ендпоїнти

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

### Perimeter Tracker ([`PerimeterTracker`](robot_ws/src/perimeter_miner/include/perimeter_miner/perimeter_tracker.hpp))

#### Lateral PID Controller ([`LateralPID`](robot_ws/src/perimeter_miner/include/perimeter_miner/perimeter_tracker.hpp))
- **Параметри:** `kp=2.0, ki=0.5, kd=0.3`
- **Обмеження:** max_output=1.5, max_integral=5.0
- **Призначення:** Бічне відхилення від сегмента периметру
- **Anti-windup:** Так (clamp integral)

#### Pure Pursuit Navigator ([`PurePursuit`](robot_ws/src/perimeter_miner/include/perimeter_miner/perimeter_tracker.hpp))
- **Look-ahead:** min=1.0m, max=3.0m, gain=1.5
- **Адаптивний:** Залежить від швидкості робота
- **Формула:** `curvature = 2.0 * sin(alpha) / lookahead`

#### Perimeter Tracker Logic
```
1. Отримати поточну позицію з odometry
2. Визначити цільовий waypoint
3. Обчислити lateral error (cross product)
4. Lateral PID → steering component
5. Heading error × 3.0 + steering × 0.5 → angular_z
6. Linear speed: max_speed × clamp(dist/(tolerance*3), 0.3, 1.0)
7. Waypoint advance: dist < tolerance → next waypoint
```

### Mode Switch ([`ModeSwitch`](robot_ws/src/perimeter_miner/include/perimeter_miner/mode_switch.hpp))

| Режим | ID | Опис |
|-------|----|------|
| AUTONOMOUS | 0 | Периметр-трекер / coverage |
| TELEOP | 1 | Оператор має пріоритет |
| HOLD | 2 | Утримання позиції при детекції міни |
| AREA_COVERAGE | 3 | Boustrophedon zigzag pattern |

#### Перехід між режимами:
- **AUTONOMOUS → TELEOP:** Миттєве перехоплення (operator override)
- **TELEOP → HOLD:** Автоматично при детекції міни
- **HOLD → AUTONOMOUS:** Коли robot at hold position + mine_cleared = true
- **TELEOP/AUTONOMOUS → HOLD:** При детекції міни з `mine_detected_ = true`
- **Timeout:** TELEOP → AUTONOMOUS після 30s без телеоп вводу

#### Safety Checks:
- Повернення до AUTONOMOUS: `|lateral_error| < 5.0m`
- Operator override завжди успішний (пріоритет)

### Hold Controller ([`HoldController`](robot_ws/src/perimeter_miner/include/perimeter_miner/hold_controller.hpp))

#### Position PID:
- **Параметри:** `pos_kp=2.0, pos_ki=0.5, pos_kd=0.3`
- **Обмеження:** max_linear=1.0 m/s
- **Integral limits:** ±5.0

#### Heading PID:
- **Параметри:** `heading_kp=3.0, heading_ki=0.5, heading_kd=0.2`
- **Обмеження:** max_angular=1.0 rad/s
- **Integral limits:** ±3.0

#### At Position Check:
- Position tolerance: Euclidean distance ≤ 0.5m (default)
- Heading tolerance: < 0.1 rad (~5.7°)

### Area Coverage ([Boustrophedon Pattern](robot_ws/src/perimeter_miner/src/perimeter_loader.cpp:376-445))

Зигзагоподібний патерн покриття області:
- **Scan direction:** 'X' (горизонтальні) або 'Y' (вертикальні проходи)
- **Start corner:** BL (bottom-left), BR, TL, TR
- **Pass spacing:** Відстань між паралельними проходами (2.0m default)
- **Pattern:** Parabolic scan — кожен наступний прохід зміщений на pass_spacing

## Структура файлів

```
course_project/
├── README.md                          # Цей файл
├── http_server.py                     # Демо HTTP сервер для прийому звітів
├── run_test.sh                        # Окремий інтеграційний тест
├── run_all_tests.sh                   # Пакетний запуск всіх тестів
├── .gitignore                         # Ігнорування build artifacts
├── robot_ws/
│   ├── src/
│   │   ├── perimeter_msgs/            # Спільні повідомлення та сервіси
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   ├── msg/                   # .msg файли
│   │   │   │   ├── PerimeterStatus.msg
│   │   │   │   ├── MineDetection.msg
│   │   │   │   ├── ClearanceReport.msg
│   │   │   │   └── MissionSummary.msg
│   │   │   └── srv/                   # .srv файли
│   │   │       ├── SwitchMode.srv
│   │   │       └── TriggerClearance.srv
│   │   ├── perimeter_miner/           # Основний пакет контролера
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   ├── LICENSE
│   │   │   ├── include/perimeter_miner/
│   │   │   │   ├── perimeter_config.hpp      # Waypoint, PerimeterConfig, CoverageConfig, RobotState, MoveCommand
│   │   │   │   ├── perimeter_tracker.hpp     # LateralPID, PurePursuit, PerimeterTracker
│   │   │   │   ├── mode_switch.hpp           # ModeSwitch class
│   │   │   │   ├── hold_controller.hpp       # HoldController class
│   │   │   │   └── perimeter_loader.hpp      # PerimeterLoader (YAML parser)
│   │   │   ├── src/
│   │   │   │   ├── miner_node.cpp            # Main MinerNode (ROS 2 node)
│   │   │   │   ├── mode_switch_node.cpp      # ModeSwitchNode (service handler)
│   │   │   │   ├── lateral_pid.cpp           # LateralPID, PurePursuit, PerimeterTracker impl
│   │   │   │   ├── hold_controller.cpp       # HoldController impl
│   │   │   │   ├── mode_switch.cpp           # ModeSwitch impl
│   │   │   │   └── perimeter_loader.cpp      # YAML parser + boustrophedon generator
│   │   │   ├── launch/
│   │   │   │   └── system.launch.py          # Main system launcher
│   │   │   ├── config/
│   │   │   │   ├── training_ground.yaml           # Square closed perimeter (4 WP)
│   │   │   │   ├── patrol_alpha.yaml              # Open linear patrol (4 WP)
│   │   │   │   ├── large_patrol.yaml              # Large closed perimeter (15 WP)
│   │   │   │   └── training_ground_coverage.yaml  # Coverage mode config
│   │   │   └── test/
│   │   │       ├── hold_controller_pid_test.cpp   # 15 тестів HoldController
│   │   │       ├── mode_switch_test.cpp           # 8 тестів ModeSwitch
│   │   │       ├── perimeter_tracker_test.cpp     # 15 тестів tracker/mode/hold
│   │   │       ├── perimeter_loader_test.cpp      # Тести YAML loader
│   │   │       ├── perimeter_tracker_decide_test.cpp # Тести decide() logic
│   │   │       └── integration_test.cpp           # Інтеграційні тести
│   │   ├── mine_simulator/            # Симуляція мін
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   ├── include/mine_simulator/
│   │   │   │   └── mine_config.hpp      # MineDefinition, MineSimConfig
│   │   │   ├── src/
│   │   │   │   ├── mine_spawner_node.cpp  # MineSpawnerNode (ROS 2 node)
│   │   │   │   └── fake_odom_publisher.cpp # Fake odometry generator
│   │   │   ├── launch/
│   │   │   │   ├── mine_spawner.launch.py
│   │   │   │   ├── fake_odom_publisher.launch.py
│   │   │   │   └── gazebo_simulation.launch.py
│   │   │   ├── config/
│   │   │   │   └── training_ground_mines.yaml
│   │   │   ├── sdf/
│   │   │   │   └── perimeter_mining_ground.sdf
│   │   │   ├── urdf/
│   │   │   │   └── perimeter_miner.urdf
│   │   │   └── test/
│   │   │       └── mine_spawner_test.cpp  # 17 тестів MineDefinition/config
│   │   └── http_reporter/         # HTTP звітність через API
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       ├── include/http_reporter/
│   │       │   └── http_reporter.hpp  # HttpReporter class (curl-based)
│   │       ├── src/
│   │       │   ├── reporter_node.cpp    # ReporterNode (ROS 2 node)
│   │       │   └── http_reporter.cpp    # HTTP client impl
│   │       ├── launch/
│   │       │   └── reporter.launch.py
│   │       └── test/
│   │           ├── http_reporter_test.cpp         # Тести HTTP client
│   │           └── http_reporter_serialization_test.cpp  # JSON сериалізація
│   └── teleop_operator/       # Телеоперація
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/teleop_operator/
│       │   └── teleop_driver.hpp
│       ├── src/
│       │   ├── teleop_node.cpp
│       │   └── teleop_driver.cpp
│       └── launch/
│           └── teleop.launch.py
├── docs/                      # Документація
│   ├── architecture.md        # Архітектура системи
│   ├── startup.md             # Посібник запуску
│   └── test_analysis.md       # Аналіз тестів
├── install/                   # Colcon інсталяція (build artifacts)
├── log/                       # Colcon логи
└── bags/                      # Rosbag записи (генерається під час тестів)
```

## Критерії успіху

1. ✅ Робот повністю обходить замкнений периметр без втрати треку
2. ✅ Бічне відхилення < 0.5 м на прямих ділянках (з PID `kp=2.0, ki=0.5, kd=0.3`)
3. ✅ Mode switch працює коректно (autonomous ↔ teleop ↔ hold)
4. ✅ Оператор може примусово перехопити керування (operator override)
5. ✅ HTTP звіти надходять на сервер
6. ✅ Rosbag записує всі топіки
7. ✅ Mine simulator детектує міни з заданою ймовірністю
8. ✅ Area coverage mode генерує boustrophedon патерн

## Деталі реалізації

### PerimeterLoader ([`perimeter_loader.cpp`](robot_ws/src/perimeter_miner/src/perimeter_loader.cpp))
- **Custom YAML parser** без зовнішніх залежностей (yaml-cpp)
- Парсинг `waypoints:` секції з inline maps `{x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}`
- Підтримка `type: coverage` конфігурацій
- **Boustrophedon generator** ([`generateBoustrophedonPattern()`](robot_ws/src/perimeter_miner/src/perimeter_loader.cpp:376)):
  - Горизонтальні/вертикальні проходи з alternating direction
  - Крок: `pass_spacing` (2.0m default)
  - Кількість проходів: `ceil(area_dimension / pass_spacing) + 1`

### PerimeterTracker ([`lateral_pid.cpp`](robot_ws/src/perimeter_miner/src/lateral_pid.cpp))
- **Lateral error computation:** Cross product `rx * dy - ry * dx` від вектора сегмента до робота
- **Desired heading:** `atan2(target_y - robot_y, target_x - robot_x)` (до цільового waypoint)
- **Linear speed factor:** `clamp(1.0 - dist/(tolerance*3), 0.3, 1.0)` — сповільнення біля waypoint
- **Angular command:** `heading_error * 3.0 + steering * 0.5`, clamped to ±1.5 rad/s
- **Waypoint initialization guard:** `has_initialized_` запобігає premature advance при старті

### Mode Switch ([`mode_switch.cpp`](robot_ws/src/perimeter_miner/src/mode_switch.cpp))
- **Service → Topic pattern:** `mode_switch_node` обробляє `/control/switch_mode` service, публікує в `/control/status` topic
- `miner_node` підписаний на `/control/status` для отримання змін режиму
- **setMode()** bypasses validation (для internal topic-based sync)
- **requestMode() + applyRequest()** з safety checks

### Mine Spawner ([`mine_spawner_node.cpp`](robot_ws/src/mine_simulator/src/mine_spawner_node.cpp))
- **Detection probability:** `prob = base_prob * (1.0 - dist/detection_range * decay)`
- **Default params:** range=3.0m, prob=0.95, decay=0.5, update=100ms
- Слухає `/robot/position` у форматі `"x,y"`

### Hold Controller ([`hold_controller.cpp`](robot_ws/src/perimeter_miner/src/hold_controller.cpp))
- **Separate PID** для position (x, y) та heading
- **Anti-windup:** Integral clamping (`±5.0` position, `±3.0` heading)
- **At position check:** Euclidean distance ≤ tolerance AND heading error < 0.1 rad

### Test Coverage ([test/](robot_ws/src/perimeter_miner/test/))

| Тест | Файл | Кількість | Що тестується |
|------|------|-----------|---------------|
| HoldControllerPIDTest | [`hold_controller_pid_test.cpp`](robot_ws/src/perimeter_miner/test/hold_controller_pid_test.cpp) | 15 | PID compute, clamping, heading normalization, tolerance |
| ModeSwitchTest | [`mode_switch_test.cpp`](robot_ws/src/perimeter_miner/test/mode_switch_test.cpp) | 8 | Transitions, override, safety checks, messages |
| PerimeterTrackerTest | [`perimeter_tracker_test.cpp`](robot_ws/src/perimeter_miner/test/perimeter_tracker_test.cpp) | 15 | Init, waypoint reach, closed loop, PID, status |
| MineDefinitionTest | [`mine_spawner_test.cpp`](robot_ws/src/mine_simulator/test/mine_spawner_test.cpp) | 17 | Default values, state transitions, distance, probability |

**Загалом:** ~55+ unit тестів з покриттям ключових компонентів.

## Залежності

### Системні
- ROS 2 Jazzy (або Humble)
- CMake 3.8+
- GCC 11+ / Clang 14+
- libcurl (для HTTP reporter)

### ROS пакети
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `builtin_interfaces`
- `rosidl_default_runtime`
- `nlohmann-json` (опціонально, для HTTP reporter)

### Тестування
- `ament_cmake_gtest`
- `yaml_cpp_vendor` (не використовується — custom parser)

## Примітки реалізації

1. **YAML parsing:** Переміщено з yaml-cpp на кастомний парсер для зменшення залежностей ([`perimeter_loader.cpp`](robot_ws/src/perimeter_miner/src/perimeter_loader.cpp))
2. **Mode switch architecture:** Сервіс обробляється окремою нодою (`mode_switch_node`) для чіткого розділення відповідальності
3. **Real dt tracking:** PID контролери використовують реальний time step з `control_timer_` (50Hz) замість фіксованого
4. **Diagnostic logging:** Діагностичні виводи через `fprintf(stderr, ...)` для швидкого дебагування
5. **Coverage mode:** Підтримка boustrophedon патерну через `enable_coverage:=true` або `type: coverage` в YAML

## HTTP Server (Demo)

Демо сервер для прийому звітів: [`http_server.py`](http_server.py)

```bash
python3 http_server.py
# Waiting on port 8080...

# Test with curl:
curl -X POST http://localhost:8080/api/v1/movement \
  -H "Content-Type: application/json" \
  -d '{"mode": 0, "waypoint_index": 0, "target_x": 20.0, "target_y": 0.0, "current_x": 5.0, "current_y": 0.0, "lateral_error": 0.0, "speed": 2.0, "mine_detected": false, "timestamp": "1700000000.0"}'
```

## Ліцензія

MIT License

## Автор

Студентський проєкт для курсової роботи з робототехніки
