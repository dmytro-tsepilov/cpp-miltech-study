# 01_msg_array

Приклад показує, як у C++ працювати з масивом, оголошеним у `.msg`.

У ДЗ 14 `LocalScan.msg` містить:

```text
CellObservation[] cells
```

У цьому demo використовується така сама форма:

```text
CellObservationLite[] cells
```

У згенерованому C++ типі це звичайний контейнер, з яким можна працювати як з
`std::vector`: `reserve(...)`, `push_back(...)`, range-for.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_6/01_msg_array/robot_ws
colcon build --symlink-install --packages-select msg_array_demo
source install/setup.bash
```

## Run

Термінал 1:

```bash
ros2 run msg_array_demo scan_publisher
```

Термінал 2:

```bash
ros2 run msg_array_demo scan_subscriber
```

Перевірка через CLI:

```bash
ros2 topic info /demo/local_scan
ros2 topic echo /demo/local_scan --once
```

Очікуваний тип:

```text
msg_array_demo/msg/LocalScanLite
```

## Файли

- `msg/CellObservationLite.msg` - один елемент локального огляду;
- `msg/LocalScanLite.msg` - повідомлення з масивом `CellObservationLite[]`;
- `src/scan_publisher.cpp` - створення масиву через `push_back`;
- `src/scan_subscriber.cpp` - обхід масиву через `for (const auto& cell : ...)`;
- `CMakeLists.txt` - генерація двох `.msg` і підключення typesupport.

