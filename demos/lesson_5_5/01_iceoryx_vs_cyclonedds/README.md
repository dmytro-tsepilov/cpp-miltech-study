# ROS 2 loaned messages + CycloneDDS/iceoryx

Демо для заняття 5.5: показати, що прикладний ROS 2 C++ код лишається тим
самим, а різниця між звичайним CycloneDDS і shared-memory шляхом вмикається
через конфігурацію CycloneDDS.

```text
ros2_payload_publisher -> rmw_cyclonedds_cpp -> ros2_payload_subscriber

publisher API:
  borrow_loaned_message()
  publish(std::move(loaned_message))

перемикач transport:
  CYCLONEDDS_URI=config/cyclonedds_no_shm.xml
  CYCLONEDDS_URI=config/cyclonedds_iceoryx.xml + iox-roudi
```

У demo немає окремого iceoryx publisher/subscriber. iceoryx працює під капотом
CycloneDDS SharedMemory.

## Що показує demo

Publisher і subscriber навмисно прості:

- три topic-и:
  - `/demo/loaned_payload_1mb`;
  - `/demo/loaned_payload_10mb`;
  - `/demo/loaned_payload_100mb`;
- три fixed-size message types: `PayloadOneMb`, `PayloadTenMb`, `PayloadHundredMb`;
- 4 семпли для кожного розміру payload;
- `Reliable`, `Volatile`, `KeepLast(4)`;
- той самий C++ код для обох запусків.

Publisher завжди використовує loaned message API:

```cpp
auto loaned_message = publisher_->borrow_loaned_message();
auto& message = loaned_message.get();
publisher_->publish(std::move(loaned_message));
```

У звичайному CycloneDDS запуску log publisher має показати:

```text
loaned_1mb=false loaned_10mb=false loaned_100mb=false
```

У CycloneDDS + iceoryx SharedMemory запуску очікуваний сигнал:

```text
loaned_1mb=true loaned_10mb=true loaned_100mb=true
```

Publisher друкує `publish_call_ms`: скільки часу займає сам виклик
`publish(...)` після підготовки loaned message.

```text
PUBLISH payload=1MB seq=0 bytes=1048576 publish_call_ms=...
PUBLISH payload=10MB seq=0 bytes=10485760 publish_call_ms=...
PUBLISH payload=100MB seq=0 bytes=104857600 publish_call_ms=...

PUBLISH_RESULT transport=cyclonedds-iceoryx payload=1MB samples=4 payload_bytes=1048576 avg_publish_call_ms=... max_publish_call_ms=...
PUBLISH_RESULT transport=cyclonedds-iceoryx payload=10MB samples=4 payload_bytes=10485760 avg_publish_call_ms=... max_publish_call_ms=...
PUBLISH_RESULT transport=cyclonedds-iceoryx payload=100MB samples=4 payload_bytes=104857600 avg_publish_call_ms=... max_publish_call_ms=...
```

Перед shutdown publisher очікує acknowledgments для всіх трьох Reliable
topic-ів. Це не дає останньому 100 MB семплу загубитися під час завершення
процесу:

```text
reliable_delivery acked_1mb=true acked_10mb=true acked_100mb=true timeout_ms=10000
```

Subscriber друкує окремі callback samples і summary для кожного розміру
payload:

```text
SAMPLE transport=cyclonedds-iceoryx payload=1MB seq=0 payload_bytes=1048576 callback_age_ms=...
SAMPLE transport=cyclonedds-iceoryx payload=10MB seq=0 payload_bytes=10485760 callback_age_ms=...
SAMPLE transport=cyclonedds-iceoryx payload=100MB seq=0 payload_bytes=104857600 callback_age_ms=...

CALLBACK_RESULT transport=cyclonedds-iceoryx payload=1MB samples=4 payload_bytes=1048576 avg_callback_age_ms=... max_callback_age_ms=...
CALLBACK_RESULT transport=cyclonedds-iceoryx payload=10MB samples=4 payload_bytes=10485760 avg_callback_age_ms=... max_callback_age_ms=...
CALLBACK_RESULT transport=cyclonedds-iceoryx payload=100MB samples=4 payload_bytes=104857600 avg_callback_age_ms=... max_callback_age_ms=...
```

Це навчальне demo, а не матеріал для публікації результатів. Очікувана форма:
без SharedMemory `publish_call_ms` помітніше росте з розміром payload, а з
CycloneDDS/iceoryx SharedMemory лишається значно пласкішим.
`callback_age_ms` показує повний шлях до callback і може включати executor
scheduling або subscriber-side обробку.

## Залежності

Devcontainer має містити:

```text
ros-jazzy-rmw-cyclonedds-cpp
ros-jazzy-iceoryx-posh
ros-jazzy-iceoryx-introspection
ros-jazzy-rosidl-default-generators
```

`ros-jazzy-iceoryx-posh` потрібен для `iox-roudi`.
`ros-jazzy-iceoryx-introspection` потрібен для сумісного
`iox-introspection-client`. Прикладний C++ код напряму з iceoryx не лінкується.

Важливо: не використовувати Ubuntu package `iceoryx` і його
`/usr/bin/iox-introspection-client` разом з ROS Jazzy RouDi. Це інша збірка
iceoryx. Має використовуватись ROS binary з `/opt/ros/jazzy/bin`:

```bash
which iox-roudi
which iox-introspection-client
```

Очікувано:

```text
/opt/ros/jazzy/bin/iox-roudi
/opt/ros/jazzy/bin/iox-introspection-client
```

Якщо client взятий з Ubuntu package, може бути помилка:

```text
Version mismatch from 'introspection'
```

Для shared memory перевірити budget:

```bash
df -h /dev/shm
```

У devcontainer цього репо додано `--shm-size=2g`; поточний RouDi config має
пули для 1 MB, 10 MB і 100 MB семплів.

Для візуального моніторингу під час demo можна відкрити окремий термінал:

```bash
btop
```

У `btop` зручно дивитися CPU/RAM і процеси `iox-roudi`,
`ros2_payload_publisher`, `ros2_payload_subscriber`. Сам факт використання
shared memory краще підтверджувати через `loaned_*=true`, `PUBLISH_RESULT` і
`df -h /dev/shm`, бо `btop` не показує ownership окремих iceoryx chunks.

Для окремої перевірки iceoryx introspection можна відкрити ще один термінал
після старту RouDi:

```bash
iox-introspection-client --all
```

## Збірка

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
colcon build --symlink-install --packages-select iceoryx_compare_demo
source install/setup.bash
```

## CycloneDDS без SharedMemory

Термінал A:

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
source install/setup.bash

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=file://$(pwd)/src/iceoryx_compare_demo/config/cyclonedds_no_shm.xml \
DEMO_TRANSPORT_LABEL=cyclonedds-no-shm \
ros2 run iceoryx_compare_demo ros2_payload_subscriber
```

Термінал B:

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
source install/setup.bash

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=file://$(pwd)/src/iceoryx_compare_demo/config/cyclonedds_no_shm.xml \
DEMO_TRANSPORT_LABEL=cyclonedds-no-shm \
ros2 run iceoryx_compare_demo ros2_payload_publisher
```

## CycloneDDS + iceoryx SharedMemory

Термінал A:

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
source install/setup.bash

iox-roudi --log-level warning -c src/iceoryx_compare_demo/config/roudi_100mb.toml
```

Термінал B:

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
source install/setup.bash

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=file://$(pwd)/src/iceoryx_compare_demo/config/cyclonedds_iceoryx.xml \
DEMO_TRANSPORT_LABEL=cyclonedds-iceoryx \
ros2 run iceoryx_compare_demo ros2_payload_subscriber
```

Термінал C:

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/01_iceoryx_vs_cyclonedds/robot_ws
source install/setup.bash

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=file://$(pwd)/src/iceoryx_compare_demo/config/cyclonedds_iceoryx.xml \
DEMO_TRANSPORT_LABEL=cyclonedds-iceoryx \
ros2 run iceoryx_compare_demo ros2_payload_publisher
```

Publisher/subscriber commands лишаються тими самими. Міняється тільки
`CYCLONEDDS_URI` і наявність RouDi.
