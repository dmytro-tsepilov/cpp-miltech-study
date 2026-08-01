# ROS 2 QoS profiles demo

Демо для заняття 5.5: швидко показати, що QoS у ROS 2 - це контракт даних між
publisher і subscriber, а не просто "налаштування на всяк випадок".

```text
qos_counter_publisher  ->  /qos_demo/counter  ->  qos_counter_subscriber
```

Один і той самий код запускається з різними QoS параметрами:

- `reliability`: `reliable` або `best_effort`;
- `durability`: `volatile` або `transient_local`;
- `depth`: розмір `KeepLast` queue.

## Збірка

```bash
source /opt/ros/jazzy/setup.bash
cd demos/lesson_5_5/02_qos_profiles/robot_ws
colcon build --symlink-install --packages-select qos_profiles_demo
source install/setup.bash
```

## Базовий запуск

Термінал A:

```bash
ros2 run qos_profiles_demo qos_counter_subscriber --ros-args \
  -p reliability:=best_effort \
  -p durability:=volatile \
  -p depth:=4 \
  -p max_idle_ms:=3000
```

Термінал B:

```bash
ros2 run qos_profiles_demo qos_counter_publisher --ros-args \
  -p reliability:=best_effort \
  -p durability:=volatile \
  -p depth:=4 \
  -p period_ms:=100 \
  -p stop_after:=20
```

Підсумок має такий формат:

```text
SUMMARY role=subscriber received=20 first_seq=0 last_seq=19 gaps=0
```

## Несумісність reliability

Несумісність reliability: subscriber просить `reliable`, publisher пропонує
`best_effort`. Очікування: messages не проходять, subscriber завершує вимір за
idle-timeout.

Термінал A:

```bash
ros2 run qos_profiles_demo qos_counter_subscriber --ros-args \
  -p reliability:=reliable \
  -p durability:=volatile \
  -p depth:=4 \
  -p max_idle_ms:=3000
```

Термінал B:

```bash
ros2 run qos_profiles_demo qos_counter_publisher --ros-args \
  -p reliability:=best_effort \
  -p durability:=volatile \
  -p depth:=4 \
  -p period_ms:=100 \
  -p stop_after:=10
```

Очікуваний сигнал:

```text
SUMMARY role=subscriber received=0 first_seq=0 last_seq=0 gaps=0
```

## Transient local для late joiner

Transient local late joiner: publisher зберігає останній sample, subscriber
стартує пізніше і все одно отримує останнє значення.

Термінал A:

```bash
ros2 run qos_profiles_demo qos_counter_publisher --ros-args \
  -p reliability:=reliable \
  -p durability:=transient_local \
  -p depth:=1 \
  -p period_ms:=300 \
  -p stop_after:=3 \
  -p linger_ms:=8000
```

Після публікації кількох samples запустити subscriber.

Термінал B:

```bash
ros2 run qos_profiles_demo qos_counter_subscriber --ros-args \
  -p reliability:=reliable \
  -p durability:=transient_local \
  -p depth:=1 \
  -p expected_samples:=1 \
  -p max_idle_ms:=3000
```

Очікуваний сигнал: subscriber отримує останній збережений sample, хоча
підключився після старту publisher.

## Тиск на depth

Depth pressure: publisher генерує samples швидше, ніж slow subscriber встигає
обробляти callback. Очікування: у summary видно gaps.

Термінал A:

```bash
ros2 run qos_profiles_demo qos_counter_subscriber --ros-args \
  -p reliability:=best_effort \
  -p durability:=volatile \
  -p depth:=1 \
  -p processing_ms:=100 \
  -p max_idle_ms:=3000
```

Термінал B:

```bash
ros2 run qos_profiles_demo qos_counter_publisher --ros-args \
  -p reliability:=best_effort \
  -p durability:=volatile \
  -p depth:=1 \
  -p period_ms:=5 \
  -p stop_after:=200
```

Очікуваний сигнал:

```text
SUMMARY role=subscriber received=... first_seq=... last_seq=... gaps=...
```

`gaps > 0` означає, що частина samples була витіснена або пропущена через
малий queue depth і повільну обробку callback.
