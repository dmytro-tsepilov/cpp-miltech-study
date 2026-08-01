# Демо для заняття 5.5

Маленькі приклади для розмови про стек передачі ROS 2, DDS/RMW, великі
payload-и, розгалуження читачів і shared memory.

## Приклади

```text
demos/lesson_5_5/
  01_iceoryx_vs_cyclonedds/
    robot_ws/src/iceoryx_compare_demo/
  02_qos_profiles/
    robot_ws/src/qos_profiles_demo/
```

## Ідея

`01_iceoryx_vs_cyclonedds` показує два рівні:

- той самий ROS 2 publisher/subscriber код з `borrow_loaned_message()`;
- `rmw_cyclonedds_cpp` без SharedMemory vs `rmw_cyclonedds_cpp` з
  CycloneDDS/iceoryx SharedMemory через `CYCLONEDDS_URI`.

Мета - не рейтинг DDS vendor-ів. Мета - показати, що той самий код може
відправити 1 MB, 10 MB і 100 MB fixed-size payload через звичайний шлях
CycloneDDS або shared-memory шлях, а прикладний ROS 2 код не має
прив'язуватися до конкретного transport.

`02_qos_profiles` показує QoS як контракт даних:

- несумісність reliability;
- transient local для late joiner;
- KeepLast depth під тиском slow subscriber.
