# Mine detection message
int32 mine_id              # Unique identifier of the detected mine
float64 x                  # X position in map frame (meters)
float64 y                  # Y position in map frame (meters)
string type                # "anti-tank" | "anti-personnel" | "unknown"
float32 confidence         # Detection confidence 0.0 - 1.0
builtin_interfaces/Time detected_at  # Time of detection
