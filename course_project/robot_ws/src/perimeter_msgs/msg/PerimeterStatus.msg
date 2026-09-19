# Perimeter patrol status message
uint8 mode                 # 0=AUTONOMOUS, 1=TELEOP, 2=HOLD
uint32 waypoint_index      # Current waypoint index (0-based)
float64 target_x           # Target waypoint X position (meters)
float64 target_y           # Target waypoint Y position (meters)
float64 current_x          # Current robot X position (meters)
float64 current_y          # Current robot Y position (meters)
float64 lateral_error      # Lateral deviation from perimeter path (meters)
float32 speed              # Current linear speed (m/s)
bool mine_detected         # Whether a mine is currently detected
builtin_interfaces/Time timestamp  # Status timestamp
