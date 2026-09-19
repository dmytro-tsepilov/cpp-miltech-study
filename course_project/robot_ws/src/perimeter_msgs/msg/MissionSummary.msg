# Mission summary report message
string scenario_name       # Name of the mission scenario
string result              # "SUCCESS" | "FAILED" | "ABORTED"
string reason              # Reason for result
uint32 total_waypoints     # Total number of waypoints in perimeter
uint32 waypoints_completed # Number of waypoints completed
uint32 mines_detected      # Number of mines detected
uint32 mines_cleared       # Number of mines cleared
float64 mission_duration   # Mission duration in seconds
float64 coverage_percent   # Perimeter coverage percentage (0-100)
builtin_interfaces/Time start_time   # Mission start time
builtin_interfaces/Time end_time     # Mission end time
