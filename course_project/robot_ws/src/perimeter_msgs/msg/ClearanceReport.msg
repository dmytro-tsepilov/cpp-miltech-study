# Mine clearance report message
int32 mine_id              # ID of the cleared mine
float64 x                  # Position X (meters)
float64 y                  # Position Y (meters)
string method              # "disposal" | "marking" | "neutralization"
bool success               # Whether clearance was successful
string details             # Additional details about the operation
builtin_interfaces/Time timestamp  # Report timestamp
