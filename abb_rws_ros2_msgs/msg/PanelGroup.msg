# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Controller status. Values as the controller reports them; known values: ctrlState init | motoroff
# | motoron | guardstop | emergencystop | emergencystopreset | sysfail opMode AUTO | MANR (manual
# reduced speed) | MANF (manual full speed) collisionDetection INIT | TRIGGERED | CONFIRMED |
# TRIGGERED_ACK

# Valid only when ctrl_state_known is true.
string ctrl_state
bool ctrl_state_known

# Valid only when op_mode_known is true.
string op_mode
bool op_mode_known

# Percent, 0-100.
# Unit: percent.
# NaN when not known.
float64 speed_ratio

# Valid only when collision_detection_known is true.
string collision_detection
bool collision_detection_known

# When these values were last known to be current. They are pushed by the controller on change, so
# while the link is `live` or `polling` this is "now"; when the link degrades it freezes at the
# last healthy moment. null if the link has never been healthy.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
