# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

# Valid only when controller_name_known is true.
string controller_name
bool controller_name_known

# Valid only when controller_id_known is true.
string controller_id
bool controller_id_known

# As reported; the generations spell it differently ("Virtual Controller" / "VIRTUAL_CONTROLLER").
# Valid only when controller_type_known is true.
string controller_type
bool controller_type_known

# Derived from controllerType; null when it is unrecognised.
int8 is_virtual
int8 IS_VIRTUAL_UNKNOWN=-1
int8 IS_VIRTUAL_FALSE=0
int8 IS_VIRTUAL_TRUE=1

# Valid only when mac_known is true.
string mac
bool mac_known

# Valid only when system_name_known is true.
string system_name
bool system_name_known

# Valid only when sys_id_known is true.
string sys_id
bool sys_id_known

# As reported - not ISO 8601 on any generation seen so far. It is the controller's local time, with
# no zone (live-verified 2026-09-29 on RW 6.16.03, 7.21.4+417 and 8.2.1+1023). Not a restart
# marker: an RW 6.16.03 virtual controller (2026-09-29) reported a fixed start time for its first
# hours, then the request time on every read, with no restart in between.
# Valid only when start_time_known is true.
string start_time
bool start_time_known

RobotWareVersion robot_ware

string[] options

# When the controller last actually re-read this group (it rides a slow poll).
builtin_interfaces/Time as_of
