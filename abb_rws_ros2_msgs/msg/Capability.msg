# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One row of a controller's capability table. Added in schema 0.13.

string id
string ID_STATUS_PUSH="status-push"
string ID_EVENTS_PUSH="events-push"
string ID_EVENT_TIMES="event-times"
string ID_IO_PUSH="io-push"
string ID_IO_TABLE="io-table"
string ID_JOINT_LIMITS="joint-limits"
string ID_MASTERSHIP="mastership"
string ID_WRITE_ACCESS="write-access"
string ID_RMMP="rmmp"

string state
string STATE_AVAILABLE="available"
string STATE_UNAVAILABLE="unavailable"
string STATE_DEGRADED="degraded"
string STATE_UNKNOWN="unknown"

# Why, in plain words, when the state is not `available`; null when it is.
# Valid only when reason_known is true.
string reason
bool reason_known
