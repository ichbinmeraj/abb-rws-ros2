# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One group's freshness. Added in schema 0.13.

string group
string GROUP_IDENTITY="identity"
string GROUP_PANEL="panel"
string GROUP_RAPID="rapid"
string GROUP_MOTION="motion"
string GROUP_IO="io"
string GROUP_EVENTS="events"
string GROUP_ACCESS="access"

string state
string STATE_CURRENT="current"
string STATE_PARTIAL="partial"
string STATE_STALE="stale"
string STATE_IDLE="idle"
string STATE_UNREAD="unread"
string STATE_UNAVAILABLE="unavailable"
string STATE_WITHHELD="withheld"

# That group's own `as_of` (null when it has none yet).
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known

# Why, in plain words, when the state is not `current`; null when it is. Never holds a time or a
# counter.
# Valid only when reason_known is true.
string reason
bool reason_known
