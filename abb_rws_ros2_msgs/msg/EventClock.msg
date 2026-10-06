# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# How a controller's event times are corrected (added in schema 0.10).

# The daemon's UTC time minus the controller clock's reading, in ms, as last measured: the
# controller's zone and its clock's error together. Events are corrected with it. null: never
# measured, and event times are not corrected.
# Unit: ms.
# Valid only when offset_ms_known is true.
int64 offset_ms
bool offset_ms_known

# When `offset_ms` was measured (UTC); null when never.
# Valid only when measured_at_known is true.
builtin_interfaces/Time measured_at
bool measured_at_known

# Why the clock was not measured the last time it was tried - the client cannot read it, the read
# failed, or the reading was no date and time; null when the last try succeeded. With an older
# `offset_ms` still given, that one is used meanwhile.
# Valid only when problem_known is true.
string problem
bool problem_known
