# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One watched I/O signal. `name` is the signal's controller-wide name exactly as `listAllSignals` /
# the watch request named it. `value` is raw, as the controller reports it ('1', '0', an analog
# number as text) - the daemon passes values through by design.

string name

# DI | DO | AI | AO | GI | GO as reported; null before the first read.
# Valid only when type_known is true.
string type
bool type_known

# Valid only when value_known is true.
string value
bool value_known

# How this signal's updates arrive: `subscription` (pushed on change, RWS 2.0 with a live stream)
# or `poll` (read on a cadence - RWS 1.0, or any link whose stream is down). null before the watch
# first serves it.
# Empty when not known.
string via
string VIA_SUBSCRIPTION="subscription"
string VIA_POLL="poll"

# When this value was last confirmed current; null before the first read.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known

# The signal's last read/subscribe failed (e.g. no such signal). Value keeps its last good state.
# Empty or one element.
GroupError[<=1] error
