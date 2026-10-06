# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One message of a controller's event log (added in schema 0.10), from its common log (domain 0).

# The daemon's number for this event on this controller: 1 for the first it recorded, one more for
# each after, in the controller's order. It orders events and is the cursor of `GET
# .../events?since=`. It starts again at 1 when the daemon starts or the controller is added again
# - `feed` changes then too.
int64 seq

# The controller's own sequence number, as reported; null when it reported none. RWS 1.0 counts it
# in 16 bits, so after 65,535 it starts again: order by `seq`, not by this.
# Valid only when seqnum_known is true.
int64 seqnum
bool seqnum_known

# The event code, e.g. 10205 (null when none was reported).
# Valid only when code_known is true.
int64 code
bool code_known

# From the controller's message type: 1 info, 2 warning, 3 error; null for any other.
# Empty when not known.
string severity
string SEVERITY_INFO="info"
string SEVERITY_WARNING="warning"
string SEVERITY_ERROR="error"

# The controller's title. A controller whose message text is missing (a language not installed, for
# one) still reports the code: the title is then "Event <code>" and `title_from_code` says so.
string title

# The controller sent no title; `title` was made from `code`.
bool title_from_code

# The message's parts, as the controller wrote them; each null when it sent none. At most
# EVENT_TEXT_MAX characters each.
# Valid only when description_known is true.
string description
bool description_known

# Valid only when causes_known is true.
string causes
bool causes_known

# Valid only when consequences_known is true.
string consequences
bool consequences_known

# Valid only when actions_known is true.
string actions
bool actions_known

# The values the controller substitutes into the message, in order, each with the type it declares
# ('long', 'string', ...). Empty when none; at most 16, each value at most 256 characters.
EventArg[] args

# The subsystem that raised it (e.g. MC0); null when not reported.
# Valid only when source_known is true.
string source
bool source_known

EventTime time

# When the daemon received it (UTC).
builtin_interfaces/Time received_at

# The message was already in the log when the daemon first read it (at the daemon's start, or when
# the controller was added): `received_at` is when the daemon read it, not when it happened. A
# consumer that alerts on new events skips these.
bool backlog
