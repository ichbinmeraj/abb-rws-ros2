# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

# The daemon holds a session the controller has answered on: true while `quality` is `live`,
# `polling` or `stale`; false while a session is being made or remade (`reconnecting`) and while
# there is none.
bool connected

string quality
string QUALITY_LIVE="live"
string QUALITY_POLLING="polling"
string QUALITY_RECONNECTING="reconnecting"
string QUALITY_STALE="stale"
string QUALITY_DISCONNECTED="disconnected"

string quality_reason

# When the current `quality` began.
builtin_interfaces/Time since

# While the link is not healthy: when it last was (`live` or `polling`), or, for a controller never
# reached, when the daemon began trying. null while the link is healthy. Unlike `since`, it does
# not move with each reconnect attempt. Added in schema 0.13.
# Valid only when down_since_known is true.
builtin_interfaces/Time down_since
bool down_since_known

string host

# The port in use now - follows a virtual controller that re-bound on restart.
int64 port

bool use_https

LinkSubscriptions subscriptions

# Is the controller on this link the one that was added?
IdentityCheck identity_check

# While disconnected: how many reconnects have failed, and when the next one runs.
# Empty or one element.
ReconnectInfo[<=1] reconnect

# Where this link's values come from: `live`, a session the daemon holds now; `replay`, a recording
# a replaying daemon serves, with no controller reached. In replay every other link field
# (`quality`, `connected`, ...) is as recorded, so a consumer labels the link by this field, never
# by `quality` alone. A replay of a recording made before schema 0.13 serves its links as recorded,
# without this field. Added in schema 0.13.
string mode
string MODE_LIVE="live"
string MODE_REPLAY="replay"

# In replay: when the recorded session began (the recording header's `sessionStartedAt`), not moved
# by the replay's time shift. null live. Added in schema 0.13.
# Valid only when recorded_at_known is true.
builtin_interfaces/Time recorded_at
bool recorded_at_known
