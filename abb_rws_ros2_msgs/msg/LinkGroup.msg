# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

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
