# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Access (added in schema 0.13): what the controller itself reports about who may write to it. The
# daemon reads this status every ACCESS_READ_MS (ACCESS_READ_SLOW_MS while the link is `polling`),
# at once on a new connection, and sooner than booked when a link recovers or the operating mode
# changes, though on one connection never twice within ACCESS_READ_MS. What was read on an earlier
# connection is cleared when the link is usable again, so every value here was read on the present
# connection or, while the link is down, on the last one. The daemon only reads: it never requests,
# releases or changes mastership, write access, RMMP or a control-station registration. Nothing
# here says that a write would succeed: the controller decides every write when it arrives.

# The generation as the client determined it from what the controller reported when it connected;
# null until then, and for the whole connection when the controller reported no RobotWare version
# the client could use.
# Empty when not known.
string generation
string GENERATION_RW6="rw6"
string GENERATION_RW7="rw7"
string GENERATION_RW8="rw8"

# Empty when not known.
string model
string MODEL_MASTERSHIP="mastership"
string MODEL_WRITEACCESS="writeaccess"

# One entry per lock of the generation (RobotWare 6: cfg, motion, rapid; RobotWare 7: edit, motion;
# RobotWare 8: writeaccess); empty until the generation is known.
AccessLockInfo[<=3] locks

# RobotWare 8: the controller's own "external control enabled" flag (remote access, switched at the
# FlexPendant), as its write-access status reported it with the last good read of the lock. null on
# RobotWare 6 and 7, until that read, and when the status did not carry the flag.
int8 remote_access_enabled
int8 REMOTE_ACCESS_ENABLED_UNKNOWN=-1
int8 REMOTE_ACCESS_ENABLED_FALSE=0
int8 REMOTE_ACCESS_ENABLED_TRUE=1

RmmpInfo rmmp

# `poll` while the daemon is reading at least one part of the status on this link; null while it
# reads none (the link is down, degraded or on the wrong controller, the generation is not known,
# or `problem` is set).
# Empty when not known.
string via
string VIA_POLL="poll"

# Why no part of the status is read on this controller (a status the client cannot read, such as
# RobotWare 8 write access with a client older than 1.4.0; a controller that reported no usable
# RobotWare version); null otherwise.
# Valid only when problem_known is true.
string problem
bool problem_known

# The oldest `as_of` of its parts (each lock; RMMP unless the controller does not offer it): every
# part is at least this current. null while a part has not been read and understood on this
# connection.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
