# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# RMMP, the privilege a remote client needs before it may write in a manual mode on RobotWare 6 and
# 7. Added in schema 0.13.

# Whether this controller offers RMMP, as the client and the controller report it: true once a
# privilege was read on this connection; false on RobotWare 8 (nothing is sent) and when the
# controller answered that it does not offer the service; null until one of them is known.
int8 supported
int8 SUPPORTED_UNKNOWN=-1
int8 SUPPORTED_FALSE=0
int8 SUPPORTED_TRUE=1

# The privilege of the daemon's own session as the controller reported it at `as_of`. The daemon
# never requests it. null where RMMP is not offered and until a privilege was read and understood
# on this connection.
# Empty when not known.
string privilege
string PRIVILEGE_NONE="none"
string PRIVILEGE_PENDING_MODIFY="pending modify"
string PRIVILEGE_PENDING_EXEC="pending exec"
string PRIVILEGE_MODIFY="modify"
string PRIVILEGE_EXEC="exec"

# When the privilege was last read and understood on this connection; null until then and where
# RMMP is not offered.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known

# The last read failed or was not understood; null once one succeeds.
# Empty or one element.
GroupError[<=1] error
