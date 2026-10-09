# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One lock of a controller, as its own status reports it. `held`, `holder` and `as_of` always
# belong to the same reading: the last one the daemon read and understood on this connection. An
# answer that was empty, of an unknown form or not understood is a failed read (`error`), never
# "nobody holds it". Added in schema 0.13.

string lock
string LOCK_CFG="cfg"
string LOCK_MOTION="motion"
string LOCK_RAPID="rapid"
string LOCK_EDIT="edit"
string LOCK_WRITEACCESS="writeaccess"

# Someone holds the lock (true) or nobody does (false), as the controller said at `as_of`; null
# while no status was read and understood on this connection. A reading, never a promise: it can
# change the moment after it was read.
int8 held
int8 HELD_UNKNOWN=-1
int8 HELD_FALSE=0
int8 HELD_TRUE=1

# Who holds it; null when nobody does, when the controller does not say, or when `held` is null.
# Empty or one element.
AccessHolder[<=1] holder

# Whether the daemon's own session holds the lock, from the client's own record of what this
# session requested, not from the controller's status. A daemon without write routes never requests
# a lock, so this is false; null when the client is not sure.
int8 held_by_daemon
int8 HELD_BY_DAEMON_UNKNOWN=-1
int8 HELD_BY_DAEMON_FALSE=0
int8 HELD_BY_DAEMON_TRUE=1

# RobotWare 6 and 7: the status word of the most recent answer that carried one, as the client
# handed it over, for example `nomaster`, `local`, `remote` or `internal`. While `error` says the
# status was not understood, this is the word that was not understood, or, when that answer carried
# no word at all, the word of the answer before it; `held` and `holder` are still those of the last
# good read. null on RobotWare 8 and until an answer carried a word.
# Valid only when reported_known is true.
string reported
bool reported_known

# When this lock's status was last read and understood on this connection; null until then. Kept,
# not advanced, while reads fail or the link is down.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known

# The last read of this lock's status failed or was not understood; `held`, `holder` and `as_of`
# keep what the last good read gave. null once a read succeeds.
# Empty or one element.
GroupError[<=1] error
