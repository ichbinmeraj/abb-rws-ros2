# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Who holds a lock, as far as the controller says. `local`: a local client, normally the
# FlexPendant (the controller only says `local`). `remote`: a remote client. `internal`: the
# controller itself. `station`: a control station (RobotWare 8). A holder is named, never
# identified: no user id and no control-station id is served, logged or recorded. Added in schema
# 0.13.

string kind
string KIND_LOCAL="local"
string KIND_REMOTE="remote"
string KIND_INTERNAL="internal"
string KIND_STATION="station"

# RobotWare 8: the holding control station's name, as the controller reports it; null when it
# reports none, and on RobotWare 6 and 7.
# Valid only when name_known is true.
string name
bool name_known

# RobotWare 6 and 7: the application name the controller reports for the holder; null when it
# reports none, and on RobotWare 8.
# Valid only when application_known is true.
string application
bool application_known

# Whether the holder is the FlexPendant, by the name the controller gives it, in the two forms
# seen: true for a `local` holder whose application is `TPU` (RobotWare 7's name for the
# FlexPendant's own session) and for a `station` named `FlexPendant` (RobotWare 8), in any case;
# false for a holder of another name (a station named `TPU` too) and for a `remote` or `internal`
# holder; null when the controller names no application or station. A consumer shows a holder with
# true as the FlexPendant, whichever name the controller used. Added in schema 0.14.
int8 flex_pendant
int8 FLEX_PENDANT_UNKNOWN=-1
int8 FLEX_PENDANT_FALSE=0
int8 FLEX_PENDANT_TRUE=1
