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
