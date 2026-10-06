# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# RAPID execution (slice 3). Execution state and cycle are pushed by the controller; tasks and
# modules ride the client's slow poll; the pointers are read by the daemon (every second while a
# task runs, otherwise every few seconds, and at once when execution changes).

# Empty when not known.
string exec_state
string EXEC_STATE_RUNNING="running"
string EXEC_STATE_STOPPED="stopped"

# once | forever | asis | oncedone, as reported.
# Valid only when cycle_known is true.
string cycle
bool cycle_known

RapidTaskInfo[] tasks

# Same meaning as `panel.as_of`.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
