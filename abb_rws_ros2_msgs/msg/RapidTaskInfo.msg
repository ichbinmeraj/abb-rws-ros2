# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

string name

# Task type as reported (NORMAL, STATIC, SEMISTATIC; the case varies by generation).
string type

# The task's own state as reported (e.g. initiated, uninitiated, linked).
# Valid only when task_state_known is true.
string task_state
bool task_state_known

# Empty when not known.
string exec_state
string EXEC_STATE_RUNNING="running"
string EXEC_STATE_STOPPED="stopped"

# Selected in the task panel (only active tasks run).
bool active

# Drives a mechanical unit.
bool motion

# Module names as the controller lists them; empty until read.
string[] modules

# null: no program pointer (no program loaded), or not read yet.
# Empty or one element.
RapidPointer[<=1] pp

# null while the task is not moving, or not read yet.
# Empty or one element.
RapidPointer[<=1] mp
