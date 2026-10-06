# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Where a RAPID task is: the program pointer (the next instruction to run) or the motion pointer
# (the instruction the robot is moving on). `line` is the 1-based source row as the controller
# reports it.

string module

# Valid only when routine_known is true.
string routine
bool routine_known

# Valid only when line_known is true.
int64 line
bool line_known
