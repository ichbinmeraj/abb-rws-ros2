# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

# Mechanical unit, e.g. ROB_1.
string name

# robot for a TCP robot, positioner / external otherwise; null when the type is unknown.
# Empty when not known.
string kind
string KIND_ROBOT="robot"
string KIND_POSITIONER="positioner"
string KIND_EXTERNAL="external"

# The mechanical unit type as reported (e.g. TCPRobot).
# Valid only when type_raw_known is true.
string type_raw
bool type_raw_known

# The RAPID task that drives this unit; null when none or unknown.
# Valid only when task_known is true.
string task
bool task_known

MotionJoint[] joints

# TCP pose for a robot unit; null for others or before the first read.
# Empty or one element.
MotionPose[<=1] pose

# The unit's last joint read failed, so `joints` are the last good values (or empty) - or, while no
# joint is reported at all, its description read failed. Never set by a failed pose read alone: the
# pose then keeps its last value (or stays null) while the joints go on being current.
# Empty or one element.
GroupError[<=1] error
