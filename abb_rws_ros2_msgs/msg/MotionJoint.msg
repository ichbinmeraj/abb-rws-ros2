# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One axis of a mechanical unit. Values as RWS reports them: degrees for a revolute axis,
# millimetres for a prismatic one (RWS converts from the SI values - radians, metres - the
# controller stores). `unit` null: not known.

# 'J1'..'J6' for the robot axes (rax_1..6), 'E1'..'E6' for external axes (eax_a..f).
string name

# In `unit`; null when not read yet.
# In the unit named by unit.
# NaN when not known.
float64 value

# deg (revolute) or mm (prismatic); null: not known, so a consumer must not convert the values.
# Empty when not known.
string unit
string UNIT_DEG="deg"
string UNIT_MM="mm"

# Lower / upper limit in the same unit; null when the daemon cannot read it.
# In the unit named by unit.
# NaN when not known.
float64 min

# In the unit named by unit.
# NaN when not known.
float64 max
