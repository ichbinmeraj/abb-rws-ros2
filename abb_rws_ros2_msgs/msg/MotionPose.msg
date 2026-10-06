# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

# TCP position in mm.
# Unit: mm.
float64 x

# Unit: mm.
float64 y

# Unit: mm.
float64 z

# Orientation quaternion [q1, q2, q3, q4] = [w, x, y, z].
# Unit: quaternion-wxyz.
float64[4] q

# Valid only when tool_known is true.
string tool
bool tool_known

# Valid only when wobj_known is true.
string wobj
bool wobj_known

# The frame the pose is given in (Wobj, Base, World, Tool), as reported.
# Valid only when coord_known is true.
string coord
bool coord_known
