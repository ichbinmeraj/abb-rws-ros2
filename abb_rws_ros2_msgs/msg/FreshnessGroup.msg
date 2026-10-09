# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Freshness (added in schema 0.13): whether what the snapshot shows is current, said once for the
# controller and once per group, so a consumer needs no thresholds of its own. It is worked out
# from the other groups of the same snapshot and changes only when one of them does.

string level
string LEVEL_CURRENT="current"
string LEVEL_DEGRADED="degraded"
string LEVEL_DOWN="down"
string LEVEL_WRONG_CONTROLLER="wrong-controller"

# One plain sentence per cause while the level is not `current`; empty when it is.
string[] reasons

# One entry per `FreshGroupName`, in that order.
GroupFreshness[] groups
