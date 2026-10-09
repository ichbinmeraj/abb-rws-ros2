# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Capabilities (added in schema 0.13): what the daemon can read on this controller and how it
# arrives. Every row is derived from what the client reports (the protocol, and the generation as
# the client determined it from the reported RobotWare version) and from the outcome of the
# daemon's own reads.

# The protocol the link speaks; null while not connected.
# Empty when not known.
string protocol
string PROTOCOL_RWS1="rws1"
string PROTOCOL_RWS2="rws2"

# Same value as `access.generation`.
# Empty when not known.
string generation
string GENERATION_RW6="rw6"
string GENERATION_RW7="rw7"
string GENERATION_RW8="rw8"

# The RobotWare version exactly as the controller reported it; null until read.
# Valid only when robot_ware_known is true.
string robot_ware
bool robot_ware_known

# The RobotWare major.minor is one this daemon was tested against; null while the version is not
# known.
int8 tested
int8 TESTED_UNKNOWN=-1
int8 TESTED_FALSE=0
int8 TESTED_TRUE=1

# One row per `CapabilityId`, in that order; empty for a link on the wrong controller.
Capability[] entries

# Same meaning as `panel.as_of`.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
