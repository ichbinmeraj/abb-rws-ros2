# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Is the controller on this link the one that was added? `confirmed`: its system id matches the
# configured one. `unverified`: one of the two ids is not known yet. `mismatch`: it is a DIFFERENT
# controller - never act on it.

string status
string STATUS_CONFIRMED="confirmed"
string STATUS_UNVERIFIED="unverified"
string STATUS_MISMATCH="mismatch"

# The system id the controller was added with; null when not known.
# Valid only when expected_known is true.
string expected
bool expected_known

# The system id the controller on the link reports; null when not read yet.
# Valid only when actual_known is true.
string actual
bool actual_known
