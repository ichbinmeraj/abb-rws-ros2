# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# A failure inside a snapshot group (a motion unit's read, a watched signal's): `code` and
# `message` as `ErrorBody.code` and `ErrorBody.error`, and the same `controller` and `refusal`
# (added in schema 0.5).

string code

string message

# Empty or one element.
ControllerAnswer[<=1] controller

# JSON text.
# Empty when not known.
string refusal
