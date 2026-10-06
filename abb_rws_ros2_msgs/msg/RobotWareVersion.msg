# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.

# Exactly what the controller reported, e.g. "7.21.0+229" or "6.16.3007".
# Valid only when raw_known is true.
string raw
bool raw_known

# Valid only when major_known is true.
int64 major
bool major_known

# Valid only when minor_known is true.
int64 minor
bool minor_known

# Empty when not known.
string generation
string GENERATION_RWS1="rws1"
string GENERATION_RWS2="rws2"
