# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# How often the motion sampler reads (named in schema 0.12).

# The rate the sampler aims at; null when not sampling.
# Unit: Hz.
# NaN when not known.
float64 target_hz

# Measured over the last few seconds; null when not sampling.
# Unit: Hz.
# NaN when not known.
float64 achieved_hz

# Empty when not known.
string via
string VIA_POLL="poll"
