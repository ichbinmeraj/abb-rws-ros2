# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# When a controller event happened. The controllers seen so far report their clock's reading
# without a zone ("2026-07-09 T 01:29:56"), in whatever zone and with whatever error the
# controller's clock is set to. The daemon reads the controller's clock on every new connection and
# every CLOCK_REFRESH_MS, against its own UTC time, and shifts each event's time by that offset
# (see `EventClock`) - good to about a second, the clocks' resolution.

# Exactly what the controller reported; null when it reported nothing.
# Valid only when raw_known is true.
string raw
bool raw_known

# `raw` in UTC. With `corrected` true: shifted by the controller clock's offset. With it false no
# offset was known when the event was recorded, and this is the controller's reading taken as if
# its clock ran on UTC - off by its zone and its error. null when `raw` could not be read as a date
# and time.
# Valid only when utc_known is true.
builtin_interfaces/Time utc
bool utc_known

# `utc` was corrected with the controller clock's offset. An event recorded before the clock could
# be read is corrected once it is (this turns true).
bool corrected
