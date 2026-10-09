# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# I/O. Signals are watched behind the same lease pattern as motion (`POST
# /v0/controllers/:id/io/watch`): nothing is subscribed or polled until a consumer asks, and a
# lapsed lease releases the signals again. The watch list is capped (`IO_WATCH_MAX`); the full
# signal table is served by `GET /v0/controllers/:id/io/signals` (search), not by this group.
#
# Since schema 0.6 each consumer may hold a lease of its own (`leaseId`), and `signals` is the
# union of every lease's set: a consumer picks out the names its own grant accepted. A consumer's
# lease that runs out while another is still held takes out only the signals no other lease holds;
# when the last one runs out, the whole set stays listed with its last values, `watching` false.

# A consumer holds a lease - any consumer, not necessarily the reader.
bool watching

# The watched signals, every lease's together: at most IO_WATCH_MAX.
IoWatchedSignal[<=24] signals

# When a value was last confirmed; frozen when the lease lapses or the link degrades.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
