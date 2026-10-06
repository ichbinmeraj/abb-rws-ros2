# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# The controller's event log (slice 6, added in schema 0.10). The daemon reads it on every
# connection, then follows it: while the link's event subscription is live the controller announces
# each new message and the log is read again; while none delivers, the daemon reads the log itself
# every few seconds. Messages are de-duplicated across those reads and across reconnects, and
# numbered in the controller's order (`seq`). The daemon keeps the last EVENTS_KEEP_MAX per
# controller; this group carries the newest EVENTS_RECENT_MAX of them, and `GET
# /v0/controllers/:id/events` the rest. Before a link to a controller with a known system id is
# confirmed, nothing is read from it: recorded events outlive the link.

# Names the numbering `seq` belongs to. It changes when the daemon starts or the controller is
# added again, and `seq` starts again at 1: a consumer holding a cursor from another feed starts
# over. null while this group carries nothing (a link on the WRONG controller).
# Valid only when feed_known is true.
string feed
bool feed_known

# The `seq` of the newest event recorded; 0 before the first. A consumer that has read up to a
# lower one fetches the rest with `since`.
int64 last_seq

# The newest events, oldest first, at most EVENTS_RECENT_MAX.
ControllerEvent[<=20] recent

# How new messages reach the daemon now: `subscription` (the controller announces each one), `poll`
# (the daemon reads the log every few seconds, because no subscription delivers), or null while it
# cannot read the log (the link is down or degraded, or not yet confirmed to be the controller that
# was added).
# Empty when not known.
string via
string VIA_SUBSCRIPTION="subscription"
string VIA_POLL="poll"

EventClock clock

# When the list was last known complete: while `via` is `subscription` and the log has been read
# since the stream began, now (every new message is announced); otherwise the last successful read.
# Frozen once the link degrades. null before the first read.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known

# The daemon's last read of the log failed; null once a read succeeds.
# Empty or one element.
GroupError[<=1] error
