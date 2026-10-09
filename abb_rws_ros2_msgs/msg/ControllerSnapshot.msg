# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# One controller as the daemon serves it: every group, each with its own freshness. The root of the
# exported schema, and the ROS message of the same name.

# header.stamp is when the daemon reported the state this message holds: the observedAt of the
# stream frame, or of the controller list, that last delivered or confirmed it. A publisher set to
# stamp with its own clock uses the time that frame or list arrived. header.frame_id is empty.
std_msgs/Header header

string id

string name

# null until the controller has been read at least once.
# Empty or one element.
IdentityGroup[<=1] identity

LinkGroup link

PanelGroup panel

# Added in schema 0.2. Empty tasks until the controller has been read.
RapidGroup rapid

# Added in schema 0.3. `watching` false and no values until a consumer watches.
MotionGroup motion

# Added in schema 0.4. `watching` false and no signals until a consumer watches.
IoGroup io

# Added in schema 0.10. No events until the log has been read.
EventsGroup events

# Added in schema 0.13. Nothing read until the controller is connected.
AccessGroup access

# Added in schema 0.13.
CapabilitiesGroup capabilities

# Added in schema 0.13.
FreshnessGroup freshness
