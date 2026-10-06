# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# Motion (slice 4). Joint positions are not subscribable on any generation (no subscription
# resource carries them), so the daemon polls them - but only while a consumer holds a watch lease
# (`POST /v0/controllers/:id/motion/watch`); otherwise nothing is sampled and `watching` is false.
# One sampler per controller serves every consumer: `watching` is true while any consumer's lease
# runs (since schema 0.6 each consumer may hold its own, `leaseId`). Joints at `rate.target_hz`,
# the pose at a quarter of that; both ride the controller's own session, whose request spacing
# keeps the controller under its 20 req/s ceiling. The target drops while the link is `polling`
# (the manager's own polls then use ~12 req/s, as measured on virtual controllers). The stream
# coalesces this group like `panel` (at most 4 frames/s).

bool watching

MotionUnitInfo[] units

MotionRate rate

# When the positions were last read; frozen when sampling stops or the link degrades.
# Valid only when as_of_known is true.
builtin_interfaces/Time as_of
bool as_of_known
