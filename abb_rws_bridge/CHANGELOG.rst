^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package abb_rws_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Supervisory bridge node for abb-rws-daemon. Read-only; not a motion driver.
* Needs a daemon that serves snapshot schema 0.13 or later: an older one is
  refused and nothing is published; a newer minor is accepted with a warning.
