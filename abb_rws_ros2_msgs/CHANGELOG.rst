^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package abb_rws_ros2_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Messages generated from the abb-rws-daemon JSON Schema (version 0.13):
  ``ControllerSnapshot`` and the 32 messages it refers to, with the access,
  capabilities and freshness groups, ``LinkGroup.down_since``, and
  ``LinkGroup.mode`` (``live`` or ``replay``) with ``LinkGroup.recorded_at``.
* Field names are the schema's ``x-ros-name``: the generator renames nothing.
  ``IdentityGroup.is_virtual`` is named so by the daemon's schema.
* The generator ``scripts/generate_msgs.py``, the vendored schema and the
  JSON-to-ROS field mapping ``schema/v0.mapping.json``.
