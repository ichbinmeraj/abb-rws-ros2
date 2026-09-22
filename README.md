# abb-rws-ros2

ROS 2 supervisory bridge for ABB robot controllers over Robot Web Services: RWS 1.0 (IRC5 / RobotWare 6) and RWS 2.0 (OmniCore / RobotWare 7 and 8).

> **Status: early development.** Nothing to install yet. Watch or star the repo to follow progress.

## What it does

Built on [abb-rws-client](https://github.com/ichbinmeraj/abb-rws-client), it exposes an ABB controller to ROS 2:

- Joint states, controller state and I/O published as ROS 2 topics (standard `sensor_msgs/JointState`)
- Services to start/stop RAPID execution, request mastership and write signals
- Runs in Docker

## What it does not do

No real-time motion. This is a supervisory bridge, not a motion driver. Real-time control over EGM may follow later.

## Planned packages

| Package | Role |
|---|---|
| `abb_rws_ros2` | Bringup and launch files |
| `abb_rws_ros2_msgs` | Message and service definitions |
| `abb_rws_bridge` | Supervisory node |

## Related projects

- [abb-rws-client](https://github.com/ichbinmeraj/abb-rws-client): TypeScript RWS client tested on RobotWare 6, 7 and 8
- [abb-rws-ros2](https://github.com/ichbinmeraj/abb-rws-ros2): ROS 2 supervisory bridge
- [abb-rws-conformance](https://github.com/ichbinmeraj/abb-rws-conformance): conformance suite and compatibility matrix
- [abb-rws-mcp](https://github.com/ichbinmeraj/abb-rws-mcp): MCP server for AI agents
- [abb-rws-vscode](https://github.com/ichbinmeraj/abb-rws-vscode): RAPID Live, VS Code extension for ABB controllers

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE). If you redistribute this work, keep the NOTICE file.

## Disclaimer

Independent open-source project by [Meraj Safari](https://github.com/ichbinmeraj). Not affiliated with, endorsed by or sponsored by ABB. ABB, RobotWare, OmniCore, IRC5 and RobotStudio are trademarks of ABB. See [TRADEMARKS.md](TRADEMARKS.md).
