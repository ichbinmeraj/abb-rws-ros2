# abb-rws-ros2

ROS 2 supervisory bridge for ABB robot controllers. It reads
[abb-rws-daemon](https://github.com/ichbinmeraj/abb-rws-daemon) and republishes what the
daemon knows as ROS 2 topics: joint states, a snapshot of each controller, I/O signals and
diagnostics.

> **Status: early development, read-only.** Version 0.1.0, built from source. It is not
> released: there is no apt package, no rosdistro entry and no published image. ROS 2 Jazzy
> only so far. Topic names, parameters and messages may still change. What has been tested
> and what has not is under [Tested with](#tested-with).

## What it is

A supervisory bridge, not a motion driver.

```
controller  <- RWS ->  abb-rws-daemon  <- HTTP and WebSocket, loopback ->  bridge  ->  ROS 2 topics
```

- One node, `abb_rws_bridge` (executable `bridge`), holds one connection to one daemon and
  bridges any number of controllers.
- The bridge holds no Robot Web Services knowledge. The daemon speaks RWS to the
  controllers (RWS 1.0 on IRC5 with RobotWare 6, RWS 2.0 on OmniCore with RobotWare 7 and 8)
  and owns the session on each of them. The bridge speaks to the daemon only.
- Three packages in this repository: the messages, the node and the bringup. See
  [Packages and messages](#packages-and-messages).

## What it publishes

| Topic | Type | Published |
|---|---|---|
| `<c>/joint_states` | `sensor_msgs/JointState` | When the joint positions change or are confirmed, and only while they are current |
| `<c>/controller_snapshot` | `abb_rws_ros2_msgs/ControllerSnapshot` | On every stream frame that touches the controller, and at `republish_hz`, while the daemon answers |
| `<c>/io/<signal>` | `std_msgs/Bool`, `std_msgs/Float64` or `std_msgs/UInt32` | On change and at `republish_hz`, while the value is current |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Always: every `diagnostics_period_s`, and at once when a level changes |

The data topics are relative to the node's namespace (empty by default); `/diagnostics`
is absolute. `<c>` is the controller's name in topic names, made from its id in the
daemon: `cell-1` gives `cell_1/joint_states`. One controller can be named the primary one
(`primary_controller`); its topics have nothing in front: `joint_states`,
`controller_snapshot`, `io/<signal>`. No controller is primary by default.

`controller_snapshot` is the whole of what the daemon knows about one controller: its id
and name, and ten groups (identity, link health, panel state, RAPID, motion, I/O, the
newest event log messages, access, capabilities and freshness). Every data publisher is reliable, volatile and keeps the last
10 messages. Nothing is latched: a late subscriber is never handed an old sample.

### Units

| | `joint_states` | `controller_snapshot` |
|---|---|---|
| Follows REP-103 | yes | **no** |
| Revolute joints | radians | degrees |
| Linear joints | metres | millimetres |
| Unit of a value | fixed by the message type | named per joint: `deg`, `mm`, or empty when not known |
| TCP pose | not published | millimetres, and a quaternion in the order w, x, y, z |

`joint_states` follows [REP-103](https://www.ros.org/reps/rep-0103.html).
`controller_snapshot` does not: it carries degrees and millimetres, as the controller
reports them, with the unit named per joint. Only `sensor_msgs/JointState` is converted.

A joint whose unit is not known is never converted and never guessed. It is left out of
`joint_states` and stays, untouched, in `controller_snapshot`.

`velocity` and `effort` in `joint_states` are empty. They are not measured, and zeros
would say otherwise.

### Freshness

One rule for every data topic: **a silent topic means there is no current data, and
`/diagnostics` always speaks and says why.**

- When the daemon cannot be reached, or has not answered for `stale_after_s`, every data
  topic falls silent, `controller_snapshot` included.
- When the daemon still answers but its stream has stopped delivering (a connection that is
  open in name only, for example through a forwarder after the host was suspended), the
  bridge sees it in the controller list it asks for: a value the list shows that no stream
  frame brought. After `stale_after_s` of that, every data topic falls silent, `/diagnostics`
  says so (`stream-stalled`), and the bridge connects again. The time such a value was
  published as current is bounded: one refresh period plus a quarter of a second plus
  `stale_after_s`.
- `joint_states` is published only while the controller's link is up, its joints are
  watched and the daemon has confirmed them within `stale_after_s`. A robot that stands
  still is confirmed again and again (the bridge asks the daemon `refresh_hz` times a
  second), so its topic keeps speaking. A joint reading that was already older than
  `stale_after_s` when the bridge first saw it (against the time the daemon sent it) is
  not published; the next reading the daemon takes is.
- An I/O value is published only while the controller's link is up, the daemon watches the
  signal for the bridge and reports no error for it.

`/diagnostics` carries one status for the daemon (`<node name>: daemon`) and one for each
controller (`<node name>: controller <controller id>`): every controller the bridge is told
to bridge, or every one the daemon serves, plus every controller that `io_signals` or
`joint_name_overrides` names, so a mistyped id shows up as "not served by the daemon"
instead of a topic that never appears. A status that is not OK names the reason, for
example `unreachable`, `host-refused`, `token-file`, `token-refused`, `replay-refused` or
`stream-stalled`, and what to do about it.

### Stamps

`stamp_source` picks the clock behind `header.stamp` of `joint_states` and
`controller_snapshot`:

| `stamp_source` | Stamp |
|---|---|
| `daemon` | The daemon's time: when it read the joints, or when it reported the state the snapshot holds |
| `bridge` | This machine's clock at the moment that reading or state arrived |
| `auto` (default) | `daemon` when `daemon_url` names `127.0.0.1`, `localhost` or `::1` (same machine, same clock), otherwise `bridge` (a container's clock is not the host's clock) |

- Stamps on one topic never go backwards. A message whose stamp would be older than the
  last one published is held back and counted in `/diagnostics`.
- A keep-alive of `controller_snapshot` repeats the stamp of the state it repeats, so a
  subscriber tells a repeat from news by the stamp.
- The times inside a snapshot stay the daemon's, whatever `stamp_source` says.
- Every stamp is a wall-clock time. Simulated time is not supported: with `use_sim_time`
  set to true the node does not start.

### I/O values

A digital signal is a `std_msgs/Bool`, an analog signal a `std_msgs/Float64` and a group
signal a `std_msgs/UInt32`. These messages carry no stamp. So:

- A value is published only while it is current. Every message, a change or a keep-alive,
  says: this is the value, confirmed within `stale_after_s`.
- The time of each signal and how the daemon reads it (`as_of`, `via`) are in the `io`
  group of `controller_snapshot`, which is stamped.
- A topic never changes its type. If a controller later reports another type for the same
  signal name, the bridge stops publishing that signal and reports an error; a restart of
  the bridge picks the new type up.

### The joints in `joint_states` can change

All mechanical units of a controller go into one message, in the daemon's order. A joint is
left out while its unit is not known or is neither degrees nor millimetres, while it has
no value, and while its mechanical unit reports an error. The joints that are left out are
listed in `/diagnostics`, each with the reason, and stay visible in `controller_snapshot`.

So the set of joints in `joint_states` can change over time. `robot_state_publisher` keeps
the last transform of a joint that is missing from a message: a model can show a joint
standing still that the bridge has stopped reporting. When no joint is left, nothing is
published.

### Rates

This is a supervisory rate. The daemon samples the joints of a watched controller at up to
8 Hz and sends at most 4 frames a second for one controller. That is enough to show a
cell, log it or supervise it. It is not enough to control anything.

## What it does not do

- No motion, no writes, no RAPID start or stop: it offers no services of its own (only the
  parameter and type-description services every ROS 2 node has). The daemon has no write
  path yet, and the bridge ships no service that pretends there is one.
- It subscribes to nothing.
- No URDF, no TF and no pose topic. The TCP pose is in `controller_snapshot` as the daemon
  gives it. The daemon does not yet say which tool, work object and coordinate system a
  pose refers to, and the bridge does not guess a frame.
- Not real time, and not a `ros2_control` hardware interface.
- No simulated time.

## It needs the daemon

The bridge speaks to [abb-rws-daemon](https://github.com/ichbinmeraj/abb-rws-daemon) and
to nothing else. The daemon's README says how to build it, start it and add controllers.

- The daemon must serve snapshot schema 0.14 or later. An older one is refused: nothing is
  published and `/diagnostics` says so. A newer minor version is accepted with a warning,
  and what the messages do not know is ignored.
- The bridge holds no controller credentials. User names and passwords stay with the
  daemon. The one secret the bridge reads is the daemon's access token, from a file (see
  [The token](#the-token)).
- The daemon listens on 127.0.0.1 only, so the bridge runs on the same machine as the
  daemon, or in a container on that machine.
- The daemon is in early development and read-only too. It installs from npm
  (`abb-rws-daemon@0.1.0`) or builds from source.

## What a running bridge causes

Read this before the first command. With the default parameters, a bridge that connects:

- **Bridges every controller the daemon serves**, and holds a motion watch on each of
  them. A motion watch makes the daemon read that controller's joint positions and TCP
  pose (up to 8 Hz) for as long as the bridge runs, whether or not anything subscribes,
  and it takes one of that controller's lease slots in the daemon.
- Opens one stream connection to the daemon, asks it for the controller list `refresh_hz`
  times a second and for its health every 5 s. The daemon answers these from what it
  already holds; they add no request to a controller.
- Watches no I/O signal, unless `io_signals` names some: then those, and only those.

`controllers` limits the bridge to the controllers you name. `watch_motion` set to false
stops it from asking for joints at all; they then appear only while another consumer of
the daemon watches them. A controller whose joints nobody watches then keeps an OK status
that says so; with `watch_motion` true the same state is reported as stale.

The bridge sends the daemon nothing but these requests: health, the stream, the controller
list and the two watch requests. When it stops, it frees its I/O signals at once. A motion
watch has no release: it runs out by itself within 15 s.

## Run it

Start the daemon first, on every path: it creates the token file on its first start, and
the token should go to the daemon and to nothing else (see [The token](#the-token)).

| Where the bridge runs | `daemon_url` | Token file | Daemon option |
|---|---|---|---|
| Same Linux host as the daemon, built from source | `http://127.0.0.1:7810` | default path, same account | none |
| Container on Linux with host networking | `http://127.0.0.1:7810` | mounted read-only | none |
| Docker Desktop on Windows or macOS | `http://host.docker.internal:7810` | mounted read-only | `--allowed-hosts host.docker.internal` |
| Container on Linux without host networking | not reachable: the daemon listens on loopback only and has no option to widen that | | |
| Any other host | refused at start unless `allow_remote_daemon` is set; not something this project sets up | | |

### Linux, from source

On Ubuntu 24.04 with ROS 2 Jazzy, next to a daemon on the same host and under the same
account:

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ichbinmeraj/abb-rws-ros2.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch abb_rws_ros2 bridge.launch.py
```

Every dependency is a rosdep key that resolves to an Ubuntu or ROS apt package; nothing is
installed with pip. The discovery range line keeps the topics on this host; see
[Who can read the topics](#who-can-read-the-topics) before you leave it out.

In a second terminal (same setup, same discovery range):

```sh
ros2 topic echo --once /diagnostics
ros2 topic list
```

The launch file takes the arguments `namespace`, `params_file`, `daemon_url`,
`token_file`, `controller`, `allow_replay` and `log_level`. An argument that is left out
does not override the parameter file. `controller:=cell-1` bridges that one controller and
makes it the primary one:

```sh
ros2 launch abb_rws_ros2 bridge.launch.py controller:=cell-1
ros2 topic echo joint_states
```

For everything else, copy `abb_rws_ros2/config/bridge.yaml`, change it and pass it with
`params_file:=<your file>` (see [Configuration](#configuration)).

What to know about the launch file:

- An argument that is given wins over the parameter file, with one exception: a section
  of your own parameter file that names the node in full wins over the launch arguments.
  Put the parameters under `/**`, as the shipped file does, and the arguments win.
- `controller:=` takes the id as text. An id that reads as a number or as yes or no (such
  as `08`, `1e3` or the single letter y) arrives at the node as something else, and the
  node ends with a line that says so. Name such a controller in a parameter file, in
  double quotes.
- When the node ends with an error, for example because it cannot start with its
  parameters, `ros2 launch` ends with exit code 1.
- Stop it with Ctrl+C (SIGINT). Launch hands that on to the node, which closes its stream
  and frees its I/O signals before it ends. On SIGTERM launch ends at once and leaves the
  node running.

### Docker Desktop (Windows or macOS)

The daemon runs on the host, the bridge in a container. A container reaches the host under
the name `host.docker.internal`, and the daemon refuses a request that names any host but
a local one. The bridge does not send another host name to get past that. The daemon has
an option for it instead:

1. Start the daemon with `--allowed-hosts host.docker.internal`. The option names one more
   host name the daemon accepts. The daemon still listens on 127.0.0.1 only and still
   asks for the token. The option is new in abb-rws-daemon: a daemon that knows it logs a
   warning at start that names the extra hosts ("extra host names accepted in the Host
   header"); a daemon built before it ignores the option without a word and keeps
   refusing the name. If that warning is missing from the daemon's log, update the daemon.
2. Set `ABB_RWS_DAEMON_TOKEN_FILE` to the path of the daemon's token file, and check that
   the file is there.
3. Start the bridge from the repository root.

In PowerShell:

```powershell
$env:ABB_RWS_DAEMON_TOKEN_FILE = "$env:APPDATA\abb-rws-daemon\token"
Test-Path -PathType Leaf $env:ABB_RWS_DAEMON_TOKEN_FILE   # must print True
docker compose -f docker/compose.yaml up --build
```

| Variable | Default | Meaning |
|---|---|---|
| `ABB_RWS_DAEMON_TOKEN_FILE` | none, required | The daemon's token file on the host |
| `ABB_RWS_DAEMON_URL` | `http://host.docker.internal:7810` | Becomes `daemon_url` |
| `ROS_DOMAIN_ID` | `0` | The ROS domain of the container |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | Who can discover the bridge; see [Who can read the topics](#who-can-read-the-topics) |

What the compose file sets up:

- The container publishes no port, gets no capability and does not share the host's
  network. Its file system is read-only; `/tmp` is kept in memory. Its IPC namespace is
  marked shareable, so that a second container can read the topics (below).
- The token is one file, mounted read-only at `/run/secrets/abb_rws_daemon_token`. It is
  in no image, no environment variable, no compose file and no log. Mount the file, never
  its folder: by default that folder also holds the daemon's recordings.
- **The token file must exist before the container is created.** On Docker Desktop for
  Windows a path that did not exist was seen to be created as an empty directory, in spite
  of the setting that should prevent it. If that happened, remove the directory: the
  daemon writes its token file at that path.
- A single-file mount keeps showing the old file when the token file is replaced. Restart
  the bridge after that.
- The image sets SIGINT as its stop signal, so `docker compose down` and `docker stop`
  end the bridge the way Ctrl+C does: in order, with exit code 0.
- The compose file passes `daemon_url` and `token_file`; every other parameter keeps its
  default. To change one, replace the `command` in an override file (the way
  `docker/compose.linux-host.yaml` does) and add launch arguments, or mount a parameter
  file read-only and name it with `params_file:=`.
- Stamps come from the container's clock here, because the daemon is not on `127.0.0.1`
  from the container's point of view (see [Stamps](#stamps)).

If `/diagnostics` reports `host-refused`, the daemon runs without the option of step 1:
either it was started without it, or it is a daemon that does not know it yet (see step 1).

### Linux container with host networking

On Docker Engine on Linux, with the daemon on the same host:

```sh
export ABB_RWS_DAEMON_TOKEN_FILE="$HOME/.config/abb-rws-daemon/token"
export ABB_RWS_BRIDGE_USER="$(id -u):$(id -g)"
docker compose -f docker/compose.yaml -f docker/compose.linux-host.yaml up --build
```

The override gives the container the host's network, so the bridge reaches the daemon at
`http://127.0.0.1:7810` and the daemon needs no option. The token file is readable by its
owner only, so the container must run as that account: that is what `ABB_RWS_BRIDGE_USER`
is for (default `1000:1000`). Never loosen the mode of the token file. The discovery range
stays `LOCALHOST`, which with the host's network means: the ROS 2 nodes on this host.

The override also gives the container the host's IPC namespace (`ipc: host`). With the
default RMW (Fast DDS) two nodes that find each other on one host hand messages over through
shared memory; a container with its own IPC namespace shares none with the host's nodes,
which would list the topics and receive nothing. Readers on the host run under the account
of `ABB_RWS_BRIDGE_USER`.

This path is written and has not been run on a Linux host (see below). The mechanism was
measured under Docker Desktop with a second container: sharing the bridge container's
network namespace alone listed the topics and delivered no message; sharing its IPC
namespace too delivered them.

### Reading the topics under Docker Desktop

Under Docker Desktop the topics exist inside Docker's virtual machine. They are not on the
network of the Windows or macOS host.

With the default discovery range, `LOCALHOST`, a reader is a process in the bridge's own
container. A command started with `exec` does not pass through the image's entry point,
which is what sets up ROS, so name the entry point in front of the command:

```sh
docker compose -f docker/compose.yaml exec bridge /entrypoint.sh ros2 topic list
docker compose -f docker/compose.yaml exec bridge /entrypoint.sh ros2 topic echo --once /diagnostics
```

A second container reads the topics when it shares the bridge container's network namespace
**and** its IPC namespace (the compose file marks the IPC namespace shareable for that):

```sh
docker run --rm --network container:abb-rws-ros2-bridge-1 --ipc container:abb-rws-ros2-bridge-1 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST abb-rws-ros2:jazzy \
  /entrypoint.sh ros2 topic echo --once /diagnostics
```

Measured with a bridge started from this repository's image: with the network namespace
shared alone, the reader lists the topics and receives no message (the default RMW, Fast DDS,
hands messages between nodes on one host over shared memory, and the two containers share
none); with the IPC namespace shared as well, the first snapshot arrived within about ten
seconds. A second container on the same compose network with
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` on both sides has not been tried.

### What was run and what was not

| Path | State |
|---|---|
| Building the packages and running their tests with colcon, in a `ros:jazzy-ros-base` container without network | Run; this is how the packages are developed |
| The bridge in this repository's image under Docker Desktop on Windows, against a daemon on the Windows host started with `--allowed-hosts host.docker.internal` | Run, against a daemon in replay mode and against a live daemon with three RobotStudio virtual controllers; see [Tested with](#tested-with) |
| The same daemon without the option | Run: `/diagnostics` reports `host-refused` and the bridge sends no token |
| `docker/compose.yaml` under Docker Desktop on Windows | Run with the daemon above: OK statuses for the three controllers, a second container reading the topics (below), `docker compose stop` in 2 s with the node finishing cleanly. With the token variable pointing at a path that does not exist, `compose up` did not fail: an empty directory appeared at that path (see above) |
| A second container reading the topics (Docker Desktop) | Run: sharing the bridge container's network namespace alone lists the topics and delivers nothing; sharing its IPC namespace too delivers them |
| `docker/compose.linux-host.yaml` (Linux, host networking) | Written, not run on a Linux host |
| Linux from source next to a daemon on the same Linux host | Not run |
| ROS 2 inside WSL 2, without Docker, reaching a daemon on Windows | Not tried |
| Docker Desktop on macOS | Not tried |
| Podman (a container reaches the host as `host.containers.internal`) | Not tried |

## Who can read the topics

The daemon guards its data with a token and listens on loopback only. The bridge
republishes that data on the ROS graph, and **the ROS graph has no access control**: every
ROS 2 node that can discover the bridge reads its topics, without any token.

What `controller_snapshot` carries: the controller's and the system's names, the system
id, the MAC address, the installed RobotWare options, the host and port the controller
answers on, event log messages and the watched I/O signals. No parameter leaves the
identity out: a blanked field would be a value the daemon never reported.

Who can discover the bridge is decided by the ROS 2 environment, not by the bridge:

- `ROS_AUTOMATIC_DISCOVERY_RANGE`. Both compose files set `LOCALHOST` unless you override
  it. The value ROS 2 uses when nothing is set is `SUBNET`: every ROS 2 node on the same
  network segment. A bridge built from source follows your own environment, so set
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, or use SROS 2, unless visibility on the
  network is what you want.
- `ROS_DOMAIN_ID` separates graphs. It protects nothing: anyone can pick the same number.

## The token

The daemon's access token opens everything the daemon offers: not only the data the
bridge reads, but also the daemon's setup, recording and shutdown routes. The bridge
treats it accordingly.

- Only the path of the token file is configured (`token_file`). The token itself is never
  a parameter, a log line or a diagnostic value. The file is read again on every connect,
  so a new token is picked up.
- The token is sent to `daemon_url` and nowhere else. No redirect is followed and no proxy
  is used. It is not sent before a health request has been answered by something that
  looks like the daemon.
- That health answer proves nothing about who gave it. **The token goes to whoever listens
  on `daemon_url`.** Another account on the same machine that takes the daemon's port
  first, and answers health, receives it. So start the daemon before the bridge, and read
  `token-refused` right after a good health answer as a sign that something else holds the
  port.
- A host other than `127.0.0.1`, `localhost`, `::1`, `host.docker.internal` or
  `host.containers.internal` in `daemon_url` is refused at start unless
  `allow_remote_daemon` is true. The reason: the daemon listens on 127.0.0.1 only, so
  another host is a forwarder somebody put in front of it, and the token then travels
  unencrypted to that host. The daemon has no TLS, and `https` in `daemon_url` is refused.
  With `allow_remote_daemon` the node logs one warning at start that says so.
- The token file is readable by its owner only. The bridge never changes its mode; run the
  bridge as the account that owns the file.

## Configuration

Parameters are read once, at start, and are read-only afterwards. A value of the wrong
type, a value out of range or a combination that cannot work ends the node at start with
exit code 2 and a list of what is wrong. A double needs a decimal point in a parameter
file: write `2.0`, not `2`.

`abb_rws_ros2/config/bridge.yaml` lists every parameter with its default.

A list parameter that is empty reads as not set: `ros2 param get` on `controllers`,
`io_signals` or `joint_name_overrides` then answers that the parameter is not initialized.
That is how an empty list looks here; it is not an error.

### Parameters

| Parameter | Type | Default | Range | Meaning |
|---|---|---|---|---|
| `daemon_url` | string | `http://127.0.0.1:7810` | | Where the daemon listens: `http://<host>:<port>` and nothing else. Refused at start: `https`, a missing port, user info, a path, a query. The host must be `127.0.0.1`, `localhost`, `::1`, `host.docker.internal` or `host.containers.internal` unless `allow_remote_daemon` is true |
| `allow_remote_daemon` | bool | `false` | | Accept any host in `daemon_url`. Read [The token](#the-token) first |
| `token_file` | string | `""` | | File that holds the daemon's access token. Empty: the file named by the environment variable `ABB_RWS_DAEMON_TOKEN_FILE`, else `$XDG_CONFIG_HOME/abb-rws-daemon/token`, else `~/.config/abb-rws-daemon/token` (the daemon's own rule). Only the path is a parameter, never the token |
| `controllers` | string array | `[]` | | Daemon controller ids to bridge. Empty: every controller the daemon serves |
| `primary_controller` | string | `""` | | The topics of this controller sit directly in the node's namespace. Every other controller gets its own name in front. When `controllers` is not empty it must hold this id |
| `joint_name_format` | string | `{unit}_{joint}` | | Joint names in `joint_states`; see [Joint names for a URDF](#joint-names-for-a-urdf) |
| `joint_name_overrides` | string array | `[]` | | Entries `<controller id>/<unit>/<joint>=<name>` that name single joints. They win over `joint_name_format`. With a `controllers` list, every entry must name a controller in it |
| `joint_states_frame_id` | string | `""` | | `header.frame_id` of `joint_states` |
| `stamp_source` | string | `auto` | `auto`, `daemon`, `bridge` | Which clock stamps `joint_states` and `controller_snapshot`; see [Stamps](#stamps) |
| `watch_motion` | bool | `true` | | Hold a motion watch on every bridged controller, which makes the daemon sample its joints (up to 8 Hz). False: the bridge never asks, and joints appear only while another consumer of the daemon watches them |
| `io_signals` | string array | `[]` | | Entries `<controller id>:<signal>` to watch and publish; see [I/O signals](#io-signals). With a `controllers` list, every entry must name a controller in it |
| `allow_replay` | bool | `false` | | Publish what a daemon in replay mode serves, under other topic names; see [Replay](#replay) |
| `refresh_hz` | double | `2.0` | 0.2 to 10 | How often the bridge asks the daemon for all controllers, in Hz. This is what confirms values that did not change and what shows that the daemon still answers, so it cannot be switched off |
| `republish_hz` | double | `1.0` | 0 to 10 | Keep-alive rate of `controller_snapshot` and the I/O topics while their data is current, in Hz. 0: on change only |
| `stale_after_s` | double | `2.0` | 0.5 to 60 | A value not confirmed for this many seconds is stale, and a daemon that has not answered for as long is treated as gone. Must be at least 2 / `refresh_hz` (two chances to confirm) |
| `diagnostics_period_s` | double | `1.0` | 0.2 to 10 | Period of `/diagnostics`, in seconds |
| `reconnect_min_s` | double | `0.5` | 0.1 to 60 | Shortest wait before the bridge connects to the daemon again, in seconds. Must not exceed `reconnect_max_s` |
| `reconnect_max_s` | double | `15.0` | 1 to 300 | Longest wait before the bridge connects to the daemon again, in seconds |
| `http_timeout_s` | double | `5.0` | 0.5 to 30 | Timeout of every request to the daemon and of opening the stream, in seconds |

The node also has parameters it did not choose: `use_sim_time` (true is refused),
`start_type_description_service`, and one `qos_overrides` group for each data publisher.

A parameter file for two controllers:

```yaml
/**:
  ros__parameters:
    controllers: ["cell-1", "cell-2"]
    primary_controller: "cell-1"
    io_signals: ["cell-1:do_gripper", "cell-2:Local/DRV_1/di_ready"]
    joint_name_overrides: ["cell-1/ROB_1/J1=joint_1"]
```

It gives `joint_states`, `controller_snapshot` and `io/do_gripper` for `cell-1`, and
`cell_2/joint_states`, `cell_2/controller_snapshot` and `cell_2/io/Local_DRV_1_di_ready`
for `cell-2`.

### Joint names for a URDF

This repository ships no URDF. To match the joint names of yours:

- `joint_name_format` builds every name from placeholders: `{unit}` (the mechanical unit as
  the controller reports it, `ROB_1`), `{joint}` (`J1` to `J6` for robot axes, `E1` to `E6`
  for external axes), `{axis}` (the number in the joint name) and `{controller}` (the
  controller's name as topic names write it, `cell_1` for `cell-1`; the primary
  controller has it too, although its topics carry no name in front). The default
  `{unit}_{joint}` gives `ROB_1_J1`.
- `joint_name_overrides` names single joints and wins over the format:
  `cell-1/ROB_1/J1=joint_1`.
- Names must be unique inside one message. A format such as `joint_{axis}` gives the same
  name to `J1` and `E1`; `joint_states` of that controller then stops, with an error in
  `/diagnostics` that names the joints. Nothing is renamed silently.

### I/O signals

An entry of `io_signals` is `<controller id>:<signal>`, the signal as the daemon names it:
`do_gripper`, or with its network and device, `Local/DRV_1/do_gripper`.

- The topic is `io/` plus the signal name; a `/` in the name becomes `_`, and so does
  every other character a topic name cannot hold.
- Two entries of one controller that would get the same topic name are an error at start:
  the node does not run with them.
- The daemon accepts at most 16 signals for one controller from one consumer. What it
  refuses is listed in `/diagnostics` with the daemon's reason.
- A signal whose type is not known, or whose value does not fit its type, is not
  published, and `/diagnostics` names it.
- An entry for a controller that is not bridged has no effect, and the bridge does not let
  that pass in silence: with a `controllers` list that does not hold the id the node does not
  start; with every controller bridged, an id the daemon does not serve gets its own status,
  "not served by the daemon". The same holds for `joint_name_overrides`.

### Several controllers

Each controller's name in topic names is made from its daemon id: every character outside
letters, digits and `_` becomes `_`, runs of `_` become one, a `_` at the end is dropped,
and an id that starts with a digit gets `_` in front. ROS 2 tools hide names that start
with `_`: `ros2 topic list --include-hidden-topics` shows them.

Two ids that would get the same name, or an id whose name is one the bridge uses itself
(`joint_states`, `controller_snapshot`, `io`, `diagnostics`, `replay`), are not repaired
with a suffix: the later controller is not bridged, and its status in `/diagnostics` is an
error.

### QoS overrides

Depth, history and reliability of every data publisher can be set from the parameter
file, under the full name of the topic. `/diagnostics` is fixed.

```yaml
/**:
  ros__parameters:
    qos_overrides:
      /cell_1/joint_states:
        publisher:
          depth: 1
          reliability: best_effort
```

### Replay

A daemon started in replay mode serves a recording, not a robot.

- `allow_replay` false (the default): nothing is published from such a daemon, and
  `/diagnostics` reports `replay-refused`.
- `allow_replay` true: the data is published, but never on the names above. Every data
  topic gets `replay` in front: `replay/<c>/joint_states`,
  `replay/<c>/controller_snapshot`, `replay/<c>/io/<signal>`, and `replay/joint_states`
  for the primary controller. Every status in `/diagnostics` carries a warning that
  starts with `REPLAY`.

So a subscriber wired to `joint_states`, or a bag recorded from it, cannot be fed a
recording because a parameter was flipped. The bridge checks the daemon's mode again every
5 s; when it changes, the connection ends and the bridge decides afresh.

## Packages and messages

| Package | Build type | Role |
|---|---|---|
| `abb_rws_ros2_msgs` | `ament_cmake` | The 33 messages, `ControllerSnapshot` and what it is made of |
| `abb_rws_bridge` | `ament_python` | The node |
| `abb_rws_ros2` | `ament_python` | Bringup: the launch file and the parameter file |

Every message is generated from the daemon's JSON Schema; none is written by hand.

- `abb_rws_ros2_msgs/schema/v0.schema.json` is a copy of the daemon's schema file, byte
  for byte (version 0.14). `schema/SOURCE.json` records its version and hashes.
- `abb_rws_ros2_msgs/scripts/generate_msgs.py` (Python standard library only) writes the
  `.msg` files, the list CMake builds from, and `schema/v0.mapping.json`, which says how
  each JSON property maps to a message field. The bridge converts from that file, so the
  conversion cannot drift from the messages.
- What the daemon reports as not known stays not known: a number is NaN; a text, a whole
  number or a time that may be missing has a `<name>_known` flag next to it; a value from
  a fixed list is the empty string; a yes-or-no value has three states; and an object that
  may be missing is an array of none or one.
- `ControllerSnapshot` starts with a `std_msgs/Header`, which the schema does not have;
  see [Stamps](#stamps). Field names are the schema's `x-ros-name`, renamed by none
  here: `virtual` is a C++ keyword, so the daemon's schema itself names the identity
  group's field `is_virtual`.

```sh
# Are the generated files what the generator would write? Writes nothing; exit 1 if not.
python3 abb_rws_ros2_msgs/scripts/generate_msgs.py --check

# Generate again from the vendored schema.
python3 abb_rws_ros2_msgs/scripts/generate_msgs.py

# Take a new schema file from a daemon checkout (here: next to this one), then generate.
python3 abb_rws_ros2_msgs/scripts/generate_msgs.py --vendor ../abb-rws-daemon/schema/v0.schema.json
```

The check also runs as a test of the message package. The schema version the messages
come from is not final, so the messages may change until the first release.

## Tested with

Every controller below is a RobotStudio virtual controller: RobotWare 6.16.03 with RWS 1.0,
and RobotWare 7.21.0, 7.21.4, 8.1.1 and 8.2.1 with RWS 2.0. The bridge has not run against
a real controller yet.

- **Automated tests**, run with colcon in a container, without network, without a daemon
  and without a controller: unit tests of the conversion, naming, state and diagnostics
  code; tests of the daemon client against a stand-in daemon written for the tests; node
  tests on a ROS graph against the same stand-in; tests of the generator and of the
  generated messages. All test data is written by hand. These tests do not show that the
  stand-in behaves like the daemon.
- **Against a running daemon, replaying a recording** (abb-rws-daemon 0.1.0-dev built from
  source with `--allowed-hosts`, schema 0.13; the bridge in this repository's image under
  Docker Desktop on Windows, reaching the daemon as `host.docker.internal`): the daemon
  client alone (health, a wrong token refused, a snapshot and every frame, controller list
  and lease answer valid against the schema, one lease id kept over renewals and
  reconnects, the restart of a looping recording read as `replay-restart`), then the node:
  `replay-refused` without `allow_replay`; with it, topics under `replay/` for every
  recorded controller (three in one recording, five in another) and none on the live names,
  the recorded joints in radians, stamps never stepping back within a turn of the
  recording, status WARN starting with `REPLAY`. Where a recording holds no joint frame
  for a stretch that, at the replay speed, lasts longer than `stale_after_s`, the
  controller's status reads "joints not confirmed" for that stretch.
- **Against a live daemon and three RobotStudio virtual controllers** (the same daemon
  build; RobotWare 6.16.03 on an IRB 1600 with RWS 1.0, RobotWare 7.21.0 and 8.1.1 on
  CRB 1910 with RWS 2.0; nothing was written to a controller): `joint_states`,
  `controller_snapshot` and two I/O signals per controller for 60 s, with the daemon's
  stamps, next to a capture of the daemon's own stream. Every published snapshot (335)
  agreed with the captured state on id, link quality, controller state, operating mode,
  execution state, joints with their units, signal values and the event sequence. Every
  `JointState` carried the captured joints in radians (389 of 403 by the capture's own
  frames; 3 were published from a controller list a moment before the stream delivered a
  transient read error for the same sample, with the last good values; 11 fell after the
  capture had ended). The link of the 7.21.0 controller went stale once for a few seconds
  and its joints were reported as left out meanwhile. With the default stamp source
  (the container's clock here) stamps lay within 0.4 s of arrival and never stepped back,
  and the daemon status named the stamp source and the clock offset. After the bridge
  stopped, the daemon showed the I/O watch set empty at once and the motion watch gone
  within 15 s.
- **The same, a second time, with the PC fully loaded by other work** (the virtual
  controllers answered slowly and some reads timed out): in two 60 s comparisons every
  `JointState` (238 and 391) and every snapshot (275 and 302) agreed with the captured
  stream. Right after the start the daemon had not yet read the axis configuration of two
  controllers and reported their joints without a unit; the bridge left those joints out
  of `joint_states` and named them in `/diagnostics`, and published them once the unit was
  known. Twice the bridge found the stream 2 s behind the controller list, reported
  `stream-stalled` and was connected again within a second. Once the load had dropped, a
  four-minute comparison of the daemon's controller list and stream, by the bridge's own
  rule, found no difference that lasted longer than 0.6 s.
- **A full check of the bridge's behaviour, 2026-10-13** (abb-rws-daemon 0.1.0-dev built
  from source with `--allowed-hosts host.docker.internal`, schema 0.13; the image built by
  `docker compose up --build` under Docker Desktop on Windows; RobotWare 6.16.03 with RWS
  1.0, RobotWare 7.21.4 and 8.2.1 with RWS 2.0, each on an IRB 1600; nothing was written to
  a controller): 14 checks, all passed.
  - The image built, and the node connected and published. In the development image the
    colcon tests passed (2910 tests, 0 failures, 6 skipped), and the generated messages
    matched the vendored schema.
  - Each controller had its namespace with `controller_snapshot` and `joint_states`, and
    `/diagnostics` had one OK status for the daemon and one per controller, at 1 Hz.
  - The 7.21.4 snapshot carried all ten groups, `is_virtual` set and the RobotWare version,
    and its `joint_states` named `ROB_1_J1` to `ROB_1_J6` in radians (J5 at 0.5236 rad, 30
    degrees). Over 60 s, `joint_states` arrived at about 2 Hz from each controller. No joint
    moved during the run, so a moving joint was not followed.
  - I/O signals from a mounted parameters file: three digital signals as `std_msgs/Bool`
    and a group signal as `std_msgs/UInt32`, 1 Hz each, with the daemon's values. These
    controllers have no analog signal, so `std_msgs/Float64` was not exercised.
  - `controller:=<id>` bridged that one controller and no other.
  - The daemon started without `--allowed-hosts`: `/diagnostics` read `host-refused`, and
    the bridge went no further than the health request, which carries no token.
  - A replay of a 60 s recording made while the bridge ran: refused by default; with
    `allow_replay:=true`, every topic under `replay/` and none on the live names, every
    status starting with `REPLAY: recorded`, `link.mode` replay and `recorded_at` the
    recording's start in every snapshot.
  - The daemon shut down while the bridge ran: no data message after that on any topic,
    and `/diagnostics` went on at 1 Hz with the daemon unreachable.
  - `docker compose stop` took 1.1 s, and the node finished cleanly. On the daemon, the I/O
    watch set was empty within a second, and the motion watch and the `io.watching` flag
    ended within 15 s, when their watch leases lapsed.
- **On a pre-release build of the 0.1.0 daemon, 2026-10-22** (abb-rws-daemon 0.1.0 built
  from source before its last change, schema 0.13, on an earlier abb-rws-client, with
  `--allowed-hosts host.docker.internal`; the bridge image rebuilt from the code of that
  day, with messages from schema 0.13, under Docker Desktop on Windows; nothing was
  written to a controller): RobotWare 7.21.4 and 8.2.1 together, then RobotWare 6.16.03 on
  its own after that virtual controller had been restarted. On all three, the bridge
  connected and published `controller_snapshot` and `joint_states` with the joints the
  controller showed, and `/diagnostics` showed no reconnect, the link live and the
  controller's identity confirmed. In the first run it also showed the daemon live and
  schema 0.13 on both ends; the 8.2.1 snapshot named RobotWare 8.2.1 and a virtual
  controller, and `joint_states` arrived at about 1.8 Hz (J5 at 30 degrees). In both runs
  the node stopped cleanly within 1 s.
- **The released abb-rws-daemon 0.1.0, 2026-10-25** (schema 0.14, which adds whether a
  lock holder is the FlexPendant; the messages come from the same schema; the bridge in
  this repository's image under Docker Desktop on Windows; nothing was written to a
  controller): RobotWare 6.16.03, 7.21.4 and 8.2.1. On all three, the bridge connected and
  published `controller_snapshot` and `joint_states`, and `/diagnostics` showed the daemon
  status OK with schema 0.14 on both ends. In the development image the colcon tests
  passed (2919 tests, 0 failures, 6 skipped), and the generated messages
  matched the vendored schema.
- **ROS 2:** Jazzy on Ubuntu 24.04 only so far. Lyrical and Rolling are in the CI matrix
  and are allowed to fail there; nothing was tried on them locally.
- **Real hardware:** none yet. Every run above was on RobotStudio virtual controllers.

## Development

Everything runs in containers built from `ros:jazzy-ros-base`, with the sources mounted
read-only and without network. From the repository root:

```sh
# The development image: the dependencies of the three packages, no sources.
docker build -f docker/Dockerfile.dev -t abb-rws-ros2-dev:jazzy .

# Build and test the whole tree. Nothing is written into the working tree.
docker run --rm --network none \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PWD":/ws/src/abb-rws-ros2:ro abb-rws-ros2-dev:jazzy bash -c '
    . /opt/ros/jazzy/setup.bash && set -eo pipefail && cd /ws &&
    colcon --log-base /tmp/log build --build-base /tmp/build --install-base /tmp/install \
      --executor sequential &&
    . /tmp/install/setup.bash &&
    colcon --log-base /tmp/log test --build-base /tmp/build --install-base /tmp/install \
      --test-result-base /tmp/build --executor sequential --pytest-args -p no:cacheprovider &&
    colcon test-result --test-result-base /tmp/build --verbose'
```

`colcon test` exits 0 even when a test fails; the last command, `colcon test-result`, is
the one that fails then. It exits 0 on a summary of 0 tests, though, which means that no
test ran: read the summary line.

- Every file has LF line endings (`.gitattributes`); CRLF breaks the scripts inside the
  containers.
- Tests that need a recording are skipped while none is in the tree; see
  [abb_rws_bridge/test/fixtures/README.md](abb_rws_bridge/test/fixtures/README.md).
- Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md). Security reports: see
  [SECURITY.md](SECURITY.md).

## Related projects

- [abb-rws-daemon](https://github.com/ichbinmeraj/abb-rws-daemon): the local daemon this bridge reads
- [abb-rws-client](https://github.com/ichbinmeraj/abb-rws-client): TypeScript RWS client tested on RobotWare 6, 7 and 8
- [abb-rws-vscode](https://github.com/ichbinmeraj/abb-rws-vscode): RAPID Live, VS Code extension for ABB controllers
- abb-rws-conformance: conformance suite and compatibility matrix (planned)
- [abb-rws-mcp](https://github.com/ichbinmeraj/abb-rws-mcp): read-only Model Context Protocol server over the same daemon

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE). If you redistribute this work, keep the NOTICE file.

## Disclaimer

Independent open-source project by [Meraj Safari](https://github.com/ichbinmeraj). ABB is a trademark of ABB Asea Brown Boveri Ltd. RobotWare, OmniCore, IRC5 and RobotStudio are trademarks of their respective owners. Not affiliated with, endorsed by, sponsored by or supported by ABB, ABB Robotics or any later owner of these products. See [TRADEMARKS.md](TRADEMARKS.md).
