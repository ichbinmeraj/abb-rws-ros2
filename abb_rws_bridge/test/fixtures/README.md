# Test fixtures

Recorded test data for `abb_rws_bridge`: sanitized, frames-only recordings of the
`abb-rws-daemon` stream.

**This folder holds no recording yet.** A recording is controller data. It is added
only after the owner of the controllers has read the sanitized file and approved it
(see [Approval](#approval)). Everything else the tests use is synthetic and written by
hand (`test/frames.py`).

## What a fixture is

One JSON Lines file (`*.jsonl`, LF line endings, one record per line) in the daemon's
recording format, version 1:

- Line 1 is the `header` record: `"format": "abb-rws-recording"`, `"formatVersion": 1`,
  `"content": ["frames"]`, `"sanitized": true`, and `schemaVersion`, the version of the
  snapshot schema its frames were made with.
- Then come `frame` records, `{"kind": "frame", "t": <ms>, "frame": {...}}`. `t` counts
  milliseconds from the start of the recording. `frame` is one message of the daemon's
  `WS /v0/stream`, as a consumer received it: a `snapshot` first, then `delta` frames
  (the changed groups of one controller, each replaced whole) and `removed` frames.
- The last line is the `end` record with the counts of the file.

A fixture holds no other kind of record: no client calls, no state records, no raw I/O
events. Frames alone are enough to replay it.

## Schema version

Fixtures are made with snapshot schema **0.13**, the version vendored in
`abb_rws_ros2_msgs/schema/v0.schema.json`. A fixture's header names the version it was
made with, and the fixture tests accept only the vendored one. When the vendored schema
moves on, the fixtures are recorded again; they are never edited by hand.

## How the tests find them

The `fixtures_dir` fixture (`test/conftest.py`) is this folder, or the folder named by
the environment variable `ABB_RWS_TEST_FIXTURES` when that is set. The fixture tests
read every `*.jsonl` there. With no recording they are skipped and say why.

## How one is made

All of it runs on the machine that reaches the controllers. Nothing below writes to a
controller.

1. Start `abb-rws-daemon` with a recording of frames only, on a port and with a token
   file of its own:

   ```sh
   node dist/cli.js --config <config> --port <port> --token-file <token file> \
     --record --record-label <label> --record-content frames \
     --record-for <seconds> --record-dir <folder>
   ```

2. While it records, hold a motion watch and an I/O watch on every controller
   (`POST /v0/controllers/<id>/motion/watch`, `POST /v0/controllers/<id>/io/watch`,
   renewed before half of `leaseMs`). Without them the `motion` and `io` groups stay
   empty.
3. Stop the daemon with `POST /v0/admin/shutdown` or Ctrl+C. Its log names every
   controller it logged out of.
4. Keep the recording only if every controller's link is connected with quality `live`
   or `polling` from its first good frame to the end, and no group carries an error.
   Otherwise record again.
5. Sanitize it in the daemon repository:

   ```sh
   node scripts/sanitize-recording.mjs <part>.jsonl --out <name>.jsonl
   ```

   The result has frames only. Controller names, system names, system ids, controller
   ids, MAC addresses and hosts other than loopback are placeholders (`controller-1`,
   `system-1`, `sysid-1`, `ctrl-1`, `mac-1`, `host-1`), the same one wherever a value
   occurs. The script refuses to write a file that still holds an address, a machine
   name or a user folder.
6. Check the result once more for what the sanitizer cannot know: the user name and
   password the daemon logged in with, the daemon's access token, the name of the
   machine and of the account that recorded, and any serial or licence number. One
   hit means no fixture.
7. Replay it and read it back:

   ```sh
   node dist/cli.js --replay <name>.jsonl --port <port> --token-file <token file>
   ```

   `GET /v0/health` then says `"mode": "replay"`, and `GET /v0/controllers` answers
   from the recording. A replaying daemon reads no config and reaches no controller.

## What a sanitized recording still holds

Sanitizing replaces identifiers. It does not remove controller data, and it cannot
judge free text. A fixture still shows:

- RobotWare versions and the installed option names;
- RAPID task and module names, program pointers and execution state;
- mechanical unit names, joint values with their units and limits, and the TCP pose;
- the names, types and values of the watched I/O signals;
- event log messages: titles, texts and their arguments, which can name users,
  programs, modules and client applications;
- the controller's start time and event times as its own clock reads them, and the
  offset of that clock from UTC, which shows the time zone the controller runs in;
- the ports the controllers answered on.

## Approval

The owner of the controllers reads the sanitized file, with the lists above in mind,
before it is committed. Until then the file is kept outside this repository's working
tree: a file in the tree is one `git add` from being published.

After approval the file is copied here unchanged and committed. The repository's
`.gitattributes` keeps it LF on every platform.
