# dora-openarm-mujoco

MuJoCo simulation node for the [OpenArm](https://github.com/enactic/openarm_mujoco) bimanual robot, designed to run inside a [dora-rs](https://github.com/dora-rs/dora) dataflow.

It replaces the physical follower arms and cameras: it accepts joint-position commands and publishes arm observations and JPEG camera frames. The optional `--arm-interface openarm` mode exposes the same normalized position, state, and lifecycle interface as the physical OpenArm nodes.

## Installation

```bash
uv sync
```

## Quick start

A self-contained dummy dataflow is included for testing without real hardware.
It wires a dummy leader node to the MuJoCo sim and records to disk.

```bash
uv run dora build dataflow-dummy.yaml --uv
uv run dora run dataflow-dummy.yaml
```

![Example](media/example.png)

## Dataflow configuration

### Minimal (headless and position forwarding only no cameras)

```yaml
- id: openarm-mujoco
  build: pip install -e .
  path: dora-openarm-mujoco
  inputs:
    position_right: leader/follower_position_right
    position_left:  leader/follower_position_left
  outputs:
    - status
    - arm_right_observation
    - arm_left_observation
```

### Full (interactive viewer + all cameras, with contacts, can be used for vr teleoperation)

```yaml
- id: openarm-mujoco
  build: pip install -e .
  path: dora-openarm-mujoco
  args: "--viewer --render --enable-collision --ctrl --keyframe home"
  inputs:
    position_right: leader/follower_position_right
    position_left:  leader/follower_position_left
  outputs:
    - status
    - arm_right_observation
    - arm_left_observation
    - camera_wrist_right
    - camera_wrist_left
    - camera_head_left
    - camera_head_right
    - camera_ceiling
```

### OpenArm-compatible arm interface

The original simulator interface publishes a flat joint observation whenever it
receives a position command. That is convenient for simple simulation, but it
does not match the physical OpenArm nodes used by the evaluation, IK, and
recorder dataflows. Those nodes have an explicit lifecycle, publish sampled
state on request ticks, and distinguish measured state from the most recently
accepted command.

`--arm-interface openarm` adds that contract while keeping the legacy interface
as the default. One MuJoCo node represents both physical arm nodes, so its
side-specific ports use `_right` and `_left` suffixes.

Use it to replace both physical arm nodes in an existing OpenArm dataflow:

```yaml
- id: openarm-mujoco
  build: pip install -e .
  path: dora-openarm-mujoco
  args: "--arm-interface openarm --scene demo"
  inputs:
    request_state: dora/timer/millis/4
    command: ui/arm_command
    position_right: action-mux/move_position_right
    position_left: action-mux/move_position_left
  outputs:
    - position_right
    - position_left
    - state_right
    - state_left
    - latest_command_right
    - latest_command_left
    - status_right
    - status_left
```

The complete evaluation wiring is available as `dataflow-mujoco.yaml` in the
`dora-openarm-evaluation-ui` project.

#### Interface modes

| Behavior | `legacy` (default) | `openarm` |
|----------|--------------------|-----------|
| Initial state | Immediately active | Both arms publish `stopped` |
| Lifecycle input | Not used | `start`, `stop`, and `quit` on `command` |
| State sampling | After each accepted position command | On each `request_state` event while started |
| Position output | Flat `arm_*_observation` array | Canonical `position_*` qpos struct |
| Full state output | None | Canonical `state_*` struct |
| Accepted command output | None | `latest_command_*`, immediately and on later state requests |

The modes are intentionally separate. Existing dataflows continue to receive
the old startup and observation behavior unless they explicitly select
`--arm-interface openarm`.

#### Lifecycle

OpenArm mode starts disabled and handles lifecycle commands as follows:

| Command | Effect |
|---------|--------|
| `start` | Clears cached commands, enables position and state processing, and publishes `started` for both arms. |
| `stop` | Disables position and state processing, clears cached commands, and publishes `stopped` for both arms. |
| `quit` | Performs the `stop` behavior and then terminates the simulator event loop. |

Position commands and `request_state` events received while stopped are
ignored. Clearing cached commands at each lifecycle boundary prevents a target
from the preceding episode from being reported in the next episode.

#### Position command processing

Each `position_right` or `position_left` event is handled in this order:

1. Decode a canonical `qpos` struct, a legacy `new_position` struct, or a flat
   array into an eight-element target: seven arm joints followed by the gripper.
2. Ignore the event unless the selected interface is active and the decoded
   target has exactly eight elements.
3. Under the MuJoCo data lock, write the target using the selected simulation
   mode:
   - Without `--ctrl`, write directly to `data.qpos` and call `mj_forward`.
     This is kinematically exact and takes effect immediately, but does not
     model actuator dynamics.
   - With `--ctrl`, write to `data.ctrl`. The main simulation loop subsequently
     advances actuator dynamics with `mj_step`.
4. In OpenArm mode, capture `executed_timestamp` after the target has been
   written, publish `latest_command_<side>`, and cache the target and metadata.

`latest_command_*` is the accepted target, not measured state. In `--ctrl`
mode it may differ from `state_*.qpos` while the simulated arm is moving toward
the target. The simulator currently performs shape validation but does not
apply the physical driver's safety clamping.

#### State snapshots

A `request_state` event in OpenArm mode samples both arms under one MuJoCo data
lock. For each side it then publishes:

- `position_<side>` containing the sampled `qpos`.
- `state_<side>` containing `qpos`, `qvel`, `qtorque`, `tmos`, and `trotor`.
- `latest_command_<side>` when that side has accepted a command since the most
  recent `start`.

The canonical eight-element state maps joints 1-7 followed by
`finger_joint1`. `qtorque` comes from MuJoCo's generalized actuator force
(`data.qfrc_actuator`). MuJoCo does not model the motor temperature fields used
by the physical driver, so `tmos` and `trotor` are eight zeros. A joint missing
from a custom scene also remains zero in the corresponding state slot.

The cached command is republished unchanged on state ticks. This gives IK and
recording nodes the same "latest accepted target" snapshot behavior as the
physical arm interface. In particular, its timestamps are not replaced by the
`request_state` timestamp.

#### Metadata and timestamp semantics

| Output | Metadata source | Time meaning |
|--------|-----------------|--------------|
| `position_*`, `state_*` | Copied from `request_state` | A `timestamp` supplied by the tick identifies the state sampling request. |
| `status_*` | Copied from the lifecycle command | Identifies the lifecycle transition. |
| `latest_command_*` | Copied from the accepted position command | The source `timestamp` continues to identify the upstream action. |
| `latest_command_*.executed_timestamp` | Added by this node | Wall-clock nanoseconds captured after writing the target to MuJoCo under the data lock. |

`executed_timestamp` records command dispatch into the simulator. It is not an
arrival or settling time. Replayed `latest_command_*` snapshots retain both the
original source timestamp and the original `executed_timestamp`, allowing a
recorder to deduplicate snapshots and align accepted commands with observations.

Camera and VR debug-frame behavior is independent of the selected arm
interface.

## Inputs

| ID | Type | Description |
|----|------|-------------|
| `position_right` | `float32[8]` or qpos struct | Target joint positions for the right arm: joints 1–7 then the gripper. Canonical `qpos` and legacy `new_position` structs are also accepted. |
| `position_left` | `float32[8]` or qpos struct | Same layout and accepted representations as `position_right`. |
| `request_state` | any | Sample and publish both arm states in `--arm-interface openarm` mode. The payload is ignored; metadata is forwarded. |
| `command` | `string[1]` | `start`, `stop`, or `quit` lifecycle command in `--arm-interface openarm` mode. Metadata is forwarded to status outputs. |
| `pose_right` | `float32[7]` | VR controller pose `[x, y, z, qw, qx, qy, qz]`, expressed in the `--origin-frame` frame (default: the scene's `arm_origin` site). Used only with `--debug-frames`. |
| `pose_left` | `float32[7]` | Same for the left controller. |
| `button_x` | `bool[1]` | X button state. Edge-triggered: on press every scene joint on non-arm bodies (freejoint objects plus fixtures like drawers/doors) snaps back to the `--keyframe` pose; with `--randomize-objects` the freejoint objects land at a randomized pose instead. The button must be released to re-arm. |

## Outputs

| ID | Type | Description |
|----|------|-------------|
| `status` | `string["ready"]` | Legacy mode only; published once at startup. |
| `arm_right_observation` | `float32[8]` | Legacy mode only. Joint positions sampled after each accepted right-arm command. |
| `arm_left_observation` | `float32[8]` | Legacy mode only. Joint positions sampled after each accepted left-arm command. |
| `position_right`, `position_left` | `struct<qpos: list<float32>>[1]` | Joint positions sampled in response to `request_state` in OpenArm mode. |
| `state_right`, `state_left` | `struct<qpos, qvel, qtorque, tmos, trotor>[1]` | State sampled in response to `request_state`; unavailable temperature fields are zero. |
| `latest_command_right`, `latest_command_left` | `struct<qpos: list<float32>>[1]` | Targets accepted in OpenArm mode. Source metadata is preserved and `executed_timestamp` records the MuJoCo write time. |
| `status_right`, `status_left` | `string[1]` | OpenArm-mode lifecycle status: `stopped` or `started`. |
| `camera_wrist_right` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_wrist_left` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_head_left` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_head_right` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_ceiling` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |

Camera outputs carry `metadata={"encoding": "jpeg"}`.

## Arguments

Pass these via the `args:` field in the dataflow YAML, or directly on the command line.

| Argument | Default | Description |
|----------|---------|-------------|
| `--xml PATH` | unset | MJCF scene file to load. Overrides `--scene` when set. |
| `--scene NAME` | `cell` | Bundled scene to load when `--xml` is not set. Choices: `cell`, `demo`, `pedestal`, `bimanual`. |
| `--keyframe NAME` | `home` | Keyframe in the MJCF to reset to on startup. |
| `--randomize-objects [RANGE_M]` | off | Randomize freejoint scene objects (e.g. cubes) on startup and on each `button_x` reset: uniform xy offset within ±`RANGE_M` m of the keyframe pose plus a uniform yaw. `z` and articulated fixtures are unchanged. `RANGE_M` defaults to `0.05` when omitted. |
| `--enable-collision` | off | Enable contact/collision detection. Disabled by default to avoid unexpected joint-locking during teleoperation. |
| `--ctrl` | off | Write incoming positions to `data.ctrl` and step the physics (`mj_step`) to simulate actuator control. The default writes directly to `data.qpos` with `mj_forward`. |
| `--arm-interface MODE` | `legacy` | Arm I/O contract. `legacy` preserves the original flat event-driven observations; `openarm` enables lifecycle control and canonical sampled state. |
| `--viewer` | off | Open the interactive MuJoCo viewer window. Requires a display. |
| `--render` | off | Enable offscreen camera rendering and publish JPEG frames. Leave off if cameras are not needed. |
| `--debug-frames` | off | Draw VR controller poses as coloured arrows in the viewer. Only visible with `--viewer`. |
| `--origin-frame NAME` | `arm_origin` | Frame incoming `pose_right`/`pose_left` are relative to; the overlay composes poses with its live world pose and draws the reference axes there. Pass `world` for raw world-frame poses. Missing frame → warning + world fallback. |
| `--origin-frame-type TYPE` | `site` | MuJoCo object type of `--origin-frame`. Choices: `body`, `site`, `geom`. |

### Environment variables

The arguments listed below can also be set via an environment variable named
`DORA_OPENARM_MUJOCO_` + the upper-cased argument name. `--arm-interface` is
currently selected through the CLI only.

| Environment variable | Argument |
|----------------------|----------|
| `DORA_OPENARM_MUJOCO_XML` | `--xml` |
| `DORA_OPENARM_MUJOCO_SCENE` | `--scene` |
| `DORA_OPENARM_MUJOCO_KEYFRAME` | `--keyframe` |
| `DORA_OPENARM_MUJOCO_ENABLE_COLLISION` | `--enable-collision` |
| `DORA_OPENARM_MUJOCO_CTRL` | `--ctrl` |
| `DORA_OPENARM_MUJOCO_VIEWER` | `--viewer` |
| `DORA_OPENARM_MUJOCO_RENDER` | `--render` |
| `DORA_OPENARM_MUJOCO_DEBUG_FRAMES` | `--debug-frames` |
| `DORA_OPENARM_MUJOCO_ORIGIN_FRAME` | `--origin-frame` |
| `DORA_OPENARM_MUJOCO_ORIGIN_FRAME_TYPE` | `--origin-frame-type` |

Boolean flags accept `1`/`0`, `true`/`false`, `yes`/`no` and `on`/`off`.
`DORA_OPENARM_MUJOCO_VIEWER` additionally accepts an FPS number
(e.g. `60`); `true` enables the viewer at the default 30 Hz.

Explicit CLI arguments override the environment. Boolean flags also gain a
`--no-*` form (e.g. `--no-render`, `--no-viewer`) to turn an
environment-enabled option back off. Set the variables via the `env:` field
of the node in the dataflow YAML:

```yaml
- id: openarm-mujoco
  path: dora-openarm-mujoco
  env:
    DORA_OPENARM_MUJOCO_VIEWER: "60"
    DORA_OPENARM_MUJOCO_RENDER: "true"
```

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
