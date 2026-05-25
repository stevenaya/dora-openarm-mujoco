# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
dora-openarm-mujoco — MuJoCo simulation node for OpenArm bimanual
========================================================================

This dora node simulates the OpenArm bimanual in MuJoCo.  It replaces
the physical follower arms and cameras in a dataflow, accepting joint-position
commands and publishing arm observations and JPEG camera frames.

Dataflow configuration
----------------------
Minimal (headless, no cameras)::

    - id: openarm-mujoco
      build: pip install -e .
      path: dora-openarm-mujoco
      args: "--viewer ..."
      inputs:
        position_right: leader/follower_position_right
        position_left:  leader/follower_position_left
      outputs:
        - status
        - arm_right_observation
        - arm_left_observation

Inputs
------
position_right / position_left : float32[8] or struct{new_position: float32[8], ...}
    Target joint positions for each arm: joints 1–7 followed by the gripper
    finger joint.  Accepts either a plain float32 array or a StructArray
    with a ``new_position`` field.

pose_right / pose_left : float32[7]
    VR controller pose as [x, y, z, qw, qx, qy, qz].  Only used for the
    ``--debug-frames`` overlay; ignored otherwise.

Outputs
-------
status : string["ready"]
    Published once on startup so downstream nodes know the sim is live.

arm_right_observation / arm_left_observation : float32[8]
    Observed joint positions (same layout as the inputs) published in response
    to each incoming position command.

camera_wrist_right / camera_wrist_left / camera_head_left / camera_head_right / camera_ceiling : uint8[N]
    JPEG-encoded frames at ~30 Hz.  Only published when ``--render`` is set.
    Each output carries ``metadata={"encoding": "jpeg"}``.

CLI arguments (set via ``args:`` in the dataflow YAML)
--------------------------------------------------------
--xml PATH
    MJCF scene file.  Defaults to the bundled openarm_cell scene.

--scene NAME
    Bundled scene to load when --xml is not set.  Choices: {cell, demo, pedestal}.

--keyframe NAME  (default: "home")
    Name of the keyframe in the MJCF to reset to on startup.

--enable-collision
    Enable contact/collision detection.  Disabled by default for speed and
    to avoid unexpected joint-locking during teleoperation.

--ctrl
    Write incoming positions to ``data.ctrl`` and advance the physics
    simulation (``mj_step``).  The default is to write directly to
    ``data.qpos`` (``mj_forward`` only), which is faster and kinematically
    exact but ignores actuator dynamics.

--viewer
    Open the interactive MuJoCo viewer window.  Requires a display.
    Headless by default.

--render
    Enable offscreen camera rendering and publish JPEG frames.  Adds latency;
    leave off if cameras are not needed.

--debug-frames
    Draw the VR controller coordinate frames as coloured arrows in the viewer.
    Only visible when ``--viewer`` is also set.
"""

import argparse
import os
import signal
import sys
import threading
import time
import traceback

import cv2
import dora
import mujoco
import mujoco.viewer
import numpy as np
import openarm_mujoco_v2 as openarm_mujoco
import pyarrow as pa
from openarm_mujoco_v2 import JointResolver

from dora_openarm_mujoco._draw import draw_frame, draw_world_frame

_SCENE_RESOLVERS = {
    "cell": openarm_mujoco.openarm_cell_xml,
    "demo": openarm_mujoco.openarm_demo_xml,
    "pedestal": openarm_mujoco.openarm_pedestal_xml,
}
_DEFAULT_SCENE = "cell"
_VIEWER_FPS = 30
_FRAME_DT = 1.0 / _VIEWER_FPS

# Arm control rate (matches quittable-tick-leader: 2ms = 500Hz)
_ARM_HZ = 500
_ARM_DT = 1.0 / _ARM_HZ

# Camera rendering rate (matches quittable-tick-camera: 33ms ≈ 30Hz)
_CAM_HZ = 30
_CAM_DT = 1.0 / _CAM_HZ
_JPEG_QUALITY = 90

_CAMERAS = [
    "camera_wrist_right",
    "camera_wrist_left",
    "camera_head_left",
    "camera_head_right",
    "camera_ceiling",
]

# Maps dora input IDs to arm sides for position events.
_ARM_INPUT_SIDES = {"position_right": "right", "position_left": "left"}


# ── helpers ────────────────────────────────────────────────────────────────────


def _lock(viewer, fallback: threading.Lock):
    """Return viewer.lock() when the viewer is active, otherwise the fallback lock."""
    if viewer is not None:
        return viewer.lock()
    return fallback


# ── observation extraction ─────────────────────────────────────────────────────


def _get_arm_qpos(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> np.ndarray:
    """Extract current joint positions (7 arm + 1 gripper = 8 elements)."""
    q = np.zeros(8)
    for i in range(1, 8):
        jnt_name = f"openarm_{side}_joint{i}"
        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
        if jnt_id >= 0:
            q[i - 1] = data.qpos[model.jnt_qposadr[jnt_id]]

    grp_name = f"openarm_{side}_finger_joint1"
    jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, grp_name)
    if jnt_id >= 0:
        q[7] = data.qpos[model.jnt_qposadr[jnt_id]]

    return q.astype(np.float32)


# ── offscreen camera rendering ─────────────────────────────────────────────────


class CameraRenderer:
    """Offscreen renderer for MuJoCo cameras. Renders to JPEG bytes."""

    def __init__(self, model: mujoco.MjModel, jpeg_quality: int = 90):
        self.jpeg_quality = jpeg_quality
        self.cam_ids: dict[str, int] = {}
        self.renderers: dict[str, mujoco.Renderer] = {}

        cam_resolutions: dict[str, tuple[int, int]] = {}
        for cam_name in _CAMERAS:
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            if cam_id < 0:
                print(f"[camera] Warning: camera '{cam_name}' not found in model")
                continue
            self.cam_ids[cam_name] = cam_id
            res = model.cam_resolution[cam_id]  # [width, height]
            cam_resolutions[cam_name] = (int(res[0]), int(res[1]))

        if cam_resolutions:
            max_w = max(w for w, _ in cam_resolutions.values())
            max_h = max(h for _, h in cam_resolutions.values())
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, max_w)
            model.vis.global_.offheight = max(model.vis.global_.offheight, max_h)

        for cam_name, (w, h) in cam_resolutions.items():
            try:
                self.renderers[cam_name] = mujoco.Renderer(model, height=h, width=w)
                print(f"[camera] '{cam_name}' renderer: {w}x{h}")
            except Exception as e:
                print(
                    f"[camera] ERROR: could not initialize renderer for '{cam_name}': {e}"
                )

    def render_all(self, data: mujoco.MjData) -> dict[str, bytes]:
        images = {}
        for cam_name, renderer in self.renderers.items():
            renderer.update_scene(data, camera=self.cam_ids[cam_name])
            rgb = renderer.render()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(
                ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if ok:
                images[cam_name] = buf.tobytes()
        return images

    def close(self) -> None:
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()


class CameraScheduler:
    """Throttles camera renders to _CAM_HZ and publishes JPEG frames via dora."""

    def __init__(self, renderer: CameraRenderer, node: dora.Node, data: mujoco.MjData):
        self._renderer = renderer
        self._node = node
        self._data = data
        self._next = time.perf_counter() + _CAM_DT

    def tick(self, lock_fn) -> None:
        if time.perf_counter() < self._next:
            return
        with lock_fn():
            images = self._renderer.render_all(self._data)
        for cam_name, jpeg_bytes in images.items():
            self._node.send_output(
                cam_name,
                pa.array(np.frombuffer(jpeg_bytes, dtype=np.uint8), type=pa.uint8()),
                metadata={"encoding": "jpeg"},
            )
        self._next += _CAM_DT

    def close(self) -> None:
        self._renderer.close()


# ── arm event handler ──────────────────────────────────────────────────────────


def _handle_arm(
    side: str,
    values: np.ndarray,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mapper: JointResolver,
    node: dora.Node,
    viewer,
    data_lock: threading.Lock,
    use_ctrl: bool,
) -> None:
    with _lock(viewer, data_lock):
        if use_ctrl:
            mapper.set_ctrl(data.ctrl, values, side)
        else:
            mapper.set_qpos(data.qpos, values, side)
            mujoco.mj_forward(model, data)
    obs = _get_arm_qpos(model, data, side)
    node.send_output(f"arm_{side}_observation", pa.array(obs, type=pa.float32()))


# ── dora event loop (background thread) ───────────────────────────────────────


def _run_dora(
    node: dora.Node,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mapper: JointResolver,
    viewer,
    data_lock: threading.Lock,
    stop_event: threading.Event,
    use_ctrl: bool = False,
    debug_frames: bool = False,
) -> None:
    print("[dora] Event loop started.")
    pose_right: np.ndarray | None = None
    pose_left: np.ndarray | None = None

    try:
        for event in node:
            if stop_event.is_set():
                break
            if event["type"] != "INPUT":
                continue

            eid = event["id"]

            if eid in _ARM_INPUT_SIDES:
                value = event["value"]
                if isinstance(value, pa.StructArray):
                    value = value.field("new_position")
                    # TODO: We use this for safety check later.
                    # other_arm_position = value.field("other_arm_position")
                values = np.array(value, dtype=np.float32)
                if values.shape == (8,):
                    _handle_arm(
                        _ARM_INPUT_SIDES[eid],
                        values,
                        model,
                        data,
                        mapper,
                        node,
                        viewer,
                        data_lock,
                        use_ctrl,
                    )
            elif eid == "pose_right":
                pose_right = np.array(event["value"], dtype=np.float32)
            elif eid == "pose_left":
                pose_left = np.array(event["value"], dtype=np.float32)

            if viewer is not None and debug_frames:
                with viewer.lock():
                    scn = viewer.user_scn
                    scn.ngeom = 0
                    draw_world_frame(scn)
                    draw_frame(scn, pose_right)
                    draw_frame(scn, pose_left)

    finally:
        print("[dora] Event loop ended – signalling shutdown.")
        stop_event.set()


# ── physics + render loop ──────────────────────────────────────────────────────


def _run_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    lock_fn,  # callable: () → context manager
    stop_event: threading.Event,
    steps_per_frame: int,
    use_ctrl: bool,
    viewer=None,
    cam_scheduler: "CameraScheduler | None" = None,
) -> None:
    """Single physics/sync/camera loop used for both viewer and headless modes."""
    while not stop_event.is_set():
        if viewer is not None and not viewer.is_running():
            break
        t0 = time.perf_counter()

        if use_ctrl:
            with lock_fn():
                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)

        if viewer is not None:
            viewer.sync()

        if cam_scheduler is not None:
            cam_scheduler.tick(lock_fn)

        elapsed = time.perf_counter() - t0
        if elapsed < _FRAME_DT:
            time.sleep(_FRAME_DT - elapsed)


# ── model setup ────────────────────────────────────────────────────────────────


def _setup_model(args) -> tuple[mujoco.MjModel, mujoco.MjData, JointResolver]:
    xml_path = args.xml if args.xml is not None else _SCENE_RESOLVERS[args.scene]()
    print(f"[model] Loading scene: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mapper = JointResolver(model)

    if not args.enable_collision:
        model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        print("[model] Collision (Contact) detection is DISABLED by default.")
    else:
        print("[model] Collision (Contact) detection is ENABLED.")

    cell_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cell_vis")
    if cell_id >= 0:
        model.geom_rgba[cell_id, 3] = 0.2

    if args.keyframe:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)
        else:
            print(
                f"[model] Warning: keyframe '{args.keyframe}' not found, using defaults."
            )

    mujoco.mj_forward(model, data)

    if args.ctrl:
        mapper.set_ctrl(data.ctrl, _get_arm_qpos(model, data, "right"), "right")
        mapper.set_ctrl(data.ctrl, _get_arm_qpos(model, data, "left"), "left")

    return model, data, mapper


# ── argument parsing ───────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Viewer dora node – MuJoCo renderer with camera output for OpenArm"
    )
    p.add_argument(
        "--xml", default=None, help="MJCF scene file. Overrides --scene when set."
    )
    p.add_argument(
        "--scene",
        choices=sorted(_SCENE_RESOLVERS),
        default=_DEFAULT_SCENE,
        help=f"Bundled scene to load when --xml is not set (default: {_DEFAULT_SCENE})",
    )
    p.add_argument(
        "--keyframe", "-k", default="home", help="Initial keyframe name (default: home)"
    )
    p.add_argument(
        "--enable-collision",
        action="store_true",
        help="Enable collision detection (default: disabled)",
    )
    p.add_argument(
        "--ctrl",
        action="store_true",
        help="Write data.ctrl targets and step physics instead of writing data.qpos directly",
    )
    p.add_argument(
        "--viewer",
        action="store_true",
        help="Open the interactive MuJoCo viewer window (default: off)",
    )
    p.add_argument(
        "--render",
        action="store_true",
        help="Enable offscreen camera rendering and publish images (default: off)",
    )
    p.add_argument(
        "--debug-frames",
        action="store_true",
        help="Draw VR controller coordinate frames as overlays in the viewer (default: off)",
    )
    return p.parse_args()


# ── main ───────────────────────────────────────────────────────────────────────


def main() -> None:
    args = _parse_args()

    stop_event = threading.Event()

    def _on_signal(sig, _frame):
        print(f"[main] Received signal {sig}, shutting down.")
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    model, data, mapper = _setup_model(args)
    steps_per_frame = max(1, int(_FRAME_DT / model.opt.timestep))

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    # Bootstrap initial arm observations so the observer can begin ticking.
    for side in ("right", "left"):
        q = _get_arm_qpos(model, data, side)
        node.send_output(f"arm_{side}_observation", pa.array(q, type=pa.float32()))

    cam_scheduler: CameraScheduler | None = None
    if args.render:
        renderer = CameraRenderer(model, _JPEG_QUALITY)
        print(f"[camera] Available cameras: {list(renderer.cam_ids.keys())}")
        cam_scheduler = CameraScheduler(renderer, node, data)

    data_lock = threading.Lock()

    if args.viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.5
            viewer.cam.lookat[:] = [1.3, 0, 0.6]

            dora_thread = threading.Thread(
                target=_run_dora,
                args=(
                    node,
                    model,
                    data,
                    mapper,
                    viewer,
                    data_lock,
                    stop_event,
                    args.ctrl,
                    args.debug_frames,
                ),
                daemon=True,
            )
            dora_thread.start()
            _run_loop(
                model,
                data,
                viewer.lock,
                stop_event,
                steps_per_frame,
                args.ctrl,
                viewer=viewer,
                cam_scheduler=cam_scheduler,
            )

    else:
        print("[main] Running headless (no viewer window).")
        dora_thread = threading.Thread(
            target=_run_dora,
            args=(
                node,
                model,
                data,
                mapper,
                None,
                data_lock,
                stop_event,
                args.ctrl,
                args.debug_frames,
            ),
            daemon=True,
        )
        dora_thread.start()
        _run_loop(
            model,
            data,
            lambda: data_lock,
            stop_event,
            steps_per_frame,
            args.ctrl,
            cam_scheduler=cam_scheduler,
        )

    if cam_scheduler is not None:
        cam_scheduler.close()
    stop_event.set()
    dora_thread.join(timeout=2.0)
    print("[main] Shutdown complete.")


def cli_main() -> None:
    """Console entrypoint.

    MuJoCo/GLFW can segfault during Python interpreter teardown after the viewer
    has already closed cleanly. Exit the process after a successful shutdown so
    Dora observes the real result instead of a native finalizer crash.
    """
    exit_code = 0
    try:
        main()
    except SystemExit as exc:
        if exc.code is None:
            exit_code = 0
        elif isinstance(exc.code, int):
            exit_code = exc.code
        else:
            exit_code = 1
            try:
                print(exc.code, file=sys.stderr)
            except Exception:
                pass
    except KeyboardInterrupt:
        exit_code = 130
        try:
            traceback.print_exc()
        except Exception:
            pass
    except BaseException:
        exit_code = 1
        try:
            traceback.print_exc()
        except Exception:
            pass
    finally:
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except Exception:
                pass
    os._exit(exit_code)


if __name__ == "__main__":
    cli_main()
