#!/usr/bin/env python

# Copyright 2026 NVIDIA Corporation and The HuggingFace Inc. team. All rights reserved.
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

"""XR (VR) controller device for NVIDIA Isaac Teleop, exposed to LeRobot.

A deliberately thin reader: exposes the raw controller grip pose off
``ControllersSource`` (statically rebased into the robot base frame by
``ControllerTransform``), plus squeeze and trigger. No retargeters and no clutch —
the clutch rebasing and gripper mapping live downstream in the owning loop, so this
device is stateless across frames.

``isaacteleop`` imports are guarded behind the availability flag so this module imports
without it (construction fails fast via the base class).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import subprocess
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
import time
from scipy.spatial.transform import Rotation

import sys
from pathlib import Path
ISAAC_TELEOP_PATH = str(Path(__file__).parent.parent) + "/" + str(Path("lerobot/examples/isaac_teleop_to_so101"))
sys.path.append(ISAAC_TELEOP_PATH)

from isaac_teleop.base import IsaacTeleopTeleoperator, _isaacteleop_available
from isaac_teleop.config_isaac_teleop import XRControllerConfig

from lerobot.lerobot_types import RobotAction
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop

from lerobot_teleoperator_devices.keyboard_joint import KeyboardJointTeleop

from .twist_keyboard import TwistKeyboard

if TYPE_CHECKING or _isaacteleop_available:
    from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
    from isaacteleop.retargeting_engine.interface import OutputCombiner, TensorGroup, ValueInput
    from isaacteleop.retargeting_engine.tensor_types import TransformMatrix
    from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex
else:
    ControllersSource = None
    OutputCombiner = None
    TensorGroup = None
    ValueInput = None
    TransformMatrix = None
    ControllerInputIndex = None

# Source-node name for the static base_T_anchor rebase input fed via
# ``TeleopSession.step(external_inputs=...)`` each frame.
_BASE_T_ANCHOR_INPUT = "base_T_anchor"


class XRROSController(IsaacTeleopTeleoperator):
    """Raw XR controller grip-pose teleoperator (base-frame), no retargeters.

    Reads the raw grip pose + squeeze + trigger off a ``ControllersSource`` rebased into
    the robot base frame. :meth:`get_action` returns the absolute base-frame grip pose
    untouched; the owning loop owns the clutch and gripper mapping.
    """

    config_class = XRControllerConfig
    name = "isaac_teleop_controller"

    def __init__(self, config: XRControllerConfig):
        super().__init__(config)
        self.config: XRControllerConfig = config

        # Constant base_T_anchor input, built once in connect() (a TensorGroup is heavy and
        # isaacteleop-backed) and reused every step.
        self._external_inputs: dict[str, Any] | None = None
        # Whether the last get_action() read a tracked controller; the owning loop polls this
        # to wait for the operator to connect before driving the arm.
        self._is_tracking = False
        
        self._last_data = None
        self._last_time = None

    # ------------------------------------------------------------------
    # Pipeline construction
    # ------------------------------------------------------------------

    def _build_pipeline(self) -> OutputCombiner:
        """Build the raw-grip-pose pipeline: a ``ControllersSource`` rebased into the base
        frame by ``ControllerTransform``, exposed verbatim as ``"controller"``. No retargeters.
        """
        side = self.config.hand_side
        controller_key = f"controller_{side}"

        controllers = ControllersSource(name="controllers")
        # Static base_T_anchor rebase fed via external_inputs each step.
        xform = ValueInput(_BASE_T_ANCHOR_INPUT, TransformMatrix())
        transformed = controllers.transformed(xform.output("value"))
        ctrl = transformed.output(controller_key)

        return OutputCombiner({"controller": ctrl})

    def _build_external_inputs(self) -> dict[str, Any]:
        """Materialize the constant ``base_T_anchor`` external input (once, in connect)."""
        tg = TensorGroup(TransformMatrix())
        tg[0] = np.asarray(self.config.base_T_anchor, dtype=np.float32)
        return {_BASE_T_ANCHOR_INPUT: {"value": tg}}

    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate=calibrate)
        try:
            self._external_inputs = self._build_external_inputs()
        except Exception:
            # Roll the session/runtime back so a failed connect() leaves no half-state
            # (a live session behind a raised connect would leak the CloudXR runtime).
            self.disconnect()
            raise

    # ------------------------------------------------------------------
    # Action features
    # ------------------------------------------------------------------
    @property
    def action_features(self) -> dict:
        return {
            "linear_x.vel": float,
            "linear_y.vel": float,
            "linear_z.vel": float,
            "angular_x.vel": float,
            "angular_y.vel": float,
            "angular_z.vel": float,
            "gripper.pos": float,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_tracking(self) -> bool:
        """Whether the last :meth:`get_action` read a tracked controller. ``False`` until the
        headset is connected over CloudXR and its controllers are live; the owning loop polls
        it to wait for the operator before commanding the arm."""
        return self._is_tracking

    # ------------------------------------------------------------------
    # Action extraction
    # ------------------------------------------------------------------

    def get_action(self) -> RobotAction:
        """Step the session and return the raw base-frame grip pose.

        Reads the grip pose + squeeze + trigger off the transformed controller stream (with
        the constant ``base_T_anchor`` rebase). When the controller is not tracked, returns
        identity pose and squeeze/trigger = 0.0 so the owning loop freezes the arm.

        Returns:
            ``{"grip_pos": (3,) [m], "grip_quat": (4,) [qx,qy,qz,qw], "squeeze": float,
            "trigger": float}`` — pose in the robot base frame; squeeze/trigger in ``[0, 1]``.
        """
        result = self._step(execution_events=self._running_events(), external_inputs=self._external_inputs)

        # Optional controller group is None until the headset is connected and its controllers
        # are live; expose that as is_tracking so the loop can wait before driving the arm.
        controller = result["controller"]
        grip_pos = np.zeros(3, dtype=np.float32)
        grip_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        squeeze = 0.0
        trigger = 0.0
        self._is_tracking = not getattr(controller, "is_none", False)
        if self._is_tracking:
            # Read ALL four fields into locals before committing any of them: a failure on a
            # partially-populated frame must not mix live values with the safe defaults (a
            # live squeeze paired with a defaulted trigger=0.0 would keep the clutch engaged
            # while commanding the gripper fully open, dropping whatever is grasped). On
            # failure the defaults stand untouched and the frame reports not-tracked.
            try:
                pos = np.asarray(controller[ControllerInputIndex.GRIP_POSITION], dtype=np.float32)
                quat = np.asarray(controller[ControllerInputIndex.GRIP_ORIENTATION], dtype=np.float32)
                squeeze_val = float(controller[ControllerInputIndex.SQUEEZE_VALUE])
                trigger_val = float(controller[ControllerInputIndex.TRIGGER_VALUE])
            except (IndexError, KeyError, TypeError, ValueError):
                self._is_tracking = False
            else:
                grip_pos, grip_quat = pos, quat
                squeeze, trigger = squeeze_val, trigger_val
        
        cur_time = time.perf_counter()
        cur_data = {
            "grip_pos": grip_pos,
            "grip_quat": grip_quat,
            "squeeze": squeeze,
            "trigger": trigger,
        }
        action = self.pose_to_cartesian(cur_data, cur_time)
        self._last_time = cur_time
        self._last_data = cur_data
        
        return action
    
    def pose_to_cartesian(self, data, time):
        action = {
            "linear_x.vel": 0.0,
            "linear_y.vel": 0.0,
            "linear_z.vel": 0.0,
            "angular_x.vel": 0.0,
            "angular_y.vel": 0.0,
            "angular_z.vel": 0.0,
            "gripper.pos": 0.0,
        }
        
        if self._last_time is not None and self._last_data is not None:
            delta_time = time - self._last_time
            delta_pos = data["grip_pos"] - self._last_data["grip_pos"]
            
            linear_vel = delta_pos / delta_time
            
            if np.sum(self._last_data["grip_quat"]) == 0 or np.sum(data["grip_quat"]) == 0:
                return action
            
            last_r = Rotation.from_quat(self._last_data["grip_quat"])
            cur_r = Rotation.from_quat(data["grip_quat"])
            
            angular_vel = (cur_r * last_r.inv()).as_rotvec() / delta_time
            
            action["linear_x.vel"], action["linear_y.vel"], action["linear_z.vel"] = linear_vel
            action["angular_x.vel"], action["angular_y.vel"], action["angular_z.vel"] = angular_vel
            action["gripper.pos"] = data["trigger"]
        
        return action


class CRXTwistKeyboardTeleop(KeyboardTeleop):
    config_class = KeyboardTeleopConfig
    name = "teleop_twist_keyboard"

    def __init__(self, config: KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type
        
        self.process = None
        self.cmd_vel_sub = None
        self.executor = None
        self.spin_thread = None
    
    @property
    def action_features(self) -> dict:
        return {
            "linear_x.vel": float,
            "linear_y.vel": float,
            "linear_z.vel": float,
            "angular_x.vel": float,
            "angular_y.vel": float,
            "angular_z.vel": float,
            "gripper.pos": float,
        }

    @property
    def is_connected(self) -> bool:
        return self.process is not None and self.process.poll() is None and self.cmd_vel_sub is not None

    def connect(self, calibrate: bool = True) -> None:
        
        if self.process is not None and self.process.poll() is None:
            print("Keyboard teleop is already running.")
            return

        command = [
            "ros2", "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            "--ros-args",
            "-p", "stamped:=true",
            "-p", "frame_id:=base_link",
            "-p", "speed:=0.05", 
            "-p", "turn:=0.1", 
            "-r", "cmd_vel:=/twist_keyboard/cmd_vel",
        ]
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        if not rclpy.ok():
            rclpy.init()
        
        self.cmd_vel_sub = TwistKeyboard()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.cmd_vel_sub)
        
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
            
        print("Keyboard teleop started.")
        
    def get_action(self) -> RobotAction:
        action = {
            "linear_x.vel": 0.0,
            "linear_y.vel": 0.0,
            "linear_z.vel": 0.0,
            "angular_x.vel": 0.0,
            "angular_y.vel": 0.0,
            "angular_z.vel": 0.0,
            "gripper.pos": 0.0,
            }
        twist_data = self.cmd_vel_sub.twist_data
        gripper_data = self.cmd_vel_sub.grip_data
        
        if twist_data is not None:
            action["linear_x.vel"] = twist_data.linear.x
            action["linear_y.vel"] = twist_data.linear.y
            action["linear_z.vel"] = twist_data.linear.z
            action["angular_x.vel"] = twist_data.angular.x
            action["angular_y.vel"] = twist_data.angular.y
            action["angular_z.vel"] = twist_data.angular.z
        
        if gripper_data is not None:
            action["gripper.pos"] = gripper_data[0]

        return action

    def disconnect(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

        self.process = None
        
        if self.executor is not None:
            self.executor.shutdown()

        if hasattr(self, "executor_thread") and self.executor_thread is not None:
            self.executor_thread.join(timeout=2.0)
            self.executor_thread = None

        if self.cmd_vel_sub is not None:
            self.cmd_vel_sub.destroy_node()
            self.cmd_vel_sub = None

        self.executor = None
        
        print("Keyboard teleop stopped.")


class CRXKeyboardJointTeleop(KeyboardJointTeleop):
    pass