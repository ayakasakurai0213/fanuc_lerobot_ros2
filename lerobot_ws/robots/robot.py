from lerobot_robot_ros.robot import ROS2Robot
from lerobot_robot_ros.config import ROS2Config, ActionType
from .ros_interface import ROS2Interface

from sensor_msgs.msg import JointState

import numpy as np
import time
from functools import cached_property
from typing import Any
import logging
logger = logging.getLogger(__name__)


class CRXROS(ROS2Robot):
    def __init__(self, config: ROS2Config):
        super().__init__(config)
        self.ros2_interface = ROS2Interface(config.ros2_interface, config.action_type, config.use_isaac)
        
        self.sim_cameras = {}
        self.sim_camera_config = {"camera_names": ["top_image", "hand_image"], "height": 480, "width": 640}
        
    
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        if self.config.use_isaac:
            return {
                cam: (self.sim_camera_config["height"], self.sim_camera_config["width"], 3) for cam in self.sim_camera_config["camera_names"]
            }
        else:
            return {
                cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
            }

    
    @cached_property
    def action_features(self) -> dict[str, type]:
        if self.config.action_type == ActionType.CARTESIAN_VELOCITY:
            return {
                "J1.pos": float,
                "J2.pos": float,
                "J3.pos": float,
                "J4.pos": float,
                "J5.pos": float,
                "J6.pos": float,
                "finger_joint.pos": float,
            }
        elif self.config.action_type in (ActionType.JOINT_POSITION, ActionType.JOINT_TRAJECTORY):
            return {f"{joint}.pos": float for joint in self.config.ros2_interface.arm_joint_names} | {
                "finger_joint.pos": float
            }
        else:
            raise ValueError(f"Unsupported action type: {self.config.action_type}")
        
            
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}
        if "xr" in self.config.id: 
            joint_state = self.ros2_interface.obs_state
        else:
            joint_state = self.ros2_interface.joint_state
        if joint_state is None:
            raise ValueError("Joint state is not available yet.")
        obs_dict.update({f"{joint}.pos": pos for joint, pos in joint_state["position"].items()})

        # Capture images from cameras
        if self.config.use_isaac:
            if self.ros2_interface.camera_node is not None:
                self.sim_cameras = self.ros2_interface.camera_node.cameras
            for cam_key, cam in self.sim_cameras.items():
                start = time.perf_counter()
                obs_dict[cam_key] = cam
                
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
                
        else: 
            for cam_key, cam in self.cameras.items():
                start = time.perf_counter()
                try:
                    obs_dict[cam_key] = cam.async_read(timeout_ms=300)
                except Exception as e:
                    logger.error(f"Failed to read camera {cam_key}: {e}")
                    obs_dict[cam_key] = None
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict
    
    
    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.action_type == ActionType.CARTESIAN_VELOCITY:
            if self.config.max_relative_target is not None:
                # We don't have the current velocity of the arm, so set it to 0.0
                # Effectively the goal velocity gets clipped by max_relative_target
                goal_present_vel = {key: (act, 0.0) for key, act in action.items()}
                action = ensure_safe_goal_position(goal_present_vel, self.config.max_relative_target)

            linear_vel = (
                action["linear_x.vel"],
                action["linear_y.vel"],
                action["linear_z.vel"],
            )
            angular_vel = (
                action["angular_x.vel"],
                action["angular_y.vel"],
                action["angular_z.vel"],
            )
            self.ros2_interface.servo(linear=linear_vel, angular=angular_vel)
        elif self.config.action_type in (ActionType.JOINT_POSITION, ActionType.JOINT_TRAJECTORY):
            if self.config.max_relative_target is not None:
                goal_present_pos = {}
                joint_state = self.ros2_interface.joint_state
                if joint_state is None:
                    raise ValueError("Joint state is not available yet.")

                for key, goal in action.items():
                    present_pos = joint_state["position"].get(key.replace(".pos", ""), 0.0)
                    goal_present_pos[key] = (goal, present_pos)
                action = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

            joint_positions = [action[joint + ".pos"] for joint in self.config.ros2_interface.arm_joint_names]
            
            if self.config.use_isaac:
                gripper_pos = action["gripper.pos"]
                self.ros2_interface.send_isaac_joint_position_command(joint_positions, gripper_pos)
            else:
                self.ros2_interface.send_joint_position_command(joint_positions)

        gripper_pos = action["gripper.pos"]
        self.ros2_interface.send_gripper_command(gripper_pos)    
        return action
    
    
    def init_pos(self):
        command = JointState()
        command.name = [i for i in self.ros2_interface._last_joint_state["position"].keys()]
        
        init_pos = np.array([0.0, 0.0, 0.0, 0.0, -1.75, 0.0, 0.0])
        command.position = init_pos
        
        while True:
            last_pos = [i for i in self.ros2_interface._last_joint_state["position"].values()]
            self.ros2_interface.joint_state_pub.publish(command)
            
            if np.allclose(init_pos, np.array(last_pos), atol=0.1):
                print("Initializing Position Completely")
                break