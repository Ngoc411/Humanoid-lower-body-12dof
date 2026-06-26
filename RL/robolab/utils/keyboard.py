# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2025-2026, The RoboLab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


"""Keyboard controller for SE(2) control."""

import weakref
from collections.abc import Callable

import carb
import omni
import torch
from isaaclab.devices.device_base import DeviceBase

from robolab.tasks.direct.base.base_env import BaseEnv


class Keyboard(DeviceBase):

    def __init__(self, env: BaseEnv, lin_vel_step: float = 0.05, ang_vel_step: float = 0.05,
                 push_speed: float = 0.5):
        """Initialize the keyboard layer.

        Args:
            env: The environment to control.
            lin_vel_step: Step size for linear velocity control.
            ang_vel_step: Step size for angular velocity control.
            push_speed: Xung van toc (m/s) moi lan day base (test thang bang).
        """
        self.env = env
        self.lin_vel_step = lin_vel_step
        self.ang_vel_step = ang_vel_step
        self.push_speed = push_speed
        
        # velocity command state
        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel = 0.0
        
        # acquire omniverse interfaces
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        # note: Use weakref on callbacks to ensure that this object can be deleted when its destructor is called
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )
        # bindings for keyboard to command
        self._create_key_bindings()
        # dictionary for additional callbacks
        self._additional_callbacks = dict()
        
        print("[Keyboard] Velocity control initialized:")
        print("  W/S : Forward/Backward  (lin_vel_x +-0.05 m/s)")
        print("  Q/E : Turn left/right   (ang_vel   +-0.05 rad/s)")
        print("  A/D : Strafe left/right (lin_vel_y +-0.05 m/s)")
        print("  X   : Stop all velocities")
        print("  R   : Reset environment")
        print(f"  PUSH (test thang bang, xung {push_speed} m/s):")
        print("  I/K : day truoc/sau   |   J/L : day trai/phai")

    def __del__(self):
        """Release the keyboard interface."""
        self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
        self._keyboard_sub = None

    def __str__(self) -> str:
        """Returns: A string containing the information of joystick."""
        msg = f"Keyboard Controller for ManagerBasedRLEnv: {self.__class__.__name__}\n"
        return msg

    """
    Operations
    """

    def reset(self):
        pass

    def add_callback(self, key: str, func: Callable):
        pass

    def advance(self):
        pass

    """
    Internal helpers.
    """

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Subscriber callback to when kit is updated.

        Reference:
            https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html
        """
        # apply the command when pressed
        if event.type == carb.input.KeyboardEventType.KEY_PRESS or event.type == carb.input.KeyboardEventType.KEY_REPEAT:
            if event.input.name in self._INPUT_KEY_MAPPING:
                key = event.input.name
                
                # Reset environment
                if key == "R":
                    self.env.episode_length_buf = torch.ones_like(self.env.episode_length_buf) * 1e6
                    print("[Keyboard] Environment reset triggered")
                # Forward/Backward (lin_vel_x)
                elif key == "W":
                    self.lin_vel_x += self.lin_vel_step
                    self._update_commands()
                elif key == "S":
                    self.lin_vel_x -= self.lin_vel_step
                    self._update_commands()
                # Turn left/right (ang_vel)
                elif key == "Q":
                    self.ang_vel += self.ang_vel_step
                    self._update_commands()
                elif key == "E":
                    self.ang_vel -= self.ang_vel_step
                    self._update_commands()
                # Strafe left/right (lin_vel_y)
                elif key == "A":
                    self.lin_vel_y += self.lin_vel_step
                    self._update_commands()
                elif key == "D":
                    self.lin_vel_y -= self.lin_vel_step
                    self._update_commands()
                # Stop (zero velocity)
                elif key == "X":
                    self.lin_vel_x = 0.0
                    self.lin_vel_y = 0.0
                    self.ang_vel = 0.0
                    self._update_commands()
                    print("[Keyboard] Stopped - all velocities set to zero")
                # Push base (test thang bang) - xung van toc tuc thoi
                elif key in ("I", "K", "J", "L"):
                    self._apply_push(key)

        # since no error, we are fine :)
        return True

    def _apply_push(self, key):
        """Day base bang xung van toc tuc thoi (test thang bang on-demand)."""
        env = self.env.unwrapped
        robot = env.scene["robot"]
        s = self.push_speed
        dvx = {"I": +s, "K": -s}.get(key, 0.0)   # truoc/sau (world x)
        dvy = {"J": +s, "L": -s}.get(key, 0.0)   # trai/phai (world y)
        root_vel = robot.data.root_vel_w.clone()  # (N, 6): lin(3) + ang(3)
        root_vel[:, 0] += dvx
        root_vel[:, 1] += dvy
        robot.write_root_velocity_to_sim(root_vel)
        print(f"[Keyboard] PUSH  dvx={dvx:+.2f}  dvy={dvy:+.2f} m/s")

    def _update_commands(self):
        """Update the velocity commands in the environment."""
        env = self.env.unwrapped

        if hasattr(env, "command_manager"):
            # ManagerBasedRLEnv (Isaac Lab)
            cmd = env.command_manager.get_command("base_velocity")
            cmd[:, 0] = self.lin_vel_x
            cmd[:, 1] = self.lin_vel_y
            cmd[:, 2] = self.ang_vel
            # Clamp ranges so episode-reset resampling keeps the same value
            term = env.command_manager._terms.get("base_velocity")
            if term is not None and hasattr(term, "cfg") and hasattr(term.cfg, "ranges"):
                term.cfg.ranges.lin_vel_x = (self.lin_vel_x, self.lin_vel_x)
                term.cfg.ranges.lin_vel_y = (self.lin_vel_y, self.lin_vel_y)
                term.cfg.ranges.ang_vel_z = (self.ang_vel, self.ang_vel)
        elif hasattr(env, "command_generator"):
            # Legacy DirectEnv
            env.command_generator.command[:, 0] = self.lin_vel_x
            env.command_generator.command[:, 1] = self.lin_vel_y
            env.command_generator.command[:, 2] = self.ang_vel

        print(f"[Keyboard] Vel: vx={self.lin_vel_x:.2f}, vy={self.lin_vel_y:.2f}, ang={self.ang_vel:.2f}")

    def _create_key_bindings(self):
        """Creates default key binding."""
        self._INPUT_KEY_MAPPING = {
            "W": "forward",
            "S": "backward", 
            "Q": "turn_left",
            "E": "turn_right",
            "A": "strafe_left",
            "D": "strafe_right",
            "X": "stop",
            "R": "reset_envs",
            "I": "push_forward",
            "K": "push_back",
            "J": "push_left",
            "L": "push_right",
        }
