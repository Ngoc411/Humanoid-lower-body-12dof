# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg, ImplicitActuatorCfg, DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##


def _env_float(name: str, default: float | None) -> float | None:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)

CHAOS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="D:/IsaacSim/test7.usd",

        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
            disable_gravity=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,  # was 4 — biped contact needs more iterations
            solver_velocity_iteration_count=1,
            # fix_root_link=True,  # <-- thêm dòng này
        ),
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        # IMPORTANT: base_link origin is at the robot's FOOT PLATE level
        # (SolidWorks URDF export convention).
        # ankle_roll joint = 0.0385 m above base_link origin.
        # Foot sole is ~0.015-0.020 m below ankle_roll center.
        # → spawn z ≈ 0.01-0.02 m puts sole just above ground.
        #
        # Calibration: spawn at z=0.02, pause sim, check feet contact.
        # Increase if feet sink into ground, decrease if feet float.
        pos=(0.0, 0.0, 0.5852),
        # pos=(0.0, 0.0, 0.7),
        joint_pos={
            # Slight knee bend to avoid singular straight-leg initialization.
            #
            # Axis conventions from URDF (critical for correct signs):
            #   right_hip_pitch axis=(0 -1 0): NEGATIVE angle = forward lean
            #   left_hip_pitch  axis=(0  1 0): POSITIVE angle = forward lean
            #   right_knee      axis=(0  1 0): range [0, +2.09]  → POSITIVE bends
            #   left_knee       axis=(0 -1 0): range [-2.09, 0]  → NEGATIVE bends
            #
            # 'right_hip_pitch_joint': 0.2340196818113327, # 10°
            # 'right_hip_roll_joint': -0.09753597527742386,
            # 'right_hip_yaw_joint': 0.0,
            # 'right_knee_joint': -0.35654494166374207,  # 20°
            # 'right_ankle_pitch_joint': 0.03525879234075546,  # 10° — verify: should compensate knee bend (plantar-flex)
            
            # 'left_hip_pitch_joint': -0.18228009343147278,  # 10°
            # 'left_hip_roll_joint': -0.03368520364165306,
            # 'left_hip_yaw_joint': -0.0,
            # 'left_knee_joint': -0.2739838659763336,           # 20°
            # 'left_ankle_pitch_joint': 0.3689975142478943,  # -10° — axis is mirrored vs right: negative = plantar flex
                                                # Previously +0.1745 caused asymmetric ground contact at spawn
    #   [0.031530071049928665, 0.0, 0.2340196818113327, -0.09753597527742386, -0.0, -0.35654494166374207, 0.03525879234075546, -0.0, 
    #    0.0, -0.18228009343147278, -0.03368520364165306, -0.0, -0.2739838659763336, 0.3689975142478943, -0.0]


            'right_hip_pitch_joint': 0.1, # 5°
            'right_knee_joint': 0.2, # 10°
            'right_ankle_pitch_joint': 0.1,  # 5° — verify: should compensate knee bend (plantar-flex)
            'left_hip_pitch_joint': 0.1,  # 5°
            'left_knee_joint': 0.2,       # 10°
            'left_ankle_pitch_joint': 0.1, # 5° — axis is mirrored vs right: negative = plantar flex
                                                # Previously +0.1745 caused asymmetric ground contact at spawn

        },
        joint_vel={".*": 0.0},
    ),
    
    # 0.9 gives more usable ankle range than 0.95.
    # ankle URDF ±15° * 0.9 = ±13.5° — still tight but avoids hard-limit penalty
    # during normal gait. Legs use same factor (knee ±108° is generous).
    soft_joint_pos_limit_factor=0.9,
    collision_group=-1,
    actuators={ # Position control
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_pitch_joint"],

            # --- Torque limits --- (motor thuc te = 10 Nm, co dinh)
            effort_limit=_env_float("CHAOS_HIP_PITCH_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_HIP_PITCH_EFFORT_LIMIT_SIM", 10.0),

            # --- Velocity limits ---
            velocity_limit=_env_float("CHAOS_HIP_PITCH_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_HIP_PITCH_VELOCITY_LIMIT_SIM", 14.653),

            # --- PD gains --- (sysID theo right_hip_pitch_joint MuJoCo: posRMSE~0.84deg)
            # link nang + gravity => 45deg cham tran 1 step dau (trong ngan sach 1-4).
            # MuJoCo rise overdamped (droop ~18% vi gravity) -> kd cao fit rise tot nhat.
            stiffness=_env_float("CHAOS_HIP_PITCH_STIFFNESS", 18.0),
            damping=_env_float("CHAOS_HIP_PITCH_DAMPING", 3.0),

            # --- Inertia --- (arm thap de giu omega_n & ha torque dinh)
            armature=_env_float("CHAOS_HIP_PITCH_ARMATURE", 0.02),
            friction=_env_float("CHAOS_HIP_PITCH_FRICTION", None),
        ),
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_knee_joint",
            ],
            # motor thuc te = 10 Nm, co dinh
            effort_limit=_env_float("CHAOS_KNEE_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_KNEE_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_KNEE_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_KNEE_VELOCITY_LIMIT_SIM", 14.653),
            # sysID theo right_knee_pitch_joint MuJoCo, UU TIEN khop VEL MAX + vel curve:
            #   vpeak 8.55@40ms ~ ref 8.70  (25deg: 5.0 ~ ref 4.85), OS~7.5% ~ ref 6.6%.
            #   posRMSE~0.83 velRMSE~0.28. Cham tran 2 step dau (ngan sach 1-4).
            # kp35+kd1.75+arm0.046: kp cao giu bao hoa lau de van toc len cao, kd cao phanh
            # han che vot lo. Ref dat 8.7rad/s nho 71Nm; ta gioi han 10Nm nen vpeak ~8.55 la can.
            stiffness=_env_float("CHAOS_KNEE_STIFFNESS", 35.0),
            damping=_env_float("CHAOS_KNEE_DAMPING", 1.75),
            armature=_env_float("CHAOS_KNEE_ARMATURE", 0.046),
            friction=_env_float("CHAOS_KNEE_FRICTION", None),
        ),
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            # motor thuc te = 10 Nm, co dinh
            effort_limit=_env_float("CHAOS_HIP_ROLL_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_HIP_ROLL_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_HIP_ROLL_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_HIP_ROLL_VELOCITY_LIMIT_SIM", 14.653),
            # sysID theo right_hip_roll_joint MuJoCo: posRMSE~0.62deg, velRMSE~0.09.
            # arm=0.02, kd=2.6 -> OS~4% (MuJoCo 3.3%). 45deg cham tran 1 step dau.
            stiffness=_env_float("CHAOS_HIP_ROLL_STIFFNESS", 17.0),
            damping=_env_float("CHAOS_HIP_ROLL_DAMPING", 2.6),
            armature=_env_float("CHAOS_HIP_ROLL_ARMATURE", 0.02),
            friction=_env_float("CHAOS_HIP_ROLL_FRICTION", None),
        ),
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            # set y het hip_roll (khong co data MuJoCo rieng cho hip_yaw -> mirror hip_roll)
            effort_limit=_env_float("CHAOS_HIP_YAW_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_HIP_YAW_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_HIP_YAW_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_HIP_YAW_VELOCITY_LIMIT_SIM", 14.653),
            stiffness=_env_float("CHAOS_HIP_YAW_STIFFNESS", 17.0),
            damping=_env_float("CHAOS_HIP_YAW_DAMPING", 2.6),
            armature=_env_float("CHAOS_HIP_YAW_ARMATURE", 0.02),
            friction=_env_float("CHAOS_HIP_YAW_FRICTION", None),
        ),

        # --- ANKLE PITCH --- (ImplicitActuatorCfg, fit theo right_ankle_pitch_joint MuJoCo)
        # effort_limit NANG 6->10 Nm. De GIU NGUYEN pos/vel curve nhung torque tuc thi cao hon,
        # nhan kp/kd/armature cung he so alpha=10/6=1.667: omega_n & zeta khong doi -> curve y het,
        # gia toc khi bao hoa cung khong doi (10/(1.667*I) = 6/I) -> quy dao trung khop. Torque
        # cot Nm gio len toi ~10 thay vi ~6 (gravity ankle ~0 nen droop khong doi).
        "ankle_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint"],
            effort_limit=_env_float("CHAOS_ANKLE_PITCH_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_ANKLE_PITCH_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_ANKLE_PITCH_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_ANKLE_PITCH_VELOCITY_LIMIT_SIM", 14.653),
            stiffness=_env_float("CHAOS_ANKLE_PITCH_STIFFNESS", 33.33),
            damping=_env_float("CHAOS_ANKLE_PITCH_DAMPING", 1.667),
            armature=_env_float("CHAOS_ANKLE_PITCH_ARMATURE", 0.01667),
            friction=_env_float("CHAOS_ANKLE_PITCH_FRICTION", None),
        ),
        # --- ANKLE ROLL --- (ImplicitActuatorCfg, fit theo right_ankle_roll_joint MuJoCo)
        # effort_limit NANG 6->10 Nm, scale kp/kd/armature x1.667 nhu ankle_pitch -> giu y curve.
        "ankle_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_roll_joint"],
            effort_limit=_env_float("CHAOS_ANKLE_ROLL_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_ANKLE_ROLL_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_ANKLE_ROLL_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_ANKLE_ROLL_VELOCITY_LIMIT_SIM", 14.653),
            stiffness=_env_float("CHAOS_ANKLE_ROLL_STIFFNESS", 33.33),
            damping=_env_float("CHAOS_ANKLE_ROLL_DAMPING", 1.667),
            armature=_env_float("CHAOS_ANKLE_ROLL_ARMATURE", 0.01667),
            friction=_env_float("CHAOS_ANKLE_ROLL_FRICTION", None),
        ),
    },
    # prim_path="{ENV_REGEX_NS}/assem_symplified_3"
)

# Fixed-base variant — chỉ dùng cho step response / actuator identification.
# USD này có fix_base=True baked in, không dùng cho training.
CHAOS_FIXED_CFG = CHAOS_CFG.replace(
    spawn=CHAOS_CFG.spawn.replace(
        usd_path="D:/IsaacSim/fixed_base.usd",
        # Step-response bench can fixed-base (gravity-on) de khop dieu kien MuJoCo.
        # Doc lap voi CHAOS_CFG (training de free-base).
        articulation_props=CHAOS_CFG.spawn.articulation_props.replace(fix_root_link=True),
    ),
    # Bench reset moi khop ve 0 -> dung init_pos=0 (hop le voi moi limit). KHONG
    # dung joint_pos default cua CHAOS_CFG (de tranh crash out-of-limits khi bench).
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.7), joint_vel={".*": 0.0}),
)
"""
τ_applied = clip(Kp*(q_target - q_current) + Kd*(0 - q_dot_current), -τ_limit, +τ_limit)

Kp = stiffness (độ cứng)
Kd = damping (giảm chấn)
q_target = target position từ policy
q_current = current position
q_dot_current = current velocity
τ_limit = torque limit

from pxr import UsdPhysics, PhysxSchema
context = omni.usd.get_context()
self._stage = context.get_stage()
robot_usd_prims = self._stage.GetPrimAtPath("/World/Robot/tool")
UsdPhysics.CollisionAPI.Apply(robot_usd_prims)
UsdPhysics.MeshCollisionAPI.Apply(robot_usd_prims)
physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(robot_usd_prims)
PhysxSchema.PhysxContactReportAPI.Apply(robot_usd_prims)

"""
