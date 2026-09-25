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
            'right_hip_pitch_joint': 0.1, # 5°
            'right_knee_joint': 0.2, # 10°
            'right_ankle_pitch_joint': 0.1,  # 5° — verify: should compensate knee bend (plantar-flex)
            'left_hip_pitch_joint': 0.1,  # 5°
            'left_knee_joint': 0.2,       # 10°
            'left_ankle_pitch_joint': 0.1, # 5° — axis is mirrored vs right: negative = plantar flex
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

            # --- PD gains ---
            # RE-TUNE 2026-07-03: kp/kd doi thanh 45/7.5 (dong bo voi knee). Tune lai
            # armature/friction/viscous_friction vs D:\test_sim\csv\test_sim\test_sim_1.csv
            # (right_hip_pitch_joint, 18 targets +-5..+-45deg, symmetric range [-1.745,1.745]).
            # Grid search: armature=0 la toi uu ro rang (loss tang deu khi tang armature -
            # kp/kd nay da du damped, khong can armature bu them). friction it anh huong
            # (gan flat tren khoang 0.6-1.2), chon 1.0. Da thu ket hop ca 3 tham so (grid
            # joint) - khong tot hon fit rieng le, xac nhan armature=0 la dung. Scan rieng
            # viscous_friction 0->2.5 (full resolution) cho ra cuc tieu THAT (khong phai o
            # bien) tai 0.7: posRMSE=0.75deg velRMSE=1.31rpm, overshoot=0 (joint nay von da
            # khong overshoot du co viscous_friction hay khong, kp/kd du damped).
            #
            # RE-TUNE 2026-07-07 (wave-motion): fit tren (V ngay tren) chi dung step-response
            # 1 chieu, khong bat duoc dong luc hoc dao chieu lien tuc. Fit lai vs wave test
            # D:\test_sim\csv\test_sim\test_sim_130.csv (0<->30deg, 5 chu ky/10 lan doi target,
            # nguong 85%). QUAN TRONG VE DAU: CSV luu target = -30deg (KHONG PHAI +30) vi
            # truc hip_pitch cua robot that NGUOC CHIEU voi truc trong sim -> dung --sign -1.0
            # (sim_target = sign * csv_raw_target = (-1)*(-30) = +30 trong khung sim). Baseline
            # (armature=0, friction=1.0, viscous_friction=0.7 tu V truoc) cho period diff=
            # +0.0163s/chu ky (sim CHAM hon real) -> giam viscous_friction de sim nhanh len.
            # friction van khong anh huong ro ret (cac trial cung armature/viscfr nhung khac
            # friction cho ket qua gan nhu giong het). Scan viscous_friction tim duoc cuc tieu
            # THAT tai 0.65 (khong phai bien): posRMSE=1.24deg velRMSE=4.19rpm, period
            # diff=+0.0062s/chu ky (giam ~2.6 lan so voi truoc).
            stiffness=_env_float("CHAOS_HIP_PITCH_STIFFNESS", 45.0),
            damping=_env_float("CHAOS_HIP_PITCH_DAMPING", 7.5),

            # --- Inertia --- (arm thap de giu omega_n & ha torque dinh)
            armature=_env_float("CHAOS_HIP_PITCH_ARMATURE", 0.0),
            friction=_env_float("CHAOS_HIP_PITCH_FRICTION", 0.8),
            viscous_friction=_env_float("CHAOS_HIP_PITCH_VISCOUS_FRICTION", 0.65),
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
            # RE-TUNE 2026-07-02 (v3): du lieu D:\test_sim\csv\test_sim\test_sim_7.csv duoc
            # thay moi (9 targets -5..-45deg, them cot tau_ref, khac han ban 12-target -5..-60
            # cu). Giu nguyen kp=45/kd=2.25. Grid search lai tu dau (200Hz, fixed-base):
            #   armature=0.0425 (khong doi so voi ban cu) nhung friction toi uu doi thanh
            #   0.20 (truoc la 0.09) -> posRMSE=0.70deg velRMSE=2.14rpm nhung van con
            #   overshoot (~1.0deg). Scan viscous_friction 0->1.0: vung overshoot=0 bat dau
            #   tu 0.35, RMSE tiep tuc giam toi tan 0.75 (khac ban truoc, optimum o day CAO
            #   hon nhieu - do data moi khac). Fine scan chon 0.75 (RMSE thap nhat trong vung
            #   overshoot=0). Fit cuoi: posRMSE=0.57deg velRMSE=1.91rpm, overshoot=0.
            #
            # RE-TUNE 2026-07-07 (wave-motion): fit tren o V3 chi dung step-response 1 chieu
            # (di toi target roi settle), KHONG bat duoc dong luc hoc luc dao chieu lien tuc.
            # Phat hien qua step_response_sim.py --wave 40 (10 chu ky 0<->40deg): sim chay
            # NHANH hon real ~6.1%/chu ky (period sim=0.280s vs real=0.2983s tu
            # D:\test_sim\csv\test_sim\test_sim_6.csv), le ch dong tich tuyen tinh qua 10 chu
            # ky thanh lech pha ro ret. Viet script fit_knee_wave.py: mo phong dung logic
            # doi target 90%/10% nhu wave test, so RMSE + period voi test_sim_6.csv. Grid search
            # rieng cho wave-motion nay co dang "gai nhon" (khong muot) - vi RMSE tong the qua
            # nhieu chu ky de bi trung hop boi phase-cancellation ngau nhien, nen uu tien diem
            # co |period diff| nho nhat (khong chi RMSE thap nhat). armature=0.02 (giam tu
            # 0.0425), viscous_friction=0.70 (tang tu 0.35) cho period diff=+0.0017s/chu ky
            # (giam ~10.7 lan so voi -0.0183s/chu ky ban dau) va posRMSE=2.37deg velRMSE=9.40rpm
            # (giam manh so voi 17.79deg/63.48rpm truoc do). friction khong anh huong do (giu
            # nguyen 0.20). LUU Y: gia tri nay uu tien khop wave-motion (dao chieu lien tuc);
            # neu can lai step-response 1 chieu thuan tuy, xem lai V3 o tren.
            stiffness=_env_float("CHAOS_KNEE_STIFFNESS", 45),
            damping=_env_float("CHAOS_KNEE_DAMPING", 2.25),
            armature=_env_float("CHAOS_KNEE_ARMATURE", 0.02),
            friction=_env_float("CHAOS_KNEE_FRICTION", 0.20),
            viscous_friction=_env_float("CHAOS_KNEE_VISCOUS_FRICTION", 0.70),
        ),
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            # motor thuc te = 10 Nm, co dinh
            effort_limit=_env_float("CHAOS_HIP_ROLL_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_HIP_ROLL_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_HIP_ROLL_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_HIP_ROLL_VELOCITY_LIMIT_SIM", 14.653),
            # RE-TUNE 2026-07-03: kp/kd doi thanh 45/7.5 (dong bo voi knee/hip_pitch). Tune
            # armature/friction/viscous_friction vs D:\test_sim\csv\test_sim\test_sim_3.csv
            # (right_hip_roll_joint, sim range [-0.262,1.571]rad=[-15,+90]deg). QUAN TRONG:
            # target >=15deg trong CSV (15,20,25,30) deu settle ve CUNG mot gia tri ~13deg
            # tren robot that - day la robot that bi CLAMP boi gioi han co khi that o ~13-15deg
            # (KHONG duoc mo phong trong USD joint limit cua sim) chu KHONG PHAI loi dynamics -
            # da LOAI CAC TARGET NAY khoi tap fit, chi dung {5,10,-5,-10}. armature=0 toi uu
            # (giong hip_pitch/knee, kp/kd da du damped). friction=0.36, viscous_friction=3.5
            # (cuc tieu that, loss tang tro lai o 5.0-6.0). Fit cuoi (4 targets hop le):
            # posRMSE=0.66deg velRMSE=0.52rpm.
            #
            # RE-TUNE 2026-07-07 (wave-motion): fit tren (V ngay tren) chi dung step-response
            # 1 chieu, khong bat duoc dong luc hoc dao chieu lien tuc. Fit lai vs wave test
            # D:\test_sim\csv\test_sim\test_sim_315.csv (0<->15deg, 5 chu ky/10 lan doi target,
            # nguong 85%, sign=+1 - target CSV da la +15deg, KHONG can dao dau nhu hip_pitch).
            # Baseline (armature=0, friction=0.36, viscous_friction=3.5 tu V truoc) cho period
            # diff RAT LON: +0.305s/chu ky (sim CHAM hon real ~48%) - viscous_friction=3.5 qua
            # cao cho chuyen dong dao chieu lien tuc du no dung tot cho step-response 1 chieu.
            # Grid search rong roi tinh chinh: vung toi uu that ra kha muot quanh
            # viscous_friction~0.4-0.5 (khong phai gai nhon nhu lo ngai ban dau tu grid tho).
            # Fit cuoi: armature=0.04, friction=0.36 (khong anh huong ro ret), viscous_friction=
            # 0.5 -> posRMSE=0.65deg velRMSE=2.28rpm, period diff=+0.0050s/chu ky (giam ~61 lan
            # so voi baseline).
            stiffness=_env_float("CHAOS_HIP_ROLL_STIFFNESS", 45.0),
            damping=_env_float("CHAOS_HIP_ROLL_DAMPING", 7.5),
            armature=_env_float("CHAOS_HIP_ROLL_ARMATURE", 0.04),
            friction=_env_float("CHAOS_HIP_ROLL_FRICTION", 0.36),
            viscous_friction=_env_float("CHAOS_HIP_ROLL_VISCOUS_FRICTION", 0.5),
        ),
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            effort_limit=_env_float("CHAOS_HIP_YAW_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_HIP_YAW_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_HIP_YAW_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_HIP_YAW_VELOCITY_LIMIT_SIM", 14.653),
            stiffness=_env_float("CHAOS_HIP_YAW_STIFFNESS", 27.0),
            damping=_env_float("CHAOS_HIP_YAW_DAMPING", 0.9),
            # RE-TUNE 2026-07-03: GIU NGUYEN kp/kd=27/0.9. Tune armature/friction/viscous_friction
            # vs MuJoCo right_hip_yaw_joint (18 targets +-5..+-45deg). QUAN TRONG: sim joint_pos_limits
            # chi +-15deg (+-0.262rad) trong khi MuJoCo reference lenh toi +-45deg va DAT DUOC full
            # target (khong bi clamp o robot that/MuJoCo) - nguoc voi hip_roll (o day chinh SIM moi
            # la ben bi gioi han). Da LOAI target 20/25/30/35/40/45 (va am) khoi tap fit, chi dung
            # {5,10,15} (va am). Grid search armature x friction x viscous_friction: khac moi joint
            # khac da tune (knee/hip_pitch/hip_roll deu toi uu o armature=0), hip_yaw toi uu o
            # armature=0.02 (giu nguyen default) - armature=0 cho posRMSE/velRMSE te hon han. Coarse
            # search (24 to hop) + fine scan quanh armature=0.02: friction=0 toi uu, viscous_friction
            # trong khoang 0.30-0.40 deu tot (posRMSE~0.16-0.17, velRMSE~2.33); chon 0.35 vi posRMSE
            # thap nhat (0.1609) va overshoot rat nho (maxOS=0.095deg). Fit cuoi (vung hop le +-15deg):
            # posRMSE=0.16deg velRMSE=2.33rpm maxOS=0.095deg.
            armature=_env_float("CHAOS_HIP_YAW_ARMATURE", 0.02),
            friction=_env_float("CHAOS_HIP_YAW_FRICTION", 0.0),
            viscous_friction=_env_float("CHAOS_HIP_YAW_VISCOUS_FRICTION", 0.35),
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
            stiffness=_env_float("CHAOS_ANKLE_PITCH_STIFFNESS", 45),
            damping=_env_float("CHAOS_ANKLE_PITCH_DAMPING", 1.8),
            # RE-TUNE 2026-07-03: GIU NGUYEN kp/kd=45/1.8. Tune armature/friction/viscous_friction
            # vs MuJoCo right_ankle_pitch_joint (14 targets +-5..+25/-45deg). QUAN TRONG: sim
            # joint_pos_limits chi +-30deg (+-0.5236rad) trong khi MuJoCo reference lenh toi -45deg
            # va DAT DUOC (khong bi clamp o robot that) - giong pattern hip_yaw (sim gioi han chat
            # hon reference). Da LOAI target -35/-40/-45 khoi tap fit. Baseline (armature=0.01667
            # mac dinh, friction=0, viscous_friction=0) da chay nhanh hon MuJoCo ~13-15% (rpm_ratio
            # ~1.13-1.15) -> can tang viscous_friction de lam cham lai. Grid search armature x
            # friction x viscous_friction: armature=0.01667 (giu nguyen default) toi uu, friction
            # khong giup ich (giu 0). viscous_friction scan rong 0->2.0: posRMSE cuc tieu o ~0.7
            # (0.315deg) nhung velRMSE tiep tuc giam toi ~1.1 (vung 1.2-1.4 bi nhieu/khong on dinh -
            # tranh). Chon 1.1 (diem loss thap nhat truoc vung nhieu). Fit cuoi (vung hop le, loai
            # -35/-40/-45): posRMSE=0.45deg velRMSE=3.47rpm, overshoot=0.
            armature=_env_float("CHAOS_ANKLE_PITCH_ARMATURE", 0.01667),
            friction=_env_float("CHAOS_ANKLE_PITCH_FRICTION", 0.0),
            viscous_friction=_env_float("CHAOS_ANKLE_PITCH_VISCOUS_FRICTION", 1.1),
        ),
        # --- ANKLE ROLL --- (ImplicitActuatorCfg, fit theo right_ankle_roll_joint MuJoCo)
        # effort_limit NANG 6->10 Nm, scale kp/kd/armature x1.667 nhu ankle_pitch -> giu y curve.
        "ankle_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_roll_joint"],
            effort_limit=_env_float("CHAOS_ANKLE_ROLL_EFFORT_LIMIT", 10.0),
            effort_limit_sim=_env_float("CHAOS_ANKLE_ROLL_EFFORT_LIMIT_SIM", 10.0),
            velocity_limit=_env_float("CHAOS_ANKLE_ROLL_VELOCITY_LIMIT", 14.653),
            velocity_limit_sim=_env_float("CHAOS_ANKLE_ROLL_VELOCITY_LIMIT_SIM", 14.653),
            stiffness=_env_float("CHAOS_ANKLE_ROLL_STIFFNESS", 45),
            damping=_env_float("CHAOS_ANKLE_ROLL_DAMPING", 1.8),
            # RE-TUNE 2026-07-03: GIU NGUYEN kp/kd=45/1.8. Tune armature/friction/viscous_friction
            # vs MuJoCo right_ankle_roll_joint (6 targets +-5/10/15deg, sim range +-15deg trung
            # khop hoan toan voi reference - KHONG co clamp mismatch, dung het ca 6 target). Baseline
            # (armature=0.01667 mac dinh, friction=0, viscous_friction=0) chay nhanh hon MuJoCo
            # ~18% (rpm_ratio~1.18). Grid search armature x friction x viscous_friction: armature
            # =0.01667 (giu nguyen default) toi uu, friction khong giup ich (giu 0). viscous_friction
            # scan rong 1.0->2.0 (sau khi scan tho 0->1.2): cuc tieu THAT (khong phai bien) tai 1.2
            # (loss=2.379, smooth, khong nhieu nhu ankle_pitch). Fit cuoi: posRMSE=0.28deg
            # velRMSE=2.10rpm, overshoot=0.
            armature=_env_float("CHAOS_ANKLE_ROLL_ARMATURE", 0.01667),
            friction=_env_float("CHAOS_ANKLE_ROLL_FRICTION", 0.0),
            viscous_friction=_env_float("CHAOS_ANKLE_ROLL_VISCOUS_FRICTION", 1.2),
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
