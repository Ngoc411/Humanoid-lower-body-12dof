# AMP policy evaluation (3 phases → 3 files)

Đánh giá policy AMP đã train, xuất 3 file kết quả + time-series + biểu đồ.

## Chạy

Khi đã ở trong conda env (`env_isaaclab`) và đứng tại thư mục `chaos01_train-main`,
gọi `python` trực tiếp với đường dẫn tương đối tới script. Mỗi lệnh 1 dòng:

```powershell
# Cả 3 phase liên tiếp
python robolab\robolab-732b710f294926b8dd057057a6c3d86634c51e3f\scripts\rsl_rl\eval_amp.py --task Chaos-AMP-Play --num_envs 256 --headless --phase all --checkpoint "D:\Humanoid\Software\RL\Chaos_project\chaos01_train-main\logs\rsl_rl\chaos_amp_robolab\2026-05-22_23-27-20\model_10000.pt"

# Mục 2 — motion quality (joint tracking vs 3 clip, phase-normalized)
python robolab\robolab-732b710f294926b8dd057057a6c3d86634c51e3f\scripts\rsl_rl\eval_amp.py --task Chaos-AMP-Play --num_envs 64 --headless --phase motion --checkpoint "D:\Humanoid\Software\RL\Chaos_project\chaos01_train-main\logs\rsl_rl\chaos_amp_robolab\2026-05-22_23-27-20\model_10000.pt"

# Mục 3 — task metrics (velocity sweep + step-response + energy/survival)
python robolab\robolab-732b710f294926b8dd057057a6c3d86634c51e3f\scripts\rsl_rl\eval_amp.py --task Chaos-AMP-Play --num_envs 256 --headless --phase task --checkpoint "D:\Humanoid\Software\RL\Chaos_project\chaos01_train-main\logs\rsl_rl\chaos_amp_robolab\2026-05-22_23-27-20\model_10000.pt"

# Mục 4 — robustness (push set-root-velocity × cmd_vx × hướng + noise sweep)
python robolab\robolab-732b710f294926b8dd057057a6c3d86634c51e3f\scripts\rsl_rl\eval_amp.py --task Chaos-AMP-Play --num_envs 600 --headless --phase robust --checkpoint "D:\Humanoid\Software\RL\Chaos_project\chaos01_train-main\logs\rsl_rl\chaos_amp_robolab\2026-05-22_23-27-20\model_10000.pt"
```

Thay `--checkpoint "...\model_10000.pt"` bằng model muốn đánh giá.
Cờ thêm: `--eval_steps N` (đổi số control step), `--no_plots` (bỏ PNG), `--out_dir PATH`.

> `num_envs` nên ≥ số điều kiện của phase để mỗi điều kiện có ≥1 env (mỗi env = 1 điều kiện).
> Phase 3 có 17 điều kiện sweep → dùng 17×N (vd 256). Phase 4 có 3 cmd_vx × 5 push_vel × 4 hướng = 60 cell → dùng nhiều env để mỗi cell đủ trial.

## Output

```
logs/eval/<experiment>/<timestamp>/
├── 2_motion_quality.json     # 3 run (clip 0.29/0.43/0.72), joint tracking phase-normalized
├── 3_task_metrics.json       # velocity_sweep.table + step_response + survival
├── 4_robustness.json         # push_robustness + noise_robustness
├── timeseries/               # Parquet (CSV nếu thiếu pyarrow), env đại diện
│   ├── motion_quality.parquet
│   ├── velocity_sweep.parquet
│   ├── velocity_step_response.parquet
│   └── push_recovery.parquet
└── plots/                    # PNG sinh từ bảng JSON
    ├── motion_phase_<clip>.png
    ├── sweep_{vx,vy,wz}.png
    ├── push_robustness.png
    └── noise_robustness.png
```

## Thiết kế (tóm tắt)

| File | Vel cmd | So với gì | Phương pháp |
|------|---------|-----------|-------------|
| metrics.py | — | — | Hàm thuần numpy (test offline) |
| phase motion | 0.29/0.43/0.72 khớp clip | 3 reference clip | phase-normalized RMSE + correlation (vì `random_fetch=True`) |
| phase task | quét 3 trục dày (17 điểm) | — đo hiệu năng | đường cong RMSE/CoT/survival theo tốc độ |
| phase robust | 0/0.5/1.0 | — chống nhiễu | survival/recovery theo push_vel × hướng × cmd_vx |

Nguyên tắc: **tính metric trên TẤT CẢ env**, chỉ ghi time-series cho vài env đại diện. Trong lúc eval, terminations bị tắt (chỉ giữ time_out) và lệnh bị giữ cố định để dữ liệu liên tục; té ngã được phát hiện qua `base_z < 0.30` hoặc `tilt > 45°`.

## Phụ thuộc
`numpy` (bắt buộc), `matplotlib` (biểu đồ), `pandas`+`pyarrow` (parquet — nếu thiếu sẽ tự ghi CSV), `joblib` (đọc clip pkl cho phase motion).

## Tinh chỉnh
Mọi lưới/ngưỡng nằm trong `config.py`: `SWEEP_VX/VY/WZ`, `PUSH_VELS`, `PUSH_DIRECTIONS`, `ROBUST_CMD_VX`, `NOISE_SIGMAS`, `FALL_BASE_HEIGHT`, `FALL_TILT_DEG`, `FOOT_SLIDING_THRESHOLD`...
