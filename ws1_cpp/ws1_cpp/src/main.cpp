#include "main.hpp"
#include "utils.hpp"
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <ctime>

#define TEST 1
#define SCALE 0.25
#define USE_LOW_PASS_FILTER 0

#if TEST == 0

// ── Linkage test node ───────────────────────────────────────────────────
class LinkageTestNode : public rclcpp::Node
{
public:
    LinkageTestNode() : Node("linkage_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "===== Linkage 3D Solver Test (rad) =====");

        // All inputs in radians (rx = rot_x, ry = rot_y)
        struct TestCase { double rx, ry; const char* label; };
        TestCase tests[] = {
            { 0.000,  0.000, " 0 deg,  0 deg"},
            { 0.175,  0.000, "10 deg,  0 deg"},
            { 0.000,  0.175, " 0 deg, 10 deg"},
            { 0.175,  0.175, "10 deg, 10 deg"},
            { 0.349,  0.262, "20 deg, 15 deg"},
            {-0.175,  0.087, "-10 deg,  5 deg"},
            { 0.262, -0.175, "15 deg,-10 deg"},
            { 0.524,  0.000, "30 deg,  0 deg"},
            { 0.000,  0.524, " 0 deg, 30 deg"},
            { 0.436,  0.436, "25 deg, 25 deg"},
        };

        for (auto& t : tests) {
            auto fwd = linkage::solve_linkage(t.rx, t.ry);
            if (fwd.ok) {
                RCLCPP_INFO(this->get_logger(),
                    "FWD [%s]  rx=%7.4f ry=%7.4f  =>  theta_C=%8.4f  theta_D=%8.4f  (rad)",
                    t.label, t.rx, t.ry, fwd.theta_c_rad, fwd.theta_d_rad);

                // Inverse: recover (rot_x, rot_y) from (theta_c, theta_d)
                auto inv = linkage::solve_linkage_inverse(fwd.theta_c_rad, fwd.theta_d_rad);
                if (inv.ok) {
                    RCLCPP_INFO(this->get_logger(),
                        "INV  theta_C=%8.4f  theta_D=%8.4f  =>  rx=%8.4f  ry=%8.4f  (err=%.2e rad, iter=%d)",
                        fwd.theta_c_rad, fwd.theta_d_rad,
                        inv.rot_x_rad, inv.rot_y_rad, inv.error, inv.iterations);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "INV FAILED  theta_C=%.4f  theta_D=%.4f  (err=%.2e, iter=%d)",
                        fwd.theta_c_rad, fwd.theta_d_rad, inv.error, inv.iterations);
                }

                // Velocity conversion: test velocities in rad/s (~5 deg/s, ~-3 deg/s)
                double vel_tc = 0.087, vel_td = -0.052;
                auto vel = linkage::convert_velocity_to_rot(t.rx, t.ry, vel_tc, vel_td);
                if (vel.ok) {
                    RCLCPP_INFO(this->get_logger(),
                        "VEL  vel_tc=%7.4f  vel_td=%7.4f  =>  vel_rx=%7.4f  vel_ry=%7.4f  (rad/s)",
                        vel_tc, vel_td, vel.vel_rot_x, vel.vel_rot_y);
                } else {
                    RCLCPP_WARN(this->get_logger(), "VEL FAILED at rx=%.4f ry=%.4f", t.rx, t.ry);
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "FWD FAILED [%s]  rx=%7.4f  ry=%7.4f", t.label, t.rx, t.ry);
            }
            RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────────");
        }

        RCLCPP_INFO(this->get_logger(), "===== Test complete =====");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinkageTestNode>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}

#else

// ============================================================
// Helper: convert obs indices for one linkage pair
//   pos_d_idx:  index in obs for theta_D position  (e.g. 9+8)
//   pos_c_idx:  index in obs for theta_C position  (e.g. 9+10)
//   vel_d_idx:  index in obs for vel_theta_D       (e.g. 21+8)
//   vel_c_idx:  index in obs for vel_theta_C       (e.g. 21+10)
//   act_d_idx:  index in obs for prev_action D     (e.g. 33+8)
//   act_c_idx:  index in obs for prev_action C     (e.g. 33+10)
//
// Converts in-place:
//   pos: (theta_D, theta_C) → (rot_y, rot_x) via inverse linkage
//   vel: (vel_theta_D, vel_theta_C) → (vel_rot_y, vel_rot_x) via Jacobian inverse
//   act: (theta_D, theta_C) → (rot_y, rot_x) via inverse linkage
// ============================================================
// Converts one ankle linkage pair (pos + vel only) from motor space to rot space.
// prev_action is managed separately via prev_action_raw_ and is not touched here.
static bool convert_obs_pair_to_rot(
    std::array<float, 45>& obs,
    int pos_d_idx, int pos_c_idx,
    int vel_d_idx, int vel_c_idx,
    rclcpp::Logger logger)
{
    // --- Position: inverse linkage ---
    // Motor sends theta_c_rad directly (already -C from the formula).
    double theta_d = static_cast<double>(obs[pos_d_idx]);
    double theta_c = static_cast<double>(obs[pos_c_idx]);

    auto inv_pos = linkage::solve_linkage_inverse(theta_c, theta_d);
    if (!inv_pos.ok) {
        RCLCPP_WARN(logger,
            "Inverse pos failed: theta_c=%.2f theta_d=%.2f (err=%.2e, iter=%d)",
            theta_c, theta_d, inv_pos.error, inv_pos.iterations);
        return false;
    }

    obs[pos_d_idx] = static_cast<float>(inv_pos.rot_y_rad);
    obs[pos_c_idx] = static_cast<float>(inv_pos.rot_x_rad);

    // --- Velocity: Jacobian inverse at the solved (rot_x, rot_y) ---
    double vel_theta_d = static_cast<double>(obs[vel_d_idx]);
    double vel_theta_c = static_cast<double>(obs[vel_c_idx]);

    auto vel = linkage::convert_velocity_to_rot(
        inv_pos.rot_x_rad, inv_pos.rot_y_rad,
        vel_theta_c, vel_theta_d);

    if (!vel.ok) {
        RCLCPP_WARN(logger, "Velocity conversion failed at rx=%.2f ry=%.2f",
            inv_pos.rot_x_rad, inv_pos.rot_y_rad);
        return false;
    }

    obs[vel_d_idx] = static_cast<float>(vel.vel_rot_y);
    obs[vel_c_idx] = static_cast<float>(vel.vel_rot_x);

    return true;
}

void MainNode::openNewLogFile()
{
    namespace fs = std::filesystem;
    static const std::string kLogMotherDir = "/home/ngoc/ros2_ws/logs/";

    // Create this run's date-time child folder once, on the first CSV.
    if (log_dir_.empty()) {
        std::time_t t = std::time(nullptr);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &tm_buf);
        log_dir_ = kLogMotherDir + stamp + "/";
        fs::create_directories(log_dir_);
        RCLCPP_INFO(this->get_logger(), "[LOG] Run folder: %s", log_dir_.c_str());
    }

    if (log_file_.is_open()) log_file_.close();

    ++log_session_;
    char path[320];
    snprintf(path, sizeof(path), "%ssession_%03d.csv", log_dir_.c_str(), log_session_);

    log_file_.open(path);
    if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[LOG] Failed to open: %s", path);
        return;
    }
    log_file_opened_ = this->now();

    // header row
    for (int i = 0; i < OBS_DIM;    ++i) log_file_ << "obs_raw_"       << i << ",";
    for (int i = 0; i < ACTION_DIM; ++i) log_file_ << "action_raw_"    << i << ",";
    for (int i = 0; i < ACTION_DIM; ++i) log_file_ << "action_scaled_" << i << ",";
    for (int i = 0; i < ACTION_DIM; ++i) {
        log_file_ << "action_" << i;
        if (i < ACTION_DIM - 1) log_file_ << ",";
    }
    log_file_ << "\n";

    RCLCPP_INFO(this->get_logger(), "[LOG] New session: %s", path);
}

MainNode::MainNode()
: Node("main_node"),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  input_shape_({1, OBS_DIM})
{
    // ── 1. Init ONNX Runtime ──────────────────────────────────────────────
    onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "main_node");

    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);

    onnx_session_ = std::make_unique<Ort::Session>(
        *onnx_env_,
        "/home/ngoc/ros2_ws/src/ws1_cpp/actor.onnx",
        session_options);

    // ── 2. Get input/output names ─────────────────────────────────────────
    Ort::AllocatorWithDefaultOptions allocator;

    input_name_str_  = std::string(onnx_session_->GetInputNameAllocated(0,  allocator).get());
    output_name_str_ = std::string(onnx_session_->GetOutputNameAllocated(0, allocator).get());

    auto input_dims = onnx_session_->GetInputTypeInfo(0)
                          .GetTensorTypeAndShapeInfo().GetShape();

    std::cout << "Input  name : " << input_name_str_  << std::endl;
    std::cout << "Output name : " << output_name_str_ << std::endl;
    std::cout << "Input  shape: ";
    for (auto d : input_dims) std::cout << d << " ";
    std::cout << std::endl;

    // ── 3. Pre-allocate output message data vector ────────────────────────
    output_msg_.data.resize(ACTION_DIM);

    // ── 4. ROS2 pub/sub ───────────────────────────────────────────────────
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "sensor_45_float",
        10,
        std::bind(&MainNode::sensorDataCallback, this, std::placeholders::_1));

    result_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "main/output", 10);

    // ── 5. Watchdog: detect stale obs (lost SPI/link) → fail-safe ─────────
    last_valid_obs_ = this->now();
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MainNode::watchdogTick, this));

    RCLCPP_INFO(this->get_logger(),
        "MainNode ready — frame=%d obs=%d action=%d | state=INIT (waiting READY)",
        OBS_FRAME, OBS_DIM, ACTION_DIM);
}

void MainNode::sensorDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (static_cast<int>(msg->data.size()) != OBS_FRAME) {
        RCLCPP_WARN(this->get_logger(), "Expected %d floats, got %zu",
                    OBS_FRAME, msg->data.size());
        return;
    }

    const std::vector<float>& frame = msg->data;
    const float status = frame[OBS_DIM];   // index 45: 0=WAIT, 1=READY

    // ── Obs NaN/Inf fail-safe (before normalize / anything) ───────────────
    // A non-finite obs is a corrupt frame → drop it, never act on garbage.
    for (int i = 0; i < OBS_DIM; ++i) {
        if (!std::isfinite(frame[i])) {
            ++nan_obs_count_;
            RCLCPP_WARN(this->get_logger(),
                "Non-finite obs at index %d — frame dropped (fail-safe)", i);
            return;
        }
    }
    last_valid_obs_ = this->now();

    const bool ready = (status == STATUS_READY);

    // ── State machine (INIT → RUNNING → FALLEN → INIT) ────────────────────
    // STATUS is consulted ONLY in INIT. RUNNING reacts only to a detected fall.
    // FALLEN runs nothing but standing detection; the 5 s warmup is on the STM.
    switch (state_) {
        case RunState::INIT:
            // Wait for the STM to declare it is homed + upright + ready.
            if (ready) {
                fall_debounce_ = 0;
                state_ = RunState::RUNNING;
                RCLCPP_INFO(this->get_logger(), "[STATE] INIT → RUNNING (STATUS=READY)");
            }
            break;

        case RunState::RUNNING:
            // Leave RUNNING if the STM commands a stop (STATUS != READY, checked
            // every frame) OR Jetson detects a fall. Both → reset + FALLEN.
            if (!ready || detectFall(&frame[3])) {
                resetPolicyState();          // stop model, zero prev_action/filter
                state_ = RunState::FALLEN;
                RCLCPP_WARN(this->get_logger(), "[STATE] RUNNING → FALLEN (%s)",
                    !ready ? "STM STATUS=WAIT (stop)" : "fall detected");
            } else {
                runInference(frame);         // the ONLY path that publishes action
            }
            break;

        case RunState::FALLEN:
            // Policy fully stopped + reset. Obs still flows, so Jetson watches for
            // the robot to come back upright, then returns to INIT to await READY.
            if (detectStanding(&frame[3])) {
                state_ = RunState::INIT;
                RCLCPP_INFO(this->get_logger(),
                    "[STATE] FALLEN → INIT (robot upright again — awaiting READY)");
            }
            break;
    }
}

void MainNode::runInference(const std::vector<float>& input_data)
{
    // Continuous logging: open the first CSV lazily, then rotate to a new file
    // every kLogRotateSec so no single file grows too large.
    if (!log_file_.is_open()) {
        openNewLogFile();
    } else if ((this->now() - log_file_opened_).seconds() >= kLogRotateSec) {
        openNewLogFile();
    }

    try {
        // input_data is the 46-float frame; copy only the 45 obs into the model buffer.
        std::copy_n(input_data.begin(), OBS_DIM, input_buffer_.begin());

        // Debug formatter: float array → space-separated string
        auto fmt = [](const float* d, int n) {
            std::string s;
            char buf[16];
            for (int i = 0; i < n; ++i) {
                snprintf(buf, sizeof(buf), "%.4f", d[i]);
                s += buf;
                if (i < n - 1) s += ' ';
            }
            return s;
        };

        RCLCPP_INFO(this->get_logger(), "OBS raw  : %s",
            fmt(input_buffer_.data(), OBS_DIM).c_str());

        // ── PRE-MODEL: Convert motor angles → rot angles ──────────────────
        //
        // Pair 1: indices (D=17, C=19), vel (D=29, C=31), act (D=41, C=43)
        //   obs[9+8]=17   theta_D₁ pos     obs[9+10]=19   theta_C₁ pos
        //   obs[21+8]=29  vel_D₁           obs[21+10]=31  vel_C₁
        //   obs[33+8]=41  prev_act_D₁      obs[33+10]=43  prev_act_C₁
        //
        // Pair 2: indices (D=18, C=20), vel (D=30, C=32), act (D=42, C=44)
        //   obs[9+9]=18   theta_D₂ pos     obs[9+11]=20   theta_C₂ pos
        //   obs[21+9]=30  vel_D₂           obs[21+11]=32  vel_C₂
        //   obs[33+9]=42  prev_act_D₂      obs[33+11]=44  prev_act_C₂

        convert_obs_pair_to_rot(input_buffer_,
            17, 19,   // pos: D, C
            29, 31,   // vel: D, C
            this->get_logger());

        convert_obs_pair_to_rot(input_buffer_,
            18, 20,   // pos: D, C
            30, 32,   // vel: D, C
            this->get_logger());

        RCLCPP_INFO(this->get_logger(), "OBS inv  : %s",
            fmt(input_buffer_.data(), OBS_DIM).c_str());

        // Overwrite obs prev_action [33-44] with raw action from previous cycle
        // (ignores sensor-provided prev_action; zero on first cycle)
        for (int i = 0; i < ACTION_DIM; ++i) input_buffer_[33 + i] = prev_action_raw_[i];

        // Single pose offset — applied symmetrically on input AND output sides.
        // Set non-zero to shift the resting pose; both sides must always match.
        static constexpr float kPoseOffset[12] = {
             0.0f,  0.0f,  // [0,1]   hip pitch  L, R
             0.0f,  0.0f,  // [2,3]   hip roll   L, R
             0.0f,  0.0f,  // [4,5]   hip yaw    L, R
             -0.0f,  -0.0f,  // [6,7]   knee       L, R
             0.0f,  0.0f,  // [8,9]   ankle pitch L, R
             0.0f,  0.0f,  // [10,11] ankle roll  L, R
        };

        // INPUT side: subtract so policy sees angles relative to resting pose
        for (int i = 0; i < 12; ++i) input_buffer_[9 + i] -= kPoseOffset[i];

        RCLCPP_INFO(this->get_logger(), "OBS model: %s",
            fmt(input_buffer_.data(), OBS_DIM).c_str());

        // ── RUN MODEL ─────────────────────────────────────────────────────
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info_,
            input_buffer_.data(),
            OBS_DIM,
            input_shape_.data(),
            input_shape_.size());

        const char* input_names[]  = { input_name_str_.c_str() };
        const char* output_names[] = { output_name_str_.c_str() };

        auto start = std::chrono::high_resolution_clock::now();

        auto output_tensors = onnx_session_->Run(
            Ort::RunOptions{nullptr},
            input_names,  &input_tensor, 1,
            output_names, 1);

        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();

        // ── POST-MODEL: scale × 0.25 and clip to joint limits ────────────────
        // Limits are in model output space: rot space for ankle indices 8-11,
        // motor space for all others. Values from linkage::kAnkle*RotLimitRad.
        const float* out = output_tensors.front().GetTensorData<float>();

        // Action NaN/Inf fail-safe — MUST run before clamp. clamp(NaN) returns NaN
        // (comparisons with NaN are false), so isfinite has to gate it.
        for (int i = 0; i < ACTION_DIM; ++i) {
            if (!std::isfinite(out[i])) {
                ++nan_action_count_;
                RCLCPP_ERROR(this->get_logger(),
                    "Non-finite action at index %d — NOT published (fail-safe)", i);
                return;
            }
        }

        std::array<float, ACTION_DIM> action_raw_buf;
        std::copy_n(out, ACTION_DIM, action_raw_buf.begin());
        action_raw_buf[2] = std::max(-1.047f, std::min(0.0f,   action_raw_buf[2]));
        action_raw_buf[3] = std::max(0.0f,    std::min(1.047f, action_raw_buf[3]));
        action_raw_buf[4] = std::max(-0.35f,  std::min(0.35f,  action_raw_buf[4]));
        action_raw_buf[5] = std::max(-0.35f,  std::min(0.35f,  action_raw_buf[5]));

        static constexpr float kLimLo[12] = {
            -1.745f, -1.745f,  // [0,1]   hip pitch    L, R  ±100°
            -1.571f, -0.262f,  // [2,3]   hip roll     L(-90°…+15°), R(-15°…+90°)
            -0.262f, -0.262f,  // [4,5]   hip yaw      L, R  ±15°
            -2.094f, -2.094f,  // [6,7]   knee         L, R  -120°…0°
            -0.524f, -0.524f,  // [8,9]   ankle pitch  rot_y L, R  ±30°
            -0.262f, -0.262f,  // [10,11] ankle roll   rot_x L, R  ±15°
        };
        static constexpr float kLimHi[12] = {
             1.745f,  1.745f,
             0.262f,  1.571f,
             0.262f,  0.262f,
             0.000f,  0.000f,
             0.524f,  0.524f,
             0.262f,  0.262f,
        };

        // OUTPUT side: scale + offset + clip
        for (int i = 0; i < ACTION_DIM; ++i) {
            float v = action_raw_buf[i] * SCALE + kPoseOffset[i];
            float clipped = std::max(kLimLo[i], std::min(kLimHi[i], v));
            if (clipped != v) ++clip_count_;
            output_msg_.data[i] = clipped;
        }

        // Store clamped raw action as prev_action for next cycle's obs[33-44].
        prev_action_raw_ = action_raw_buf;

        RCLCPP_INFO(this->get_logger(), "OUT raw  : %s",
            fmt(action_raw_buf.data(), ACTION_DIM).c_str());
        RCLCPP_INFO(this->get_logger(), "OUT scaled: %s",
            fmt(output_msg_.data.data(), ACTION_DIM).c_str());

#if USE_LOW_PASS_FILTER
        // ── Low-pass filter (EMA) — mild noise reduction ──────────────────────
        // alpha=0.8: 80% new value, 20% previous. Increase toward 1.0 for less
        // filtering, decrease toward 0.0 for more smoothing (more lag).
        static constexpr float kFilterAlpha = 0.8f;

        if (!filter_initialized_) {
            std::copy(output_msg_.data.begin(), output_msg_.data.end(), filtered_action_.begin());            
            filter_initialized_ = true;
        } else {
            for (int i = 0; i < ACTION_DIM; ++i) {
                filtered_action_[i] = kFilterAlpha * output_msg_.data[i]
                                    + (1.0f - kFilterAlpha) * filtered_action_[i];
            }
        }
        // Copy filtered values back so linkage pass and publish use smoothed output
        std::copy(filtered_action_.begin(), filtered_action_.end(),
                output_msg_.data.begin());
#endif

        // Snapshot output_msg_ before linkage overwrites ankle indices
        std::array<float, ACTION_DIM> action_scaled_buf;
        std::copy(output_msg_.data.begin(), output_msg_.data.end(), action_scaled_buf.begin());

        // ── Forward linkage for ankle pairs (inputs already clipped above) ───
        // [8]=rot_y₁  [10]=rot_x₁  →  [8]=theta_D₁  [10]=theta_C₁
        // [9]=rot_y₂  [11]=rot_x₂  →  [9]=theta_D₂  [11]=theta_C₂
        {
            double rot_y_1 = static_cast<double>(output_msg_.data[8]);
            double rot_x_1 = static_cast<double>(output_msg_.data[10]);
            auto r1 = linkage::solve_linkage(rot_x_1, rot_y_1);
            if (r1.ok) {
                output_msg_.data[8]  = static_cast<float>(r1.theta_d_rad);
                output_msg_.data[10] = static_cast<float>(r1.theta_c_rad);   // already -C in formula
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Linkage pair (8,10) failed: rot_x=%.4f rot_y=%.4f", rot_x_1, rot_y_1);
            }

            double rot_y_2 = static_cast<double>(output_msg_.data[9]);
            double rot_x_2 = static_cast<double>(output_msg_.data[11]);
            auto r2 = linkage::solve_linkage(rot_x_2, rot_y_2);
            if (r2.ok) {
                output_msg_.data[9]  = static_cast<float>(r2.theta_d_rad);
                output_msg_.data[11] = static_cast<float>(r2.theta_c_rad);   // already -C in formula
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Linkage pair (9,11) failed: rot_x=%.4f rot_y=%.4f", rot_x_2, rot_y_2);
            }
        }

        // ── Publish final output ──────────────────────────────────────────
        result_publisher_->publish(output_msg_);

        std::string out_str;
        for (int i = 0; i < ACTION_DIM; ++i) {
            out_str += std::to_string(output_msg_.data[i]);
            if (i < ACTION_DIM - 1) out_str += ", ";
        }
        RCLCPP_INFO(this->get_logger(), "Action [%.3f ms]: [%s]", ms, out_str.c_str());

        // ── Write log row (continuous while RUNNING) ───────────────────────
        if (log_file_.is_open()) {
            for (int i = 0; i < OBS_DIM;    ++i) log_file_ << input_data[i]          << ",";
            for (int i = 0; i < ACTION_DIM; ++i) log_file_ << action_raw_buf[i]      << ",";
            for (int i = 0; i < ACTION_DIM; ++i) log_file_ << action_scaled_buf[i]   << ",";
            for (int i = 0; i < ACTION_DIM; ++i) {
                log_file_ << output_msg_.data[i];
                if (i < ACTION_DIM - 1) log_file_ << ",";
            }
            log_file_ << "\n";
            log_file_.flush();
        }

    } catch (const Ort::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ONNX error: %s", e.what());
    }
}

// ── Fall detection from projected gravity obs[3-5] ────────────────────────
// Upright, body-frame gravity ≈ (0, 0, -1) ⇒ tilt ≈ 0. Tilt from vertical:
//   tilt = atan2(|horizontal|, -gz)   (0° upright … 90° on side … 180° inverted)
// Returns true only after the tilt exceeds the threshold for kFallDebounce
// consecutive cycles (noise rejection).
// NOTE: assumes gz≈-1 when standing. Verify against logged obs[3-5]; if your
// convention gives +1 upright, flip the sign of gz below.
bool MainNode::detectFall(const float* gravity)
{
    const float gx = gravity[0], gy = gravity[1], gz = gravity[2];
    const double tilt = std::atan2(std::hypot(gx, gy), static_cast<double>(-gz));

    if (tilt > kFallTiltRad) {
        if (fall_debounce_ < kFallDebounce) ++fall_debounce_;
    } else {
        fall_debounce_ = 0;
    }
    return fall_debounce_ >= kFallDebounce;
}

// Standing detection (used in FALLEN): robot is upright again when the tilt is
// back under the threshold for kFallDebounce consecutive cycles. Obs keeps
// flowing during a fall, so this works while the robot is being re-homed.
bool MainNode::detectStanding(const float* gravity)
{
    const float gx = gravity[0], gy = gravity[1], gz = gravity[2];
    const double tilt = std::atan2(std::hypot(gx, gy), static_cast<double>(-gz));

    if (tilt < kFallTiltRad) {
        if (stand_debounce_ < kFallDebounce) ++stand_debounce_;
    } else {
        stand_debounce_ = 0;
    }
    return stand_debounce_ >= kFallDebounce;
}

// Reset all policy/internal state to zero (spec: do this on every fall).
void MainNode::resetPolicyState()
{
    prev_action_raw_.fill(0.0f);
    filtered_action_.fill(0.0f);
    filter_initialized_ = false;
    fall_debounce_      = 0;
    stand_debounce_     = 0;   // start FALLEN standing-detection fresh

    if (log_file_.is_open()) {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "[LOG] Session closed (policy reset)");
    }
}

// Watchdog: stale obs (lost SPI/link) must not keep driving the motors.
void MainNode::watchdogTick()
{
    ++watchdog_ticks_;

    const double age = (this->now() - last_valid_obs_).seconds();
    if (age > kWatchdogSec && state_ == RunState::RUNNING) {
        ++stale_count_;
        resetPolicyState();
        state_ = RunState::FALLEN;
        RCLCPP_WARN(this->get_logger(),
            "[WATCHDOG] No valid obs for %.3fs — RUNNING → FALLEN (fail-safe)", age);
    }

    // Periodic error-counter summary (~every 5 s at 10 Hz).
    if (watchdog_ticks_ % 50 == 0) {
        RCLCPP_INFO(this->get_logger(),
            "[STATS] nan_obs=%lu nan_action=%lu clip=%lu stale=%lu | state=%d",
            (unsigned long)nan_obs_count_, (unsigned long)nan_action_count_,
            (unsigned long)clip_count_, (unsigned long)stale_count_,
            static_cast<int>(state_));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#endif
