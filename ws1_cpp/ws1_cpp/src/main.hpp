#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <memory>
#include <string>
#include <array>
#include <fstream>

#pragma pack(push, 1)
struct InputData {
    float base_lin_vel[3];   //  3
    float base_ang_vel[3];   //  3
    float gravity[3];        //  3
    float vel_commands[3];   //  3
    float joint_pos[12];     // 12
    float joint_vel[12];     // 12
    float last_action[12];   // 12
    // total: 48 — only first 45 are used as obs
};
#pragma pack(pop)

static constexpr int OBS_DIM    = 45;   // model input width
static constexpr int OBS_FRAME  = 46;   // SPI frame: obs[0-44] + STATUS[45]
static constexpr int ACTION_DIM = 12;

// STATUS field values (frame index 45, STM->Jetson)
static constexpr float STATUS_WAIT  = 0.0f;
static constexpr float STATUS_READY = 1.0f;

// Run-state machine. The 5 s stabilization warmup lives on the STM32 side, so the
// Jetson keeps only three states. STATUS=READY resumes (INIT→RUNNING); STATUS=WAIT
// stops at any frame (RUNNING→FALLEN), so the STM can command a stop.
enum class RunState {
    INIT,      // wait for STM STATUS=READY  → RUNNING
    RUNNING,   // policy active; a fall OR STATUS=WAIT leaves this state
    FALLEN     // policy off + reset; wait until Jetson sees the robot upright → INIT
};

class MainNode : public rclcpp::Node {
public:
    MainNode();

private:
    void sensorDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void runInference(const std::vector<float>& input_data);

    // ROS2
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr result_publisher_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env>     onnx_env_;
    std::unique_ptr<Ort::Session> onnx_session_;
    Ort::MemoryInfo               memory_info_;

    // Safe name storage (avoids dangling pointer from AllocatedStringPtr)
    std::string input_name_str_;
    std::string output_name_str_;

    // Pre-allocated I/O buffers (avoids per-inference heap allocation)
    std::array<float, OBS_DIM>    input_buffer_;
    std::array<float, ACTION_DIM> output_buffer_;
    std::vector<int64_t>          input_shape_;

    // Stores raw model output from previous cycle; injected into obs[33-44]
    // instead of the sensor-provided prev_action. Zero-initialized for cycle 0.
    std::array<float, ACTION_DIM> prev_action_raw_{};

    // Pre-built output message (avoids per-inference allocation)
    std_msgs::msg::Float32MultiArray output_msg_;

    std::array<float, ACTION_DIM> filtered_action_ = {};
    bool filter_initialized_ = false;

    // ── Data logging ─────────────────────────────────────────────────────
    // Each run creates one date-time child folder under the logs/ mother dir.
    // Logging is continuous; a fresh CSV is rotated in every kLogRotateSec.
    void openNewLogFile();
    std::ofstream log_file_;
    std::string   log_dir_;             // this run's child folder (date-time)
    int           log_session_   = 0;   // CSV index within the run
    rclcpp::Time  log_file_opened_;     // when current CSV was opened (rotation)
    static constexpr double kLogRotateSec = 20.0;  // new CSV every 20 s

    // ── State machine + fall/standing handling ───────────────────────────
    bool detectFall(const float* gravity);      // RUNNING: tilt > threshold (debounced)
    bool detectStanding(const float* gravity);  // FALLEN:  tilt < threshold (debounced)
    void resetPolicyState();
    void watchdogTick();

    RunState     state_          = RunState::INIT;
    int          fall_debounce_  = 0;   // consecutive tilted cycles  (RUNNING)
    int          stand_debounce_ = 0;   // consecutive upright cycles (FALLEN)
    rclcpp::Time last_valid_obs_;       // for watchdog freshness

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Tuning (spec defaults)
    static constexpr double kFallTiltRad  = 0.349;  // 20° from vertical
    static constexpr int    kFallDebounce = 3;      // consecutive cycles (both ways)
    static constexpr double kWatchdogSec  = 0.2;    // max obs staleness

    // ── Error counters ───────────────────────────────────────────────────
    uint64_t nan_obs_count_    = 0;
    uint64_t nan_action_count_ = 0;
    uint64_t clip_count_       = 0;
    uint64_t stale_count_      = 0;
    uint64_t watchdog_ticks_   = 0;
};

#endif // MAIN_HPP_
