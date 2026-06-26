#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <gpiod.h>

#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>

class SensorNode : public rclcpp::Node
{
public:
    SensorNode();
    ~SensorNode();

private:
    // ── SPI ──────────────────────────────────────────────
    bool openSPI();
    void closeSPI();
    void doSPITransfer();

    int         spi_fd_;
    std::string spi_device_;
    int         spi_speed_;
    uint8_t     spi_mode_;
    uint8_t     bits_per_word_;
    int         num_floats_;
    int         rx_len_;

    // ── ROS ──────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback();

    // ── GPIO (libgpiod) ──────────────────────────────────
    bool            openGPIO();
    void            closeGPIO();
    void            gpioWatchThread();

    gpiod_chip*     gpio_chip_;
    gpiod_line*     line_trig_;
    gpiod_line*     line_exti_;

    std::thread         gpio_thread_;
    std::atomic<bool>   gpio_thread_run_{false};

    std::atomic<bool>   trigger_active_{true};
    std::atomic<bool>   pulse_state_{false};

    // ── Timing ───────────────────────────────────────────
    std::atomic<bool>    transfer_in_progress_{false};
    std::atomic<int64_t> pin15_high_ns_{0};
};