#include "sensor_node.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <cmath>

#include <std_msgs/msg/float32_multi_array.hpp>

static constexpr unsigned int GPIO_CHIP_NUM    = 0;
static constexpr unsigned int GPIO_PIN15_LINE  = 85;    // OUTPUT – GPIO12 (header pin 15), 50Hz toggle
static constexpr unsigned int GPIO_PIN7_LINE   = 144;   // INPUT  – GPIO09 (header pin 7), EXTI rising edge

#define TEST 0

#if TEST == 2

// ══════════════════════════════════════════════════════════════════════════════
// TEST MODE 2: SPI loopback test at 5Hz, no GPIO, print received data
// ══════════════════════════════════════════════════════════════════════════════

SensorNode::SensorNode()
: Node("sensor_node"),
  spi_fd_(-1),
  spi_device_("/dev/spidev0.0"),
  spi_speed_(1000000),
  spi_mode_(SPI_MODE_0),
  bits_per_word_(8),
  num_floats_(45),
  rx_len_(num_floats_ * sizeof(float)),
  gpio_chip_(nullptr),
  line_trig_(nullptr),
  line_exti_(nullptr)
{
    if (!openSPI()) throw std::runtime_error("Failed to open SPI");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SensorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
        "[TEST2] SPI-only mode | 5Hz transfer | print RX data");
}

SensorNode::~SensorNode()
{
    closeSPI();
}

void SensorNode::timerCallback()
{
    static uint32_t cycle = 0;
    cycle++;

    std::vector<uint8_t> tx(rx_len_, 0);
    std::vector<uint8_t> rx(rx_len_, 0);

    std::vector<float> tx_floats(num_floats_);
    for (int i = 0; i < num_floats_; i++) {
        tx_floats[i] = (float)cycle + i * 0.01f;
    }
    memcpy(tx.data(), tx_floats.data(), rx_len_);

    struct spi_ioc_transfer tr{};
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = reinterpret_cast<__u64>(tx.data());
    tr.rx_buf        = reinterpret_cast<__u64>(rx.data());
    tr.len           = rx_len_;
    tr.speed_hz      = spi_speed_;
    tr.bits_per_word = bits_per_word_;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);

    if (ret != (int)rx_len_) {
        RCLCPP_ERROR(this->get_logger(),
            "[TEST2][SPI] Transfer FAILED! Expected %d bytes, got %d", rx_len_, ret);
        return;
    }

    std::vector<float> rx_floats(num_floats_);
    memcpy(rx_floats.data(), rx.data(), rx_len_);

    RCLCPP_INFO(this->get_logger(),
        "[TEST2][SPI] Cycle %u | TX[0]=%.2f TX[1]=%.2f | RX[0]=%.2f RX[1]=%.2f RX[2]=%.2f",
        cycle,
        tx_floats[0], tx_floats[1],
        rx_floats[0], rx_floats[1], rx_floats[2]);

    for (int i = 0; i < num_floats_; i++) {
        RCLCPP_INFO(this->get_logger(),
            "[TEST2][SPI]   RX[%2d] = %.6f", i, rx_floats[i]);
    }
}

// Stubs — GPIO not used in TEST 2
bool SensorNode::openGPIO()  { return true; }
void SensorNode::closeGPIO() {}
void SensorNode::gpioWatchThread() {}
void SensorNode::doSPITransfer() {}

bool SensorNode::openSPI()
{
    spi_fd_ = open(spi_device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) { perror("SPI open"); return false; }

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0)
        { perror("SPI mode"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0)
        { perror("SPI bits"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_) < 0)
        { perror("SPI speed"); return false; }

    RCLCPP_INFO(rclcpp::get_logger("sensor_node"),
        "[TEST2] SPI opened: %s @ %d Hz", spi_device_.c_str(), spi_speed_);
    return true;
}

void SensorNode::closeSPI()
{
    if (spi_fd_ >= 0) { close(spi_fd_); spi_fd_ = -1; }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}

#elif TEST == 1

// ══════════════════════════════════════════════════════════════════════════════
// TEST MODE 1: GPIO + SPI
//   - Pin 15 pulses HIGH every 20ms (50 Hz)
//   - Pin 7 rising edge → pull pin 15 LOW, then do SPI transfer
//   - TX always sends all 1.0f
//   - Prints dt between consecutive rising edges + latency from pin15 HIGH
// ══════════════════════════════════════════════════════════════════════════════

SensorNode::SensorNode()
: Node("sensor_node"),
  spi_fd_(-1),
  spi_device_("/dev/spidev0.0"),
  spi_speed_(1000000),
  spi_mode_(SPI_MODE_0),
  bits_per_word_(8),
  num_floats_(45),
  rx_len_(num_floats_ * sizeof(float)),
  gpio_chip_(nullptr),
  line_trig_(nullptr),
  line_exti_(nullptr)
{
    if (!openSPI())  throw std::runtime_error("Failed to open SPI");
    if (!openGPIO()) throw std::runtime_error("Failed to open GPIO");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SensorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
        "[TEST1] GPIO+SPI mode | pin15=50Hz pulse | pin7=EXTI→SPI | TX=all 1.0");
}

SensorNode::~SensorNode()
{
    closeGPIO();
    closeSPI();
}

bool SensorNode::openGPIO()
{
    std::string chip_path = "/dev/gpiochip" + std::to_string(GPIO_CHIP_NUM);
    gpio_chip_ = gpiod_chip_open(chip_path.c_str());
    if (!gpio_chip_) { perror("gpiod_chip_open"); return false; }

    line_trig_ = gpiod_chip_get_line(gpio_chip_, GPIO_PIN15_LINE);
    if (!line_trig_) { perror("gpiod_chip_get_line(trig)"); return false; }
    if (gpiod_line_request_output(line_trig_, "sensor_node_trig", 0) < 0) {
        perror("gpiod_line_request_output"); return false;
    }

    line_exti_ = gpiod_chip_get_line(gpio_chip_, GPIO_PIN7_LINE);
    if (!line_exti_) { perror("gpiod_chip_get_line(exti)"); return false; }
    if (gpiod_line_request_rising_edge_events(line_exti_, "sensor_node_exti") < 0) {
        perror("gpiod_line_request_rising_edge_events"); return false;
    }

    gpio_thread_run_.store(true);
    gpio_thread_ = std::thread(&SensorNode::gpioWatchThread, this);

    return true;
}

void SensorNode::closeGPIO()
{
    gpio_thread_run_.store(false);
    if (gpio_thread_.joinable()) gpio_thread_.join();

    if (line_trig_) { gpiod_line_release(line_trig_); line_trig_ = nullptr; }
    if (line_exti_) { gpiod_line_release(line_exti_); line_exti_ = nullptr; }
    if (gpio_chip_) { gpiod_chip_close(gpio_chip_);   gpio_chip_ = nullptr; }
}

void SensorNode::gpioWatchThread()
{
    RCLCPP_INFO(this->get_logger(), "[TEST1][PIN 7] EXTI watch thread started");

    struct timespec prev_ts = {0, 0};

    while (gpio_thread_run_.load()) {
        struct timespec timeout { 0, 100'000'000 };
        int ret = gpiod_line_event_wait(line_exti_, &timeout);

        if (ret < 0) { perror("gpiod_line_event_wait"); break; }
        if (ret == 0) continue;

        struct gpiod_line_event event{};
        if (gpiod_line_event_read(line_exti_, &event) < 0) {
            perror("gpiod_line_event_read"); continue;
        }

        if (event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) continue;

        /* dt between consecutive edges */
        double dt_ms = 0.0;
        if (prev_ts.tv_sec != 0) {
            dt_ms = ((event.ts.tv_sec - prev_ts.tv_sec)
                   + (event.ts.tv_nsec - prev_ts.tv_nsec) / 1e9) * 1000.0;
        }
        prev_ts = event.ts;

        /* latency from pin15 HIGH to this pin7 rising edge */
        double latency_ms = 0.0;
        int64_t t0 = pin15_high_ns_.load();
        if (t0 != 0) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            int64_t t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            latency_ms = (t1 - t0) / 1e6;
        }

        RCLCPP_INFO(this->get_logger(),
            "[PIN 7] Rising edge | dt=%.3f ms | latency=%.3f ms", dt_ms, latency_ms);

        transfer_in_progress_.store(true);
        gpiod_line_set_value(line_trig_, 0);
        pulse_state_.store(false);

        doSPITransfer();
        transfer_in_progress_.store(false);
    }

    RCLCPP_INFO(this->get_logger(), "[TEST1][PIN 7] EXTI watch thread exited");
}

void SensorNode::timerCallback()
{
    if (!transfer_in_progress_.load()) {
        bool next = !pulse_state_.load();
        pulse_state_.store(next);
        if (next) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            pin15_high_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        }
        gpiod_line_set_value(line_trig_, next ? 1 : 0);
    }
}

bool SensorNode::openSPI()
{
    spi_fd_ = open(spi_device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) { perror("SPI open"); return false; }

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0)
        { perror("SPI mode"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0)
        { perror("SPI bits"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_) < 0)
        { perror("SPI speed"); return false; }

    RCLCPP_INFO(rclcpp::get_logger("sensor_node"),
        "[TEST1] SPI opened: %s @ %d Hz", spi_device_.c_str(), spi_speed_);
    return true;
}

void SensorNode::closeSPI()
{
    if (spi_fd_ >= 0) { close(spi_fd_); spi_fd_ = -1; }
}

void SensorNode::doSPITransfer()
{
    std::vector<uint8_t> tx(rx_len_, 0);
    std::vector<uint8_t> rx(rx_len_, 0);

    std::vector<float> tx_floats(num_floats_, 1.0f);
    memcpy(tx.data(), tx_floats.data(), rx_len_);

    struct spi_ioc_transfer tr{};
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = reinterpret_cast<__u64>(tx.data());
    tr.rx_buf        = reinterpret_cast<__u64>(rx.data());
    tr.len           = rx_len_;
    tr.speed_hz      = spi_speed_;
    tr.bits_per_word = bits_per_word_;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);

    if (ret != (int)rx_len_) {
        RCLCPP_ERROR(this->get_logger(),
            "[SPI] FAILED! Expected %d bytes, got %d", rx_len_, ret);
        return;
    }

    std::vector<float> rx_floats(num_floats_);
    memcpy(rx_floats.data(), rx.data(), rx_len_);

    RCLCPP_INFO(this->get_logger(),
        "[SPI] OK | TX=1.0 | RX[0]=%.2f RX[1]=%.2f RX[44]=%.2f",
        rx_floats[0], rx_floats[1], rx_floats[44]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}

#else

// ══════════════════════════════════════════════════════════════════════════════
// PRODUCTION MODE: full SPI + GPIO + topic publish
//   - TX always sends all 1.0f
// ══════════════════════════════════════════════════════════════════════════════

SensorNode::SensorNode()
: Node("sensor_node"),
  spi_fd_(-1),
  spi_device_("/dev/spidev0.0"),
  spi_speed_(1000000),
  spi_mode_(SPI_MODE_0),
  bits_per_word_(8),
  num_floats_(46),   // 45 obs [0-44] + STATUS [45] (STM->Jetson: 0=WAIT, 1=READY)
  rx_len_(num_floats_ * sizeof(float)),
  gpio_chip_(nullptr),
  line_trig_(nullptr),
  line_exti_(nullptr)
{
    this->declare_parameter<std::string>("spi_device", spi_device_);
    this->declare_parameter<int>("spi_speed", spi_speed_);

    spi_device_ = this->get_parameter("spi_device").as_string();
    spi_speed_  = this->get_parameter("spi_speed").as_int();

    if (!openSPI())  throw std::runtime_error("Failed to open SPI");
    if (!openGPIO()) throw std::runtime_error("Failed to open GPIO");

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "sensor_45_float", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SensorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
        "SPI Node started | pin15=trigger pulse | pin7=EXTI rising->SPI | TX=all 1.0");
}

SensorNode::~SensorNode()
{
    closeGPIO();
    closeSPI();
}

// ──────────────────────────────────────────────────────────────────────────────
// GPIO
// ──────────────────────────────────────────────────────────────────────────────
bool SensorNode::openGPIO()
{
    std::string chip_path = "/dev/gpiochip" + std::to_string(GPIO_CHIP_NUM);
    gpio_chip_ = gpiod_chip_open(chip_path.c_str());
    if (!gpio_chip_) { perror("gpiod_chip_open"); return false; }

    line_trig_ = gpiod_chip_get_line(gpio_chip_, GPIO_PIN15_LINE);
    if (!line_trig_) { perror("gpiod_chip_get_line(trig)"); return false; }
    if (gpiod_line_request_output(line_trig_, "sensor_node_trig", 0) < 0) {
        perror("gpiod_line_request_output"); return false;
    }

    line_exti_ = gpiod_chip_get_line(gpio_chip_, GPIO_PIN7_LINE);
    if (!line_exti_) { perror("gpiod_chip_get_line(exti)"); return false; }
    if (gpiod_line_request_rising_edge_events(line_exti_, "sensor_node_exti") < 0) {
        perror("gpiod_line_request_rising_edge_events"); return false;
    }

    gpio_thread_run_.store(true);
    gpio_thread_ = std::thread(&SensorNode::gpioWatchThread, this);

    return true;
}

void SensorNode::closeGPIO()
{
    gpio_thread_run_.store(false);
    if (gpio_thread_.joinable()) gpio_thread_.join();

    if (line_trig_) { gpiod_line_release(line_trig_); line_trig_ = nullptr; }
    if (line_exti_) { gpiod_line_release(line_exti_); line_exti_ = nullptr; }
    if (gpio_chip_) { gpiod_chip_close(gpio_chip_);   gpio_chip_ = nullptr; }
}

void SensorNode::gpioWatchThread()
{
    RCLCPP_INFO(this->get_logger(), "[PIN 7] EXTI watch thread started");

    struct timespec prev_ts = {0, 0};

    while (gpio_thread_run_.load()) {
        struct timespec timeout { 0, 100'000'000 };
        int ret = gpiod_line_event_wait(line_exti_, &timeout);

        if (ret < 0) { perror("gpiod_line_event_wait"); break; }
        if (ret == 0) continue;

        struct gpiod_line_event event{};
        if (gpiod_line_event_read(line_exti_, &event) < 0) {
            perror("gpiod_line_event_read"); continue;
        }

        if (event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
            RCLCPP_WARN(this->get_logger(),
                "[PIN 7] Unexpected event type=%d (ignored)", event.event_type);
            continue;
        }

        /* dt between consecutive edges */
        double dt_ms = 0.0;
        if (prev_ts.tv_sec != 0) {
            dt_ms = ((event.ts.tv_sec - prev_ts.tv_sec)
                   + (event.ts.tv_nsec - prev_ts.tv_nsec) / 1e9) * 1000.0;
        }
        prev_ts = event.ts;

        /* latency from pin15 HIGH to this pin7 rising edge */
        double latency_ms = 0.0;
        int64_t t0 = pin15_high_ns_.load();
        if (t0 != 0) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            int64_t t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            latency_ms = (t1 - t0) / 1e6;
        }

        RCLCPP_INFO(this->get_logger(),
            "[PIN 7] Rising edge | dt=%.3f ms | latency=%.3f ms", dt_ms, latency_ms);

        // EXTI → force pin 15 LOW, block timer from setting HIGH
        transfer_in_progress_.store(true);
        gpiod_line_set_value(line_trig_, 0);
        pulse_state_.store(false);

        // Trigger SPI transfer
        doSPITransfer();
        transfer_in_progress_.store(false);
    }

    RCLCPP_INFO(this->get_logger(), "[PIN 7] EXTI watch thread exited");
}

void SensorNode::timerCallback()
{
    if (!transfer_in_progress_.load()) {
        bool next = !pulse_state_.load();
        pulse_state_.store(next);
        if (next) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            pin15_high_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        }
        gpiod_line_set_value(line_trig_, next ? 1 : 0);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SPI
// ──────────────────────────────────────────────────────────────────────────────
bool SensorNode::openSPI()
{
    spi_fd_ = open(spi_device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) { perror("SPI open"); return false; }

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0)
        { perror("SPI mode"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0)
        { perror("SPI bits per word"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_) < 0)
        { perror("SPI speed"); return false; }

    RCLCPP_INFO(rclcpp::get_logger("sensor_node"),
        "SPI opened: %s @ %d Hz", spi_device_.c_str(), spi_speed_);
    return true;
}

void SensorNode::closeSPI()
{
    if (spi_fd_ >= 0) { close(spi_fd_); spi_fd_ = -1; }
}

void SensorNode::doSPITransfer()
{
    std::vector<uint8_t> tx(rx_len_, 0);
    std::vector<uint8_t> rx(rx_len_, 0);

    std::vector<float> tx_floats(num_floats_, 1.0f);
    memcpy(tx.data(), tx_floats.data(), rx_len_);

    struct spi_ioc_transfer tr{};
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = reinterpret_cast<__u64>(tx.data());
    tr.rx_buf        = reinterpret_cast<__u64>(rx.data());
    tr.len           = rx_len_;
    tr.speed_hz      = spi_speed_;
    tr.bits_per_word = bits_per_word_;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);

    if (ret != (int)rx_len_) {
        RCLCPP_ERROR(this->get_logger(),
            "[SPI] FAILED! Expected %d bytes, got %d", rx_len_, ret);
        return;
    }

    std::vector<float> float_data(num_floats_);
    memcpy(float_data.data(), rx.data(), rx_len_);

    // obs layout: [0-2] base ang vel | [3-5] projected gravity | [6-8] vel commands
    //             [9-20] joint pos (deg→rad) | [21-32] joint vel (RPM→rad/s) | [33-44] prev action (deg→rad)
    //             [45] STATUS (STM->Jetson: 0=WAIT, 1=READY) — published RAW, no conversion
    static constexpr float kDegToRad  = M_PI / 180.0f;
    static constexpr float kRpmToRads = 2.0f * M_PI / 60.0f;

    for (int i = 9;  i <= 20; ++i) float_data[i] *= kDegToRad;   // joint pos: deg → rad
    for (int i = 21; i <= 32; ++i) float_data[i] *= kRpmToRads;  // joint vel: RPM → rad/s
    for (int i = 33; i <= 44; ++i) float_data[i] *= kDegToRad;   // prev action: deg → rad

    // Published array carries 46 floats: obs[0-44] (converted) + STATUS[45] (raw).
    std_msgs::msg::Float32MultiArray msg;
    msg.data = float_data;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
        "[SPI] OK | RX[0]=%.2f RX[1]=%.2f RX[44]=%.2f STATUS=%.1f",
        float_data[0], float_data[1], float_data[44], float_data[45]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}

#endif