#include "action_node.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <cmath>

JointsCommandNode::JointsCommandNode()
: Node("joints_command_node"),
  spi_device_("/dev/spidev0.0"),
  spi_speed_(1000000),
  spi_mode_(SPI_MODE_0),
  bits_per_word_(8),
  spi_fd_(-1)
{
    this->declare_parameter<std::string>("spi_device", spi_device_);
    this->declare_parameter<int>("spi_speed", spi_speed_);

    spi_device_ = this->get_parameter("spi_device").as_string();
    spi_speed_  = this->get_parameter("spi_speed").as_int();

    if (!openSPI()) {
        throw std::runtime_error("Failed to open SPI");
    }

    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "main/output",
        10,
        std::bind(&JointsCommandNode::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Joints Command Node started");
}

JointsCommandNode::~JointsCommandNode()
{
    closeSPI();
}

bool JointsCommandNode::openSPI()
{
    spi_fd_ = open(spi_device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) { perror("SPI open"); return false; }

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0) { perror("SPI mode"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) { perror("SPI bits"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_) < 0) { perror("SPI speed"); return false; }

    return true;
}

void JointsCommandNode::closeSPI()
{
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        spi_fd_ = -1;
    }
}

void JointsCommandNode::commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 12) {
        RCLCPP_WARN(this->get_logger(), "Expected 12 floats, got %zu", msg->data.size());
        return;
    }

    // Fail-safe: never forward non-finite commands to the motors (defense in depth;
    // main_node already guards/clamps). "When in doubt, don't act."
    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(msg->data[i])) {
            RCLCPP_ERROR(this->get_logger(),
                "Non-finite action at index %d — frame NOT sent (fail-safe)", i);
            return;
        }
    }

    // Convert rad → deg before sending to SPI slave
    static constexpr float kRadToDeg = 180.0f / 3.14159265358979f;

    // Build 46-float frame (CMD_ACTION):
    //   [0]     = 2.0  (frame marker / CMD_ACTION)
    //   [1-12]  = 12 joint commands in degrees
    //   [13-45] = 0.0  (padding)
    std::vector<float> frame(NUM_FLOATS, 0.0f);
    frame[0] = 2.0f;
    for (int i = 0; i < 12; ++i) {
        frame[1 + i] = msg->data[i] * kRadToDeg;
    }

    std::string act_str;
    for (int i = 0; i < 12; ++i) {
        act_str += std::to_string(frame[1 + i]);
        if (i < 11) act_str += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Action (deg): [%s]", act_str.c_str());

    sendSPI(frame);
}

void JointsCommandNode::sendSPI(const std::vector<float>& data)
{
    const size_t tx_len = data.size() * sizeof(float);
    std::vector<uint8_t> tx(tx_len);
    memcpy(tx.data(), data.data(), tx_len);

    struct spi_ioc_transfer tr{};
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf       = reinterpret_cast<__u64>(tx.data());
    tr.rx_buf       = 0;
    tr.len          = tx_len;
    tr.speed_hz     = spi_speed_;
    tr.bits_per_word = bits_per_word_;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret != (int)tx_len) {
        std::cerr << "SPI write mismatch! Expected " << tx_len << " got " << ret << std::endl;
    } else {
        RCLCPP_INFO(this->get_logger(), "SPI TX OK | %d bytes | frame[0]=%.1f", ret, data[0]);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointsCommandNode>());
    rclcpp::shutdown();
    return 0;
}
