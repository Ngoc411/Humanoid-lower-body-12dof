#ifndef ACTION_NODE_HPP
#define ACTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>
#include <cstdint>

constexpr size_t NUM_FLOATS = 46;   // [0]=2.0 CMD_ACTION, [1-12]=action(deg), [13-45]=padding

class JointsCommandNode : public rclcpp::Node
{
public:
    JointsCommandNode();
    ~JointsCommandNode();

private:
    // SPI
    bool openSPI();
    void closeSPI();
    void sendSPI(const std::vector<float>& data);

    // ROS callback
    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // SPI config
    std::string spi_device_;
    int         spi_speed_;
    uint8_t     spi_mode_;
    uint8_t     bits_per_word_;
    int         spi_fd_;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

#endif // ACTION_NODE_HPP
