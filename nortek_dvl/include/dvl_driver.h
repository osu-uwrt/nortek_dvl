#pragma once

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "async_tcp.h"

// message includes
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nortek_dvl_msgs/msg/dvl.hpp>
#include <nortek_dvl_msgs/msg/dvl_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

namespace nortek_dvl {

class DvlInterface : public rclcpp::Node {
 private:
    asio_tcp::TCPClient client = asio_tcp::TCPClient();
    rclcpp::Publisher<nortek_dvl_msgs::msg::Dvl>::SharedPtr dvl_pub_;
    rclcpp::Publisher<nortek_dvl_msgs::msg::DvlStatus>::SharedPtr dvl_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>
        beam_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_lock_pub_;

    void connect();
    int process(std::string message);
    bool validateChecksum(std::string& message);

    bool publishMessages(std::string& str);
    std::vector<std::string> parseMessage(std::string &str);
    void parseDvlStatus(unsigned long num, nortek_dvl_msgs::msg::DvlStatus& status);
    template <class T>
    T hexStringToInt(std::string str);
    bool isVelocityValid(double vel);

    void readParams();

    std::string address_;
    std::string frame_id_, sonar_frame_id_;
    uint16_t port_;
    bool use_enu_;
    int max_connect_time_, min_connect_time_, timeout_;

    // socket manager
    std::shared_ptr<asio_tcp::TCPClient> connection;

 public:
    explicit DvlInterface();
    ~DvlInterface();
};
}  // namespace nortek_dvl
