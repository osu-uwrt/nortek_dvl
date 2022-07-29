#pragma once

#include <bits/stdc++.h>

#include <boost/algorithm/string.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tacopie/tacopie>
#include <vector>

// message includes
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nortek_dvl/msg/dvl.hpp>
#include <nortek_dvl/msg/dvl_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/header.hpp>

namespace nortek_dvl {

class DvlInterface : public rclcpp::Node {
 private:
    rclcpp::Publisher<nortek_dvl::msg::Dvl>::SharedPtr dvl_pub_;
    rclcpp::Publisher<nortek_dvl::msg::DvlStatus>::SharedPtr dvl_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>
        beam_pubs_;

    void dataCb(tacopie::tcp_client& client,
                const tacopie::tcp_client::read_result& res);
    void connect();
    void process(std::string message);
    bool validateChecksum(std::string& message);

    bool publishMessages(std::string& str);
    void parseDvlStatus(unsigned long num, nortek_dvl::msg::DvlStatus& status);
    template <class T>
    T hexStringToInt(std::string str);
    bool isVelocityValid(double vel);

    void readParams();

    std::string address_;
    std::string frame_id_, sonar_frame_id_;
    uint16_t port_;
    tacopie::tcp_client client_;
    bool use_enu_;
    int max_connect_time_, min_connect_time_, timeout_;

 public:
    explicit DvlInterface();
    ~DvlInterface();
};
}  // namespace nortek_dvl
