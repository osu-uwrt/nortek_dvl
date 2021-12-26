#include "dvl_driver.h"

#include <chrono>

using namespace nortek_dvl;

DvlInterface::DvlInterface() : Node("nortek_dvl") {
    readParams();
    connect();

    dvl_pub_ = this->create_publisher<nortek_dvl::msg::Dvl>(
        "dvl", rclcpp::SensorDataQoS());
    dvl_status_pub_ = this->create_publisher<nortek_dvl::msg::DvlStatus>(
        "dvl/status", rclcpp::SensorDataQoS());
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "dvl_twist", rclcpp::SensorDataQoS());
    for (int i = 0; i < 4; i++) {
        beam_pubs_.push_back(this->create_publisher<sensor_msgs::msg::Range>(
            "dvl_sonar" + std::to_string(i), rclcpp::SensorDataQoS()));
    }
}

DvlInterface::~DvlInterface() { client_.disconnect(); }

void DvlInterface::dataCb(tacopie::tcp_client &client,
                          const tacopie::tcp_client::read_result &res) {
    // ROS_INFO("Result: %d", res.success);
    if (res.success) {
        process(std::string(res.buffer.begin(), res.buffer.end()));
        // ROS_INFO("Client connected");
        client.async_write({res.buffer, nullptr});
        client_.async_read(
            {1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                             std::placeholders::_1)});
    } else {
        RCLCPP_WARN(this->get_logger(), "Nortek DVL: client disconnected");
        client_.disconnect();
    }
}

void DvlInterface::connect() {
    auto start = this->get_clock()->now();
    auto maxTime =
        start + rclcpp::Duration(std::chrono::seconds(max_connect_time_));
    bool connectSuccess = false;

    // loop this until the connection succeeds, RCLCPP is called to shutdown, or
    // timeout expires
    while (!connectSuccess && rclcpp::ok() &&
           maxTime > this->get_clock()->now()) {
        try {
            client_.connect(address_, port_, timeout_);
            client_.async_read(
                {1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                                 std::placeholders::_1)});
            connectSuccess = true;
        } catch (tacopie::tacopie_error &e) {
            std::cout << "DVL connection timeout, retrying " << address_ << ":"
                      << std::to_string(port_) << std::endl;
        }
    }

    if (!connectSuccess) {
        throw std::runtime_error("Unable to connect to DVL on address " +
                                 address_ + ":" + std::to_string(port_));
    }
}

bool DvlInterface::validateChecksum(std::string &message) {
    std::vector<std::string> results;
    boost::algorithm::split(results, message, boost::algorithm::is_any_of("*"));

    if (results.size() != 2) {
        return false;
    } else {
        // if checksum correct
        message = results[0];
        auto checksum = hexStringToInt<unsigned int>(results[1]);
        // TODO: compare checksum
        return true;
    }
}

void DvlInterface::process(std::string message) {
    boost::algorithm::trim(message);
    if (message.compare(0, 6, "Nortek") == 0) {
        RCLCPP_INFO(this->get_logger(), "Connected to DVL");
    } else {
        if (validateChecksum(message)) {
            RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
            publishMessages(message);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid message from DVL");
        }
    }
}

template <class T>
T DvlInterface::hexStringToInt(std::string str) {
    T x;
    std::stringstream ss;
    ss << std::hex << str;
    ss >> x;
    return x;
}

bool DvlInterface::publishMessages(std::string &str) {
    std::vector<std::string> results;
    boost::algorithm::split(results, str, boost::algorithm::is_any_of(",="));

    if (results.size() == 17) {
        nortek_dvl::msg::Dvl dvl;
        nortek_dvl::msg::DvlStatus status;
        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        std_msgs::msg::Header header;
        sensor_msgs::msg::Range beams[beam_pubs_.size()];

        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;
        dvl.header = header;
        dvl.time = std::stod(results[1]);
        dvl.dt1 = std::stof(results[2]);
        dvl.dt2 = std::stof(results[3]);
        for (int i = 0; i < beam_pubs_.size(); i++) {
            beams[i].header.stamp = this->get_clock()->now();
            std::string frame = sonar_frame_id_;
            frame.replace(frame.find("%d"), 2, std::to_string(i));
            beams[i].header.frame_id = frame;
            beams[i].range = std::stof(results[8 + i]);
            beams[i].max_range = 10;
        }
        dvl.battery_voltage = std::stof(results[12]);
        dvl.speed_sound = std::stof(results[13]);
        dvl.pressure = std::stof(results[14]);
        dvl.temp = std::stof(results[15]);

        if (dvl_pub_->get_subscription_count() > 0)
            dvl_pub_->publish(dvl);

        if (isVelocityValid(std::stof(results[4])) &&
            isVelocityValid(std::stof(results[5])) &&
            isVelocityValid(std::stof(results[6]))) {
            double fom = std::stof(results[7]);
            twist.header = header;
            twist.twist.covariance[0] = fom * fom;
            twist.twist.covariance[7] = fom * fom;
            twist.twist.covariance[14] = fom * fom;
            twist.twist.twist.linear.x = std::stof(results[4]);
            twist.twist.twist.linear.y = std::stof(results[5]);
            twist.twist.twist.linear.z = std::stof(results[6]);
            if (use_enu_) {
                twist.twist.twist.linear.y *= -1;
                twist.twist.twist.linear.z *= -1;
            }
            if (twist_pub_->get_subscription_count() > 0)
                twist_pub_->publish(twist);
        }

        for (int i = 0; i < beam_pubs_.size(); i++) {
            if (beams[i].range != 0) {
                beam_pubs_.at(i)->publish(beams[i]);
            }
        }

        parseDvlStatus(hexStringToInt<unsigned long>(results[16].substr(2)),
                       status);
        if (dvl_status_pub_->get_subscription_count() > 0)
            dvl_status_pub_->publish(status);
        return true;
    }

    return false;
}

void DvlInterface::parseDvlStatus(unsigned long num,
                                  nortek_dvl::msg::DvlStatus &status) {
    std::bitset<32> bset(num);

    status.header.stamp = this->get_clock()->now();

    status.b1_vel_valid = bset[0];
    status.b2_vel_valid = bset[1];
    status.b3_vel_valid = bset[2];
    status.b4_vel_valid = bset[3];
    status.b1_dist_valid = bset[4];
    status.b2_dist_valid = bset[5];
    status.b3_dist_valid = bset[6];
    status.b4_dist_valid = bset[7];
    status.b1_fom_valid = bset[8];
    status.b2_fom_valid = bset[9];
    status.b3_fom_valid = bset[10];
    status.b4_fom_valid = bset[11];
    status.x_vel_valid = bset[12];
    status.y_vel_valid = bset[13];
    status.z1_vel_valid = bset[14];
    status.z2_vel_valid = bset[15];
    status.x_fom_valid = bset[16];
    status.y_fom_valid = bset[17];
    status.z1_fom_valid = bset[18];
    status.z2_fom_valid = bset[19];

    if (bset[20]) {
        status.proc_cap = 3;
    } else if (bset[21]) {
        status.proc_cap = 6;
    } else if (bset[22]) {
        status.proc_cap = 12;
    }

    int wakeupstate = bset[28] << 3 | bset[29] << 2 | bset[30] << 1 | bset[31];

    if (wakeupstate == 0b0010) {
        status.wakeup_state = "break";
    } else if (wakeupstate == 0b0011) {
        status.wakeup_state = "RTC Alarm";
    } else if (wakeupstate == 0b0000) {
        status.wakeup_state = "bad power";
    } else if (wakeupstate == 0b0001) {
        status.wakeup_state = "power applied";
    }
}

void DvlInterface::readParams() {
    // Need to declare params for ros2 before retrieval
    this->declare_parameter<std::string>("address", "192.168.1.220");
    this->declare_parameter<int>("port", 9004);
    this->declare_parameter<int>("timeout", 500);
    this->declare_parameter<int>("max_connect_time", 100);
    this->declare_parameter<std::string>("frame_id", "dvl_link");
    this->declare_parameter<std::string>("sonar_frame_id", "dvl_sonar%d_link");
    this->declare_parameter<bool>("use_enu", true);

    // now we can get the param values;
    this->get_parameter("address", address_);
    this->get_parameter("port", port_);
    this->get_parameter("timeout", timeout_);
    this->get_parameter("max_connect_time", max_connect_time_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("sonar_frame_id", sonar_frame_id_);
    this->get_parameter("use_enu", use_enu_);

    // show the params to the user to confirm they were recieved
    std::cout << "DVL PARAMS" << std::endl;
    std::cout << "-----------------" << std::endl;
    std::cout << "address: " << address_ << std::endl;
    std::cout << "port: " << port_ << std::endl;
    std::cout << "timeout: " << timeout_ << std::endl;
    std::cout << "max_connect_time: " << max_connect_time_ << std::endl;
    std::cout << "-----------------" << std::endl;
    std::cout << "frame_id: " << frame_id_ << std::endl;
    std::cout << "sonar_frame_id: " << sonar_frame_id_ << std::endl;
    std::cout << "use_enu: " << use_enu_ << std::endl;
    std::cout << "-----------------" << std::endl;
}

bool DvlInterface::isVelocityValid(double vel) {
    return vel > -32;  // -32.786 is invalid velocity
}
