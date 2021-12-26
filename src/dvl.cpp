#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "dvl_driver.h"

int main(int argc, char** argv) {
    // init ros node
    rclcpp::init(argc, argv);

    // Create the node and spin it 
    auto driver = std::make_shared<nortek_dvl::DvlInterface>();
    rclcpp::spin(driver);

    // the node has ben called to shutdown, so rclcpp needs to be shut down as well
    rclcpp::shutdown();
    return 0;
}
