#ifndef __DVL_INTERFACE_H__
#define __DVL_INTERFACE_H__

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <string>
#include <tacopie/tacopie>

#include <nortek_dvl/Dvl.h>
#include <nortek_dvl/DvlStatus.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Range.h>

namespace nortek_dvl {

class DvlInterface {
  private:
  
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher dvl_pub_;
    ros::Publisher dvl_status_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher beam_pubs_[4];
    
    void dataCb(tacopie::tcp_client& client,
              const tacopie::tcp_client::read_result& res);
    void connect();
    void process(std::string message);
    bool validateChecksum(std::string& message);

    bool publishMessages(std::string& str);
    void parseDvlStatus(unsigned long num, nortek_dvl::DvlStatus& status);
    template <class T>
    T hexStringToInt(std::string str);
    bool isVelocityValid(double vel);

    void readParams();

    std::string address_;
    std::string frame_id_, sonar_frame_id_;
    uint16_t port_;
    tacopie::tcp_client client_;
    bool use_enu_;

  public:
    explicit DvlInterface();
    ~DvlInterface();

 
};
}  // namespace nortek_dvl

#endif 
