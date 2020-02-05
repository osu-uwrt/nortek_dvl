#include "dvl.h"

using namespace nortek_dvl;

DvlInterface::DvlInterface()
    : nh_(),
      private_nh_("~")
{
  readParams();
  connect();

  dvl_pub_ = nh_.advertise<nortek_dvl::Dvl>("dvl", 10);
  dvl_status_pub_ = nh_.advertise<nortek_dvl::DvlStatus>("dvl/status", 10);
  twist_pub_ =
      nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("dvl_twist", 10);
}

DvlInterface::~DvlInterface() { client_.disconnect(); }

void DvlInterface::dataCb(tacopie::tcp_client &client,
                          const tacopie::tcp_client::read_result &res)
{
  //ROS_INFO("Result: %d", res.success);
  if (res.success)
  {
    process(std::string(res.buffer.begin(), res.buffer.end()));
    //ROS_INFO("Client connected");
    client.async_write({res.buffer, nullptr});
    client_.async_read({1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                                        std::placeholders::_1)});
  }
  else
  {
    ROS_WARN("Nortek DVL: client disconnected");
    client_.disconnect();
  }
}

void DvlInterface::connect()
{
  try
  {
    client_.connect(address_, port_, 500);
    client_.async_read({1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                                        std::placeholders::_1)});
  }
  catch (tacopie::tacopie_error &e)
  {
    throw std::runtime_error("Unable to connect to DVL on address " + address_ +
                             ":" + std::to_string(port_));
  }
}

bool DvlInterface::validateChecksum(std::string &message)
{
  std::vector<std::string> results;
  boost::algorithm::split(results, message, boost::algorithm::is_any_of("*"));

  if (results.size() != 2)
  {
    return false;
  }
  else
  {
    // if checksum correct
    message = results[0];
    auto checksum = hexStringToInt<unsigned int>(results[1]);
    // TODO: compare checksum
    return true;
  }
}

void DvlInterface::process(std::string message)
{
  boost::algorithm::trim(message);
  if (message.compare(0, 6, "Nortek") == 0)
  {
    ROS_INFO("Connected to DVL");
  }
  else
  {
    if (validateChecksum(message))
    {
      ROS_DEBUG("%s", message.c_str());
      publishMessages(message);
    }
    else
    {
      ROS_WARN("Invalid message from DVL");
    }
  }
}

template <class T>
T DvlInterface::hexStringToInt(std::string str)
{
  T x;
  std::stringstream ss;
  ss << std::hex << str;
  ss >> x;
  return x;
}

bool DvlInterface::publishMessages(std::string &str)
{
  std::vector<std::string> results;
  boost::algorithm::split(results, str, boost::algorithm::is_any_of(",="));

  if (results.size() == 17)
  {
    nortek_dvl::Dvl dvl;
    nortek_dvl::DvlStatus status;
    geometry_msgs::TwistWithCovarianceStamped twist;
    std_msgs::Header header;

    header.stamp = ros::Time();
    header.frame_id = frame_id_;
    dvl.header = header;
    dvl.time = std::stod(results[1]);
    dvl.dt1 = std::stof(results[2]);
    dvl.dt2 = std::stof(results[3]);
    dvl.beamDistance[0] = std::stof(results[8]);
    dvl.beamDistance[1] = std::stof(results[9]);
    dvl.beamDistance[2] = std::stof(results[10]);
    dvl.beamDistance[3] = std::stof(results[11]);
    dvl.batteryVoltage = std::stof(results[12]);
    dvl.speedSound = std::stof(results[13]);
    dvl.pressure = std::stof(results[14]);
    dvl.temp = std::stof(results[15]);

    if (dvl_pub_.getNumSubscribers() > 0)
      dvl_pub_.publish(dvl);

    if (isVelocityValid(std::stof(results[4])) and isVelocityValid(std::stof(results[5])) and
        isVelocityValid(std::stof(results[6])))
    {
      twist.header = header;
      twist.twist.covariance[0] = std::stof(results[7]);
      twist.twist.covariance[7] = std::stof(results[7]);
      twist.twist.covariance[14] = std::stof(results[7]);
      twist.twist.twist.linear.x = std::stof(results[4]);
      twist.twist.twist.linear.y = std::stof(results[5]);
      twist.twist.twist.linear.z = std::stof(results[6]);
      if (use_enu_)
      {
        twist.twist.twist.linear.y *= -1;
        twist.twist.twist.linear.z *= -1;
      }
      if (twist_pub_.getNumSubscribers() > 0)
        twist_pub_.publish(twist);
    }

    parseDvlStatus(hexStringToInt<unsigned long>(results[16].substr(2)),
                   status);
    if (dvl_status_pub_.getNumSubscribers() > 0)
      dvl_status_pub_.publish(status);
    return true;
  }

  return false;
}

void DvlInterface::parseDvlStatus(unsigned long num, nortek_dvl::DvlStatus &status)
{
  std::bitset<32> bset(num);

  status.header.stamp = ros::Time::now();

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

  if (bset[20])
  {
    status.proc_cap = 3;
  }
  else if (bset[21])
  {
    status.proc_cap = 6;
  }
  else if (bset[22])
  {
    status.proc_cap = 12;
  }

  int wakeupstate = bset[28] << 3 | bset[29] << 2 | bset[30] << 1 | bset[31];

  if (wakeupstate == 0b0010)
  {
    status.wakeup_state = "break";
  }
  else if (wakeupstate == 0b0011)
  {
    status.wakeup_state = "RTC Alarm";
  }
  else if (wakeupstate == 0b0000)
  {
    status.wakeup_state = "bad power";
  }
  else if (wakeupstate == 0b0001)
  {
    status.wakeup_state = "power applied";
  }
}

void DvlInterface::readParams()
{
  int port;
  private_nh_.getParam("address", address_);
  private_nh_.getParam("port", port);
  private_nh_.getParam("frame_id", frame_id_);
  private_nh_.getParam("use_enu", use_enu_);
  port_ = port;

  std::cout << "DVL PARAMS" << std::endl;
  std::cout << "-----------------" << std::endl;
  std::cout << "address: " << address_ << std::endl;
  std::cout << "port: " << port_ << std::endl;
  std::cout << "frame_id: " << frame_id_ << std::endl;
  std::cout << "use_enu: " << use_enu_ << std::endl;
  std::cout << "-----------------\n"
            << std::endl;
}

bool DvlInterface::isVelocityValid(double vel)
{
  return vel > -32; // -32.786 is invalid velocity
}
