#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "unitree_lidar_sdk_pcl.h"

using namespace std::chrono_literals;

class EvrenosUnitreeL1UdpNode : public rclcpp::Node
{
public:
  EvrenosUnitreeL1UdpNode()
  : Node("evrenos_unitree_l1_udp")
  {
    lidar_ip_ = declare_parameter<std::string>(
      "lidar_ip", "192.168.1.62");

    local_ip_ = declare_parameter<std::string>(
      "local_ip", "192.168.1.2");

    lidar_port_ = declare_parameter<int>(
      "lidar_port", 6101);

    local_port_ = declare_parameter<int>(
      "local_port", 6201);

    cloud_scan_num_ = declare_parameter<int>(
      "cloud_scan_num", 18);

    rotate_yaw_bias_ = declare_parameter<double>(
      "rotate_yaw_bias", 0.0);

    range_scale_ = declare_parameter<double>(
      "range_scale", 0.001);

    range_bias_ = declare_parameter<double>(
      "range_bias", 0.0);

    range_max_ = declare_parameter<double>(
      "range_max", 50.0);

    range_min_ = declare_parameter<double>(
      "range_min", 0.0);

    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/unilidar/cloud");

    cloud_frame_ = declare_parameter<std::string>(
      "cloud_frame", "unilidar_lidar");

    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/unilidar/imu");

    imu_frame_ = declare_parameter<std::string>(
      "imu_frame", "unilidar_imu");

    cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_,
        rclcpp::SensorDataQoS());

    imu_pub_ =
      create_publisher<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS());

    lidar_ = createUnitreeLidarReader();

    if (lidar_ == nullptr)
    {
      throw std::runtime_error(
        "createUnitreeLidarReader() returned nullptr");
    }

    RCLCPP_INFO(
      get_logger(),
      "Initializing Unitree L1 UDP");

    RCLCPP_INFO(
      get_logger(),
      "LiDAR  : %s:%d",
      lidar_ip_.c_str(),
      lidar_port_);

    RCLCPP_INFO(
      get_logger(),
      "Local  : %s:%d",
      local_ip_.c_str(),
      local_port_);

    const int ret = lidar_->initializeUDP(
      static_cast<uint16_t>(cloud_scan_num_),

      static_cast<unsigned short>(lidar_port_),
      lidar_ip_,

      static_cast<unsigned short>(local_port_),
      local_ip_,

      static_cast<float>(rotate_yaw_bias_),
      static_cast<float>(range_scale_),
      static_cast<float>(range_bias_),
      static_cast<float>(range_max_),
      static_cast<float>(range_min_)
    );

    if (ret != 0)
    {
      throw std::runtime_error(
        "Unitree L1 initializeUDP() failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "Unitree L1 UDP initialization successful");

    RCLCPP_INFO(
      get_logger(),
      "Cloud topic: %s",
      cloud_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "IMU topic: %s",
      imu_topic_.c_str());

    // Unitree SDK örneği runParse()'ın en az ~1500 Hz
    // çağrılmasını öneriyor. 500 us => yaklaşık 2000 Hz.
    timer_ = create_wall_timer(
      std::chrono::microseconds(500),
      std::bind(
        &EvrenosUnitreeL1UdpNode::poll_lidar,
        this));
  }

  ~EvrenosUnitreeL1UdpNode() override
  {
    if (lidar_ != nullptr)
    {
      lidar_->closeUDP();
    }
  }

private:
  builtin_interfaces::msg::Time make_stamp(double stamp)
  {
    builtin_interfaces::msg::Time msg;

    const auto sec =
      static_cast<int32_t>(std::floor(stamp));

    const double fractional =
      stamp - static_cast<double>(sec);

    msg.sec = sec;
    msg.nanosec =
      static_cast<uint32_t>(fractional * 1e9);

    return msg;
  }

  void poll_lidar()
  {
    if (lidar_ == nullptr)
    {
      return;
    }

    const MessageType result =
      lidar_->runParse();

    if (result == IMU)
    {
      const auto &imu =
        lidar_->getIMU();

      sensor_msgs::msg::Imu msg;

      msg.header.stamp =
        make_stamp(imu.stamp);

      msg.header.frame_id =
        imu_frame_;

      msg.orientation.x =
        imu.quaternion[0];

      msg.orientation.y =
        imu.quaternion[1];

      msg.orientation.z =
        imu.quaternion[2];

      msg.orientation.w =
        imu.quaternion[3];

      msg.angular_velocity.x =
        imu.angular_velocity[0];

      msg.angular_velocity.y =
        imu.angular_velocity[1];

      msg.angular_velocity.z =
        imu.angular_velocity[2];

      msg.linear_acceleration.x =
        imu.linear_acceleration[0];

      msg.linear_acceleration.y =
        imu.linear_acceleration[1];

      msg.linear_acceleration.z =
        imu.linear_acceleration[2];

      imu_pub_->publish(msg);

      return;
    }

    if (result == POINTCLOUD)
    {
      const auto &cloud =
        lidar_->getCloud();

      pcl::PointCloud<PointType>::Ptr pcl_cloud(
        new pcl::PointCloud<PointType>());

      transformUnitreeCloudToPCL(
        cloud,
        pcl_cloud);

      sensor_msgs::msg::PointCloud2 msg;

      pcl::toROSMsg(
        *pcl_cloud,
        msg);

      msg.header.stamp =
        make_stamp(cloud.stamp);

      msg.header.frame_id =
        cloud_frame_;

      cloud_pub_->publish(msg);
    }
  }

  UnitreeLidarReader *lidar_{nullptr};

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string lidar_ip_;
  std::string local_ip_;

  int lidar_port_;
  int local_port_;

  int cloud_scan_num_;

  double rotate_yaw_bias_;
  double range_scale_;
  double range_bias_;
  double range_max_;
  double range_min_;

  std::string cloud_topic_;
  std::string cloud_frame_;

  std::string imu_topic_;
  std::string imu_frame_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node =
      std::make_shared<EvrenosUnitreeL1UdpNode>();

    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "evrenos_unitree_l1_udp"),
      "%s",
      e.what());

    rclcpp::shutdown();

    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
