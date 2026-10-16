#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>

/**
 * Minimal PointCloud2 → LaserScan converter.
 * Subscribes to /ugv/rslidar_points and publishes /scan.
 * Only depends on rclcpp + sensor_msgs (always available in ROS2 Humble).
 */
class PointcloudToLaserscan : public rclcpp::Node
{
public:
  PointcloudToLaserscan() : Node("pointcloud_to_laserscan")
  {
    this->declare_parameter("min_height",      -0.10);
    this->declare_parameter("max_height",       0.50);
    this->declare_parameter("range_min",        0.30);
    this->declare_parameter("range_max",       30.00);
    this->declare_parameter("angle_min",       -M_PI);
    this->declare_parameter("angle_max",        M_PI);
    this->declare_parameter("angle_increment",  0.00872665);  // ~0.5°
    this->declare_parameter("scan_time",        0.10);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ugv/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(&PointcloudToLaserscan::cloudCb, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(),
      "pointcloud_to_laserscan ready: /ugv/rslidar_points → /scan");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr      pub_;

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double min_height      = get_parameter("min_height").as_double();
    const double max_height      = get_parameter("max_height").as_double();
    const double range_min       = get_parameter("range_min").as_double();
    const double range_max       = get_parameter("range_max").as_double();
    const double angle_min       = get_parameter("angle_min").as_double();
    const double angle_max       = get_parameter("angle_max").as_double();
    const double angle_increment = get_parameter("angle_increment").as_double();
    const double scan_time       = get_parameter("scan_time").as_double();

    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header          = msg->header;
    scan->angle_min       = static_cast<float>(angle_min);
    scan->angle_max       = static_cast<float>(angle_max);
    scan->angle_increment = static_cast<float>(angle_increment);
    scan->time_increment  = 0.0f;
    scan->scan_time       = static_cast<float>(scan_time);
    scan->range_min       = static_cast<float>(range_min);
    scan->range_max       = static_cast<float>(range_max);

    const uint32_t ranges_size =
      static_cast<uint32_t>(std::ceil((angle_max - angle_min) / angle_increment));
    scan->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    // Iterate over the raw point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {continue;}
      if (z < min_height || z > max_height)                 {continue;}

      const float range = std::sqrt(x * x + y * y);
      if (range < range_min || range > range_max)           {continue;}

      const float angle = std::atan2(y, x);
      if (angle < angle_min || angle > angle_max)           {continue;}

      const int idx = static_cast<int>((angle - angle_min) / angle_increment);
      if (idx >= 0 && idx < static_cast<int>(ranges_size)) {
        if (range < scan->ranges[idx]) {
          scan->ranges[idx] = range;
        }
      }
    }

    pub_->publish(std::move(scan));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudToLaserscan>());
  rclcpp::shutdown();
  return 0;
}
