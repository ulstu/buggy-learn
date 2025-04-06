#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lidar_odometry/lidar_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      std::string scan_topic_name;
      std::string odom_topic_name;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
      RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
      RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
      RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());

      lidar_odometry_ptr = std::make_shared<LidarOdometry>(max_correspondence_distance, transformation_epsilon, maximum_iterations);
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 1);
      scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, 1, std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      );
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;

      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

      geometry_msgs::msg::TransformStamped tf;
      nav_msgs::msg::Odometry odom;

      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 0.25);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);
        this->declare_parameter<std::string>("scan_topic_name", "scan");
        this->declare_parameter<std::string>("odom_topic_name", "odom");
      }

      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto point_cloud_msg = laser2cloudmsg(scan_msg);
        auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;

        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry();
      }

      void publish_odometry() {
        auto state = lidar_odometry_ptr->get_state();

        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = state->pose.translation().x();
        tf.transform.translation.y = state->pose.translation().y();
        tf.transform.translation.z = 0.0; // state->pose.translation().z();

        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose = Eigen::toMsg(state->pose);
        odom.pose.pose.position.z = 0.0;

        tf.transform.rotation = odom.pose.pose.orientation;
        odom.twist.twist = Eigen::toMsg(state->velocity);

      	tf_broadcaster->sendTransform(tf);
        odom_publisher->publish(odom);
      }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
