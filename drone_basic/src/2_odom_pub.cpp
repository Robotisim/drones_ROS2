#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
    : Node("odometry_publisher")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/drone/gt_pose", 10, std::bind(&OdometryPublisher::pose_callback, this, std::placeholders::_1));
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/drone/gt_vel", 10, std::bind(&OdometryPublisher::vel_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/drone/imu", 10, std::bind(&OdometryPublisher::imu_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        odom_msg_.pose.pose = *msg;
        publish_odom();
    }

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        odom_msg_.twist.twist = *msg;
        publish_odom();
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        odom_msg_.pose.pose.orientation = msg->orientation;
        publish_odom();
    }

    void publish_odom()
    {
        odom_msg_.header.stamp = this->now();
        odom_pub_->publish(odom_msg_);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = odom_msg_.pose.pose.position.x;
        t.transform.translation.y = odom_msg_.pose.pose.position.y;
        t.transform.translation.z = odom_msg_.pose.pose.position.z;
        t.transform.rotation = odom_msg_.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
