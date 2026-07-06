#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class GpsOdomTester : public rclcpp::Node
{
public:
    GpsOdomTester() : Node("navsat_tester")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        gps_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/gpsd_client", 10,
            std::bind(&GpsOdomTester::gpsOdomCallback, this, _1));

        republished_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/gps_tester", 10);

        RCLCPP_INFO(this->get_logger(), "Test GPS node (odom -> gps_tester) opened.");
    }

private:
    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto tester_odom = *msg;
        tester_odom.child_frame_id = "gps_tester";

        republished_odom_pub_->publish(tester_odom);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = msg->header.frame_id;
        tf_msg.child_frame_id = "gps_tester";

        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;

        tf_msg.transform.rotation.x = msg->pose.pose.orientation.x;
        tf_msg.transform.rotation.y = msg->pose.pose.orientation.y;
        tf_msg.transform.rotation.z = msg->pose.pose.orientation.z;
        tf_msg.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr republished_odom_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsOdomTester>());
    rclcpp::shutdown();
    return 0;
}