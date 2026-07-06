        // this->declare_parameter<double>("base_to_imu_x", -0.18);
        // this->declare_parameter<double>("base_to_imu_y", 0.11);
        // this->declare_parameter<double>("base_to_imu_z", 0.35);
        // this->declare_parameter<double>("base_to_imu_roll", 0.0);
        // this->declare_parameter<double>("base_to_imu_pitch", 0.0);
        // this->declare_parameter<double>("base_to_imu_yaw", 3.1415);

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class ImuOdometryTransformer : public rclcpp::Node
{
public:
    ImuOdometryTransformer() : Node("imu_odometry_transformer_node")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        this->declare_parameter<double>("base_to_imu_x", -0.18);
        this->declare_parameter<double>("base_to_imu_y", 0.11);
        this->declare_parameter<double>("base_to_imu_z", 0.35);
        this->declare_parameter<double>("base_to_imu_roll", 0.0);
        this->declare_parameter<double>("base_to_imu_pitch", 0.0);
        this->declare_parameter<double>("base_to_imu_yaw", 3.1415);

        this->get_parameter("base_to_imu_x", base_to_imu_x_);
        this->get_parameter("base_to_imu_y", base_to_imu_y_);
        this->get_parameter("base_to_imu_z", base_to_imu_z_);
        this->get_parameter("base_to_imu_roll", base_to_imu_roll_);
        this->get_parameter("base_to_imu_pitch", base_to_imu_pitch_);
        this->get_parameter("base_to_imu_yaw", base_to_imu_yaw_);

        tf2::Vector3 translation_base_imu(base_to_imu_x_, base_to_imu_y_, base_to_imu_z_);
        tf2::Quaternion rotation_base_imu;
        rotation_base_imu.setRPY(base_to_imu_roll_, base_to_imu_pitch_, base_to_imu_yaw_);
        T_base_imu_ = tf2::Transform(rotation_base_imu, translation_base_imu);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/imu", 
            rclcpp::SensorDataQoS(), 
            std::bind(&ImuOdometryTransformer::odomCallback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/fixed_lio", 10);

        RCLCPP_INFO(this->get_logger(), "Transformer odometrii IMU z emulacją TF uruchomiony.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Vector3 translation_odom_imu(
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y, 
            msg->pose.pose.position.z);
        
        tf2::Quaternion rotation_odom_imu(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Transform T_odom_imu(rotation_odom_imu, translation_odom_imu);

        tf2::Transform T_odom_base = T_odom_imu * T_base_imu_.inverse();

        auto transformed_odom = nav_msgs::msg::Odometry();
        transformed_odom.header.stamp = msg->header.stamp;
        transformed_odom.header.frame_id = "odom";
        transformed_odom.child_frame_id = "base_link";

        transformed_odom.pose.pose.position.x = T_odom_base.getOrigin().x();
        transformed_odom.pose.pose.position.y = T_odom_base.getOrigin().y();
        transformed_odom.pose.pose.position.z = T_odom_base.getOrigin().z();

        transformed_odom.pose.pose.orientation.x = T_odom_base.getRotation().x();
        transformed_odom.pose.pose.orientation.y = T_odom_base.getRotation().y();
        transformed_odom.pose.pose.orientation.z = T_odom_base.getRotation().z();
        transformed_odom.pose.pose.orientation.w = T_odom_base.getRotation().w();

        for (int i = 0; i < 6; ++i) {
            transformed_odom.pose.covariance[i * 7] = 0.01;
        }

        odom_pub_->publish(transformed_odom);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link_tester";

        tf_msg.transform.translation.x = T_odom_base.getOrigin().x();
        tf_msg.transform.translation.y = T_odom_base.getOrigin().y();
        tf_msg.transform.translation.z = T_odom_base.getOrigin().z();

        tf_msg.transform.rotation.x = T_odom_base.getRotation().x();
        tf_msg.transform.rotation.y = T_odom_base.getRotation().y();
        tf_msg.transform.rotation.z = T_odom_base.getRotation().z();
        tf_msg.transform.rotation.w = T_odom_base.getRotation().w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    double base_to_imu_x_, base_to_imu_y_, base_to_imu_z_;
    double base_to_imu_roll_, base_to_imu_pitch_, base_to_imu_yaw_;
    
    tf2::Transform T_base_imu_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdometryTransformer>());
    rclcpp::shutdown();
    return 0;
}