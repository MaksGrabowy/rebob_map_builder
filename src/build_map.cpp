#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include "rebob_map_builder/srv/save_pcd.hpp"

class PointCloudAssembler : public rclcpp::Node
{
public:
    PointCloudAssembler() : Node("point_cloud_assembler"){
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        final_map.setGeometry(grid_map::Length(30.0, 30.0), 0.05, grid_map::Position(0.0, 0.0));
        final_map.add("elevation");
        final_map.add("counts");
        final_map.add("elevation_max");
        final_map.add("elevation_min");
        final_map.add("vertical_dispersion"); // Max minus Min
        final_map.setFrameId("odom");

        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", qos_profile, std::bind(&PointCloudAssembler::cloudCallback, this, std::placeholders::_1));

        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dense_map", 1);
        pub_grid_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/dense_grid", 1);
        pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map_costmap", 1);

        save_srv_ = this->create_service<rebob_map_builder::srv::SavePCD>("save_dense_map", std::bind(&PointCloudAssembler::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Assembler Node Started. Service /save_dense_map is ready.");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        sensor_msgs::msg::PointCloud2 transformed_msg;
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            tf2::doTransform(*msg, transformed_msg, transform);
        }catch (const tf2::TransformException &ex){
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(transformed_msg, *current_cloud);
        *global_map_ += *current_cloud;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PassThrough<pcl::PointXYZI> pass_filter;
        pass_filter.setInputCloud(current_cloud); // cloud transformed to odom frame
        pass_filter.setFilterFieldName("z");      // slice along the z axis

        // set the limits
        pass_filter.setFilterLimits(-1.5, 1.5);   

        pass_filter.filter(*cropped_cloud);

        sor.setInputCloud(cropped_cloud);
        sor.setMeanK(10);            // Look at the 10 nearest neighbors
        sor.setStddevMulThresh(1.0); // If it's further than 1 standard deviation away, kill it
        sor.filter(*clean_cloud);

        final_map["counts"].setConstant(0.0);

        for (const auto& point : clean_cloud->points){
            grid_map::Position position(point.x, point.y);
            if (!final_map.isInside(position)) continue;

            float& max_z = final_map.atPosition("elevation_max", position);
            float& min_z = final_map.atPosition("elevation_min", position);

            // Update MAX Z
            if (std::isnan(max_z) || point.z > max_z) {
                max_z = point.z;
            }
            
            // Update MIN Z
            if (std::isnan(min_z) || point.z < min_z) {
                min_z = point.z;
            }

            // Calculate your first condition: The Vertical Stack
            // If Max and Min are far apart, it's a vertical obstacle!
            float& dispersion = final_map.atPosition("vertical_dispersion", position);
            dispersion = max_z - min_z;
        }
        for (const auto& point : clean_cloud->points){
            grid_map::Position position(point.x, point.y);

            if (!final_map.isInside(position)){
                continue;
            }

            float& cell_elevation = final_map.atPosition("elevation", position);
            float& cell_count = final_map.atPosition("counts", position);
            float& dispersion = final_map.atPosition("vertical_dispersion", position);
            cell_count += 1;

            // update the cell
            // if (std::isnan(cell_elevation) || abs(point.z - cell_elevation) > 0.1 || cell_count > 5) 
            // {
            //     cell_elevation = point.z;
            // }
            if (std::isnan(cell_elevation) || point.z > cell_elevation){
                cell_elevation = point.z;
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
        voxel_filter.filter(*filtered_map);
        global_map_ = filtered_map;

        nav_msgs::msg::OccupancyGrid costmap_msg;

        grid_map::GridMapRosConverter::toOccupancyGrid(
            final_map,        // Your grid map object
            "vertical_dispersion",   // The layer you want to export
            0.0f,               // The value that should map to 0 (Free Space)
            0.5f,               // The value that should map to 100 (Obstacle)
            costmap_msg         // The message to populate
        );

        costmap_msg.header.frame_id = "odom";
        costmap_msg.header.stamp = this->get_clock()->now();
        pub_costmap_->publish(costmap_msg);

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.frame_id = "odom";
        map_msg.header.stamp = this->get_clock()->now();
        pub_map_->publish(map_msg);

        pub_grid_->publish(grid_map::GridMapRosConverter::toMessage(final_map));
    }

    // service Callback for saving the map
    void saveMapCallback(const std::shared_ptr<rebob_map_builder::srv::SavePCD::Request> request, std::shared_ptr<rebob_map_builder::srv::SavePCD::Response> response){
        if (global_map_->empty()){
            response->success = false;
            response->message = "The map is currently empty. Nothing to save.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");

        std::filesystem::path dir(request->directory_path);
        std::filesystem::path file("pointcloud-" + ss.str() + ".pcd");
        std::filesystem::path full_path = dir / file;

        // save the map
        try{
            pcl::io::savePCDFileBinary(full_path.string(), *global_map_);
            response->success = true;
            response->message = "Saved map to: " + full_path.string();
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
        catch (const std::exception& e){
            response->success = false;
            response->message = "Failed to save file: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    // pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map_costmap", 1);

    rclcpp::Service<rebob_map_builder::srv::SavePCD>::SharedPtr save_srv_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    grid_map::GridMap final_map;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAssembler>());
    rclcpp::shutdown();
    return 0;
}