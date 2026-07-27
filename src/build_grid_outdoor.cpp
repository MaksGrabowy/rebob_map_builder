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
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Nowe biblioteki do ekstrakcji podłoża
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include <map>
#include <cmath>
#include <utility>

#include "rebob_map_builder/srv/save_pcd.hpp"

class PointCloudAssembler : public rclcpp::Node
{
public:
    PointCloudAssembler() : Node("point_cloud_assembler"){
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        //initiating a gridmap object
        final_map.setGeometry(grid_map::Length(200.0, 200.0), 0.1, grid_map::Position(0.0, 0.0));
        final_map.add("elevation");
        final_map.add("traversability");
        final_map.setFrameId("map");

        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.transient_local();
        qos_profile.reliable();

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_odometry/localmap_points", qos_profile, std::bind(&PointCloudAssembler::cloudCallback, this, std::placeholders::_1));

        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry/localmap_filtered", 1);
        
        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry/localmap_ground", 1);
        
        pub_grid_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/lidar_odometry/localmap_gridmap", 1);
        pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/lidar_odometry/localmap_costmap", 1);

        // save_srv_ = this->create_service<rebob_map_builder::srv::SavePCD>("save_dense_map", std::bind(&PointCloudAssembler::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Assembler Node Started. PMF Filter Active.");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "--- New PointCloud received ---");

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *current_cloud);
        *global_map_ = *current_cloud;

        RCLCPP_INFO(this->get_logger(), "[1/10] Received a PointCloud: %zu points", current_cloud->points.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        sor.setInputCloud(current_cloud);
        sor.setMeanK(50);            
        sor.setStddevMulThresh(1.0); 
        sor.filter(*clean_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> pre_voxel_filter;
        pre_voxel_filter.setInputCloud(clean_cloud);
        pre_voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        pre_voxel_filter.filter(*downsampled_cloud);

        RCLCPP_INFO(this->get_logger(), "[2/10] After initial filtering (SOR + VoxelGrid): %zu points", downsampled_cloud->points.size());

        //surface extraction - PMF
        RCLCPP_INFO(this->get_logger(), "[3/10] Surface extraction");
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud(downsampled_cloud);
        pmf.setCellSize(0.1f); //
        pmf.setMaxWindowSize(10);  //20            
        pmf.setSlope(1.0f);                   
        pmf.setInitialDistance(0.2f); //0.2        
        pmf.setMaxDistance(1.0f);             
        pmf.extract(ground_indices->indices);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(downsampled_cloud);
        extract.setIndices(ground_indices);
        
        extract.setNegative(false);
        extract.filter(*ground_cloud);   
        
        extract.setNegative(true);
        extract.filter(*obstacle_cloud); 

        RCLCPP_INFO(this->get_logger(), "      -> Ground: %zu points | Obstacles: %zu points", ground_cloud->points.size(), obstacle_cloud->points.size());

        // local plane detection
        // dividing ground pointcloud into small planes so that we can delete points that "fell" under the ground
        RCLCPP_INFO(this->get_logger(), "[4/10] Local plane fitting - RANSAC in 10x10m sectors");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_ground(new pcl::PointCloud<pcl::PointXYZ>());

        if (!ground_cloud->empty()) {
            float patch_size = 10.0f;
            
            std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZ>::Ptr> grid_patches;

            // dividing into sectors
            for (const auto& pt : ground_cloud->points) {
                int grid_x = static_cast<int>(std::floor(pt.x / patch_size));
                int grid_y = static_cast<int>(std::floor(pt.y / patch_size));
                auto key = std::make_pair(grid_x, grid_y);
                
                // if it does not exist, create new pointcloud
                if (grid_patches.find(key) == grid_patches.end()) {
                    grid_patches[key].reset(new pcl::PointCloud<pcl::PointXYZ>());
                }
                grid_patches[key]->points.push_back(pt);
            }

            // RANSAC configuration
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            
            seg.setMaxIterations(100); 
            
            // ground tolerance
            seg.setDistanceThreshold(0.20); 

            for (const auto& [key, patch_cloud] : grid_patches) {
                if (patch_cloud->points.size() < 10) {
                    *cleaned_ground += *patch_cloud;
                    continue;
                }

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                
                seg.setInputCloud(patch_cloud);
                seg.segment(*inliers, *coefficients);

                if (inliers->indices.size() > 0) {
                    // 
                    for (int idx : inliers->indices) {
                        cleaned_ground->points.push_back(patch_cloud->points[idx]);
                    }
                } else {
                    *cleaned_ground += *patch_cloud;
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "      -> Sectors cleaned. Remaining ground points: %zu (removed %zu)", 
                        cleaned_ground->points.size(), ground_cloud->points.size() - cleaned_ground->points.size());
        }


        // ground upsampling
        RCLCPP_INFO(this->get_logger(), "[5/10] Ground upsampling and filling gaps");
        
        if (!cleaned_ground->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_ground(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
            
            mls.setInputCloud(cleaned_ground);
            mls.setComputeNormals(false);
            mls.setSearchMethod(tree);
            
            mls.setSearchRadius(1.0); 
            
            mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
            mls.setDilationVoxelSize(0.15); 
            mls.setDilationIterations(2); 
            
            mls.process(*smoothed_ground);
            
            ground_cloud = smoothed_ground;
            
            RCLCPP_INFO(this->get_logger(), "      -> Ground reconstruction. Increased points: %zu", 
                        ground_cloud->points.size());
        }

        
        // removing underground artifacts
        RCLCPP_INFO(this->get_logger(), "[6/10] Filtering artifacts using KdTree");
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        float underground_tolerance = 0.01f;

        if (!ground_cloud->empty()) {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(ground_cloud);

            for (const auto& point : obstacle_cloud->points) {
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                // looking for closes point K=1
                if (kdtree.nearestKSearch(point, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    float nearest_ground_z = ground_cloud->points[pointIdxRadiusSearch[0]].z;
                    
                    if (point.z >= (nearest_ground_z - underground_tolerance)) {
                        cleaned_obstacle_cloud->points.push_back(point);
                    }
                } else {
                    cleaned_obstacle_cloud->points.push_back(point);
                }
            }
        } else {
            *cleaned_obstacle_cloud = *obstacle_cloud;
        }

        RCLCPP_INFO(this->get_logger(), "      -> Removed underground mess. Remaining obstacle points: %zu", cleaned_obstacle_cloud->points.size());


        RCLCPP_INFO(this->get_logger(), "[7/10] Removing floating artifacts - ROR");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_obstacle_cloud_sparse(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cleaned_obstacle_cloud);

        // looking for points in this radius
        ror.setRadiusSearch(0.15); 
        ror.setMinNeighborsInRadius(10); 

        ror.filter(*cleaned_obstacle_cloud_sparse);

        RCLCPP_INFO(this->get_logger(), "      -> Floating points removed. Remaining obstacle points: %zu", cleaned_obstacle_cloud_sparse->points.size());

        RCLCPP_INFO(this->get_logger(), "[8/10] Elevation map creation");
        // filling ground elevation (stage 1)
        for (const auto& point : ground_cloud->points) {
            grid_map::Position position(point.x, point.y);
            if (!final_map.isInside(position)) continue;

            float& cell_elevation = final_map.atPosition("elevation", position);
            if (std::isnan(cell_elevation)) {
                cell_elevation = point.z;
            } else {
                // EMA filter, alpha = 0.2
                cell_elevation = 0.8f * cell_elevation + 0.2f * point.z;
            }
        }

        // second inpainting
        int inpainting_iterations = 3; 
        for (int i = 0; i < inpainting_iterations; ++i) {
            grid_map::GridMap map_copy = final_map;
            for (grid_map::GridMapIterator iterator(final_map); !iterator.isPastEnd(); ++iterator) {
                if (!map_copy.isValid(*iterator, "elevation")) {
                    grid_map::Position current_pos;
                    final_map.getPosition(*iterator, current_pos);

                    float min_z = std::numeric_limits<float>::infinity();
                    int valid_neighbors = 0;
                    for (grid_map::CircleIterator neighbor(map_copy, current_pos, 0.20); !neighbor.isPastEnd(); ++neighbor) {
                        if (map_copy.isValid(*neighbor, "elevation")) {
                            float val = map_copy.at("elevation", *neighbor);
                            if (val < min_z) min_z = val;
                            valid_neighbors++;
                        }
                    }
                    if (valid_neighbors >= 2) final_map.at("elevation", *iterator) = min_z;
                }
            }
        }

        // // ---------------------------------------------------------
        // // [4.5/6] WYPEŁNIANIE MAPY TERENU (Faza 2: Przywracanie ścian i pni)
        // // ---------------------------------------------------------
        // RCLCPP_INFO(this->get_logger(), "[4.5/6] Rekonstrukcja ścian, pni i stromych zboczy...");
        // float robot_height = 0.8f;         

        // for (const auto& point : obstacle_cloud->points) {
        //     grid_map::Position pos(point.x, point.y);
            
        //     if (!final_map.isInside(pos)) continue;
        //     float ground_z = final_map.atPosition("elevation", pos);
        //     if (std::isnan(ground_z)) continue; // Ignorujemy, jeśli nadal nie ma bazy gruntu

        //     float relative_h = point.z - ground_z; 

        //     // Jeśli to NIE JEST korona drzewa/wisząca gałąź (wysokość < robot_height)
        //     // Wtapiamy to z powrotem w mapę wysokości (Elevation)!
        //     if (relative_h <= robot_height) {
        //         float& cell_elevation = final_map.atPosition("elevation", pos);
        //         if (point.z > cell_elevation) {
        //             cell_elevation = point.z; // Nadpisujemy grunt ścianą/pniem
        //         }
        //     }
        // }

        // ---------------------------------------------------------
        // [4.5/6] WYPEŁNIANIE MAPY TERENU (Faza 2: Przywracanie ścian i pni)
        // ---------------------------------------------------------

        // reconstruction of obstacles
        float robot_height = 0.8f;         
        underground_tolerance = 0.05f;

        for (const auto& point : cleaned_obstacle_cloud_sparse->points) {
            grid_map::Position pos(point.x, point.y);
            
            if (!final_map.isInside(pos)) continue;
            float ground_z = final_map.atPosition("elevation", pos);
            if (std::isnan(ground_z)) continue;

            // ignoring points under ground surface
            if (point.z < (ground_z - underground_tolerance)) {
                continue; 
            }

            float relative_h = point.z - ground_z; 

            if (relative_h <= robot_height) {
                float& cell_elevation = final_map.atPosition("elevation", pos);
                // not using EMA filter (but we can)
                if (point.z > cell_elevation) {
                    cell_elevation = point.z; 
                }
            }
        }


        // traversability
        RCLCPP_INFO(this->get_logger(), "[9/10] Traversability analysis");
        
        // because standing obstacles are included in elevation map we can directly compute traversability using final_map
        double search_radius = 0.35; // 0.15 
        double step_threshold = 0.05; // max robot step

        grid_map::GridMap map_for_trav = final_map; // copy for analysis

        for (grid_map::GridMapIterator iterator(final_map); !iterator.isPastEnd(); ++iterator) {
            final_map.at("traversability", *iterator) = 0.0f; // default for the ground

            if (!map_for_trav.isValid(*iterator, "elevation")) continue; 

            float center_z = map_for_trav.at("elevation", *iterator);
            
            grid_map::Position current_pos;
            map_for_trav.getPosition(*iterator, current_pos);

            float sum_z = 0.0f;
            int valid_cells = 0;

            for (grid_map::CircleIterator window_iterator(map_for_trav, current_pos, search_radius); 
                !window_iterator.isPastEnd(); ++window_iterator) {
                
                if (map_for_trav.isValid(*window_iterator, "elevation")) {
                    sum_z += map_for_trav.at("elevation", *window_iterator);
                    valid_cells++;
                }
            }

            if (valid_cells > 0) {
                float mean_z = sum_z / valid_cells;

                // if cell is higher than the threshold
                if ((center_z - mean_z) > step_threshold) {
                    final_map.at("traversability", *iterator) = 100.0f;
                }
            }
        }

        // assembling final pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_clean_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *final_clean_cloud += *ground_cloud;
        *final_clean_cloud += *cleaned_obstacle_cloud_sparse;
        
        global_map_ = final_clean_cloud;

        // publishing
        RCLCPP_INFO(this->get_logger(), "[10/10] Publishing to topics");
        auto current_time = this->get_clock()->now();

        // costmap
        nav_msgs::msg::OccupancyGrid costmap_msg;
        grid_map::GridMapRosConverter::toOccupancyGrid(
            final_map, "traversability", 0.0f, 100.0f, costmap_msg);
        costmap_msg.header.frame_id = "map";
        costmap_msg.header.stamp = current_time;
        pub_costmap_->publish(costmap_msg);

        // gridmap
        pub_grid_->publish(grid_map::GridMapRosConverter::toMessage(final_map));

        // global map
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = current_time;
        pub_map_->publish(map_msg);

        // ground points
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_cloud, ground_msg);
        ground_msg.header.frame_id = "map";
        ground_msg.header.stamp = current_time;
        pub_ground_->publish(ground_msg);

        // end of time meas.
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "--- Finished processing: %ld ms ---\n", duration);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;

    rclcpp::Service<rebob_map_builder::srv::SavePCD>::SharedPtr save_srv_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    grid_map::GridMap final_map;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAssembler>());
    rclcpp::shutdown();
    return 0;
}