#ifndef UTILITIES_H
#define UTILITIES_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"
#include <iostream>
#include <fstream>
#include <string>
#include "yaml-cpp/yaml.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>

class pc2mapUtilities{
    public:
    pc2mapUtilities();
    int read_from_pcd(std::string pcd_file_path);
    void process_original_liosam_pc(std::string pcd_file_path);
    void gridToPointCloud(const nav_msgs::msg::OccupancyGrid &map);
    void project3Dto2D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3d);
    Eigen::Matrix4f align_grid_pc_to_liosam_2d_pc(pcl::PointCloud<pcl::PointXYZI>::Ptr grid_pc,pcl::PointCloud<pcl::PointXYZI>::Ptr liosam_2d_pc);
    Eigen::Matrix4f icp_alignment(const nav_msgs::msg::OccupancyGrid &map);
    Eigen::Matrix4f align_and_transform(std::string pcd_file_path,const nav_msgs::msg::OccupancyGrid &map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr get_pc();
    nav_msgs::msg::OccupancyGrid loadMapFromYamlAndPgm(const std::string &yaml_file, const std::string &pgm_file);

    private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr liosam_2d_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr liosam_3d_pc;
    

};

#endif