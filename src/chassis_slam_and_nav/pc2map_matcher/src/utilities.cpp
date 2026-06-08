#include "../include/utilities.h"

nav_msgs::msg::OccupancyGrid pc2mapUtilities::loadMapFromYamlAndPgm(const std::string &yaml_file, const std::string &pgm_file)
{
    YAML::Node config = YAML::LoadFile(yaml_file);

    // 初始化 OccupancyGrid 对象
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = config["resolution"].as<double>();

    // 从 PGM 文件获取图像宽度和高度
    std::ifstream pgm_file_stream(pgm_file, std::ios::binary);
    if (!pgm_file_stream.is_open())
    {
        std::cerr << "Failed to open PGM file" << std::endl;
        return map;
    }

    std::string pgm_header;
    std::getline(pgm_file_stream, pgm_header);

    // 忽略可能的注释行
    while (pgm_header[0] == '#')
    {
        std::getline(pgm_file_stream, pgm_header);
    }

    // PGM 文件头格式中宽度和高度在 "P5" 后面
    int width, height;
    pgm_file_stream >> width >> height;
    pgm_file_stream.ignore(256, '\n');  // 跳过最大值行（通常是 255）
    RCLCPP_INFO(rclcpp::get_logger("icp_alignment"),"%d,%d",width,height);
    // 设置地图的宽度和高度
    map.info.width = width;
    map.info.height = height;

    // 设置地图的原点（假设原点信息是从 YAML 配置文件中获取的）
    map.info.origin.position.x = config["origin"][0].as<double>();  // position.x
    map.info.origin.position.y = config["origin"][1].as<double>();  // position.y
    map.info.origin.position.z = config["origin"][2].as<double>();  // position.z

    // 如果 YAML 文件中没有 orientation，可以使用默认值 (0, 0, 0, 1)
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    // 读取 PGM 数据并填充到 map.data 中
    map.data.resize(width * height);
    for (int i = 0; i < width * height; ++i)
    {
        unsigned char pixel_value;
        pgm_file_stream.read(reinterpret_cast<char*>(&pixel_value), sizeof(pixel_value));
        map.data[i] = static_cast<int8_t>(pixel_value);  // PGM 的像素值应该转换为 OccupancyGrid 中的值
    }

    pgm_file_stream.close();

    return map;
}

pc2mapUtilities::pc2mapUtilities()
{
    grid_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    liosam_2d_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    liosam_3d_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
}
int pc2mapUtilities::read_from_pcd(std::string pcd_file_path)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path, *liosam_3d_pc) == -1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("icp_alignment"), "Couldn't read the PCD file");
      return 0;
    }
    else RCLCPP_INFO(rclcpp::get_logger("icp_alignment"),"read successed:)");
    return 1;
}

void pc2mapUtilities::process_original_liosam_pc(std::string pcd_file_path)
{
    if(read_from_pcd(pcd_file_path) == 1)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto& point : liosam_3d_pc->points)
        {
            if (point.z >= 0.1 && point.z <= 2.5)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        liosam_3d_pc = filtered_cloud;
    }
    RCLCPP_INFO(rclcpp::get_logger("icp_alignment"), "Filtered pointcloud with size: %lu", liosam_3d_pc->points.size());
    return ;
}

void pc2mapUtilities::gridToPointCloud(const nav_msgs::msg::OccupancyGrid &map)
{
    double res = map.info.resolution;
    double ox = map.info.origin.position.x;
    double oy = map.info.origin.position.y;

    for (unsigned int y = 0; y < map.info.height; y++)
    {
      for (unsigned int x = 0; x < map.info.width; x++)
      {
        int idx = x + y * map.info.width;
        if (map.data[idx] > 50)  
        {
          pcl::PointXYZI p;
          p.x = ox + (x + 0.5) * res;
          p.y = oy + (y + 0.5) * res;
          p.z = 0.0;
          grid_pc->points.push_back(p);
        }
      }
    }
    return;
}

void pc2mapUtilities::project3Dto2D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3d)
{
    for (auto &p : cloud_3d->points)
    {
      if (p.z > 0.1 && p.z < 2.5)
      {
        pcl::PointXYZI q;
        q.x = p.x;
        q.y = p.y;
        q.z = 0.0;
        liosam_2d_pc->points.push_back(q);
      }
    }
    return;
}

Eigen::Matrix4f pc2mapUtilities::align_grid_pc_to_liosam_2d_pc(pcl::PointCloud<pcl::PointXYZI>::Ptr grid_pc,pcl::PointCloud<pcl::PointXYZI>::Ptr liosam_2d_pc)
{
    Eigen::Matrix4f icp_tf;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr loser_pc(new pcl::PointCloud<pcl::PointXYZI>());
    icp.setInputSource(liosam_2d_pc);
    icp.setInputTarget(grid_pc);
    icp.align(*loser_pc);
    if (icp.hasConverged())
    {
        RCLCPP_INFO(rclcpp::get_logger("icp_alignment"),"ICP converge success:)");
        icp_tf = icp.getFinalTransformation();
        return icp_tf;
    }
    else
    {
        // std::cout << "ICP did not converge." << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("icp_alignment"),"ICP did not converge :(");
    }
}

Eigen::Matrix4f pc2mapUtilities::icp_alignment(const nav_msgs::msg::OccupancyGrid &map)
{
    project3Dto2D(liosam_3d_pc);
    gridToPointCloud(map);
    return align_grid_pc_to_liosam_2d_pc(grid_pc,liosam_2d_pc);
}

Eigen::Matrix4f pc2mapUtilities::align_and_transform(std::string pcd_file_path,const nav_msgs::msg::OccupancyGrid &map)
{
    process_original_liosam_pc(pcd_file_path);
    return icp_alignment(map);
}



pcl::PointCloud<pcl::PointXYZI>::Ptr pc2mapUtilities::get_pc()
{
    return liosam_3d_pc;
}
