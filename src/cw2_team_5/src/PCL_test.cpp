// #include "cw2_class.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include "detect_object.h"


int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pcl_test_node");
    ros::NodeHandle nh;
    
    // 检查命令行参数
    // if (argc < 2) {
    //     std::cout << "用法: " << argv[0] << " <pcd_file_path>" << std::endl;
    //     return -1;
    // }
    
    argv[1] = "./task3_scan_result.pcd";

    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // 加载PCD文件
    std::cout << "正在加载点云文件: " << argv[1] << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloud) == -1) {
        std::cerr << "无法加载文件: " << argv[1] << std::endl;
        return -1;
    }
    
    std::cout << "成功加载点云，包含 " << cloud->points.size() << " 个点。" << std::endl;
    
    // 过滤NaN值
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);
    std::cout << "过滤NaN后点云包含 " << cloud_filtered->points.size() << " 个点。" << std::endl;
    
    
    // 创建检测结果向量
    std::vector<ShapeDetectionResult> detected_objects;
    
    // 执行物体检测
    std::cout << "开始执行物体检测..." << std::endl;
    std::vector<Obstacle> obstacles;
    detect_objects(cloud_filtered, detected_objects, obstacles);
    
    // 显示检测结果
    std::cout << "\nDetected " << detected_objects.size() << " objects:" << std::endl;
    for (size_t i = 0; i < detected_objects.size(); i++) {
        const auto& obj = detected_objects[i];
        std::cout << "Object #" << i+1 << ":" << std::endl;
        std::cout << "  Type: " << obj.shape_type << std::endl;
        std::cout << "  Rotation angle: " << obj.rotation_angle << " degrees" << std::endl;
        std::cout << "  Size: " << obj.size << std::endl;
        std::cout << "  Position: [" << obj.centroid[0] << ", " << obj.centroid[1] << ", " << obj.centroid[2] << "]" << std::endl;
        std::cout << "  Overlap score: " << obj.overlap_score << std::endl;
        std::cout << "  Color: [R:" << static_cast<int>(obj.r) << ", G:" << static_cast<int>(obj.g) << ", B:" << static_cast<int>(obj.b) << "]" << std::endl;
        std::cout << std::endl;
    }

    // 添加：打印障碍物信息
std::cout << "\nObstacles detected: " << obstacles.size() << std::endl;
for (size_t i = 0; i < obstacles.size(); i++) {
    const auto& obstacle = obstacles[i];
    std::cout << "Obstacle #" << i+1 << ":" << std::endl;
    
    // 打印形状类型
    std::cout << "  Shape type: ";
    switch(obstacle.obstacle.type) {
        case shape_msgs::SolidPrimitive::BOX:
            std::cout << "BOX" << std::endl;
            std::cout << "  Dimensions: [" 
                      << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_X] << ", "
                      << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_Y] << ", "
                      << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_Z] << "]" << std::endl;
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            std::cout << "SPHERE" << std::endl;
            std::cout << "  Radius: " << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] << std::endl;
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            std::cout << "CYLINDER" << std::endl;
            std::cout << "  Height: " << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] << std::endl;
            std::cout << "  Radius: " << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] << std::endl;
            break;
        case shape_msgs::SolidPrimitive::CONE:
            std::cout << "CONE" << std::endl;
            std::cout << "  Height: " << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] << std::endl;
            std::cout << "  Radius: " << obstacle.obstacle.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] << std::endl;
            break;
        default:
            std::cout << "UNKNOWN (" << obstacle.obstacle.type << ")" << std::endl;
    }
    
    // 打印位置和方向
    std::cout << "  Position: [" 
              << obstacle.obstacle_pose.position.x << ", "
              << obstacle.obstacle_pose.position.y << ", "
              << obstacle.obstacle_pose.position.z << "]" << std::endl;
    
    std::cout << "  Orientation: [" 
              << obstacle.obstacle_pose.orientation.x << ", "
              << obstacle.obstacle_pose.orientation.y << ", "
              << obstacle.obstacle_pose.orientation.z << ", "
              << obstacle.obstacle_pose.orientation.w << "]" << std::endl;
    
    std::cout << std::endl;
}
    
    return 0;
}

