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
    std::cout << "\n检测到 " << detected_objects.size() << " 个物体:" << std::endl;
    for (size_t i = 0; i < detected_objects.size(); i++) {
        const auto& obj = detected_objects[i];
        // std::cout << "物体 #" << i+1 << ":" << std::endl;
        // std::cout << "  类型: " << obj.type << std::endl;
        // std::cout << "  位置: [" << obj.position.x << ", " << obj.position.y << ", " << obj.position.z << "]" << std::endl;
        // std::cout << "  尺寸: [" << obj.w << ", " << obj.l << ", " << obj.h << "]" << std::endl;
        // std::cout << "  颜色: [R:" << (int)obj.r << ", G:" << (int)obj.g << ", B:" << (int)obj.b << "]" << std::endl;
        std::cout << std::endl;
    }
    
    return 0;
}

