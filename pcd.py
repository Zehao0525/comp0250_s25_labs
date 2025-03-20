import open3d as o3d
import argparse
import numpy as np

def visualize_pcd_with_axes(pcd_file):
    """
    读取并可视化 PCD 点云文件，显示坐标系和尺寸信息
    :param pcd_file: .pcd 文件路径
    """
    # 读取 PCD 文件
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 打印点云信息
    print(pcd)

    # 检查点云是否成功加载
    if not pcd.has_points():
        print(f"Error: 读取点云失败，请检查文件路径是否正确: {pcd_file}")
        return
    
    # 计算点云的边界框（Bounding Box）
    bbox = pcd.get_axis_aligned_bounding_box()
    bbox.color = (1, 0, 0)  # 红色的边界框

    # 计算点云的尺寸
    min_bound = bbox.get_min_bound()  # 最小坐标 (x_min, y_min, z_min)
    max_bound = bbox.get_max_bound()  # 最大坐标 (x_max, y_max, z_max)
    size = max_bound - min_bound  # 计算尺寸
    center = bbox.get_center()  # 计算中心点坐标

    print(f"点云边界框尺寸: {size} (单位: m)")
    print(f"最小坐标: {min_bound}")
    print(f"最大坐标: {max_bound}")
    print(f"中心点坐标: {center}")

    # 创建坐标系，长度为 0.1m
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # 创建可视化窗口
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="PCD Viewer with Axes")
    vis.add_geometry(pcd)
    vis.add_geometry(bbox)
    vis.add_geometry(coord_frame)
    vis.run()
    vis.destroy_window()

    # 获取用户选择的点
    picked_points = vis.get_picked_points()
    if picked_points:
        for idx in picked_points:
            point = pcd.points[idx]
            print(f"选中的点索引: {idx}, 坐标: {point}")

if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description="PCD 可视化工具（带坐标系和尺寸信息）")
    # parser.add_argument("pcd_file", type=str, help="输入的 PCD 文件路径")
    # args = parser.parse_args()
    # args.pcd_file = "task3_scan_result1.pcd"

    # visualize_pcd_with_axes(args.pcd_file)

    pcd_file = "ought_shape.pcd"
    pcd_file = "2.pcd"

    visualize_pcd_with_axes(pcd_file)
