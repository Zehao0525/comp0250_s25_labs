task3 1和2中很多都是3的子功能

Task3的完整工作流程如下：

机械臂初始化，移动到安全位置
执行全域扫描，获取完整点云
点云下采样和预处理
执行点云聚类，分离不同物体
对每个聚类进行形状识别和特征提取
识别场景中的障碍物，添加到规划场景
统计不同形状物体数量，决定抓取目标
根据物体形状和旋转角度优化抓取姿态
规划并执行抓取轨迹，避开障碍物



调试信息 
每次全局扫描后的点云经过合并后点云保存在./task3_scan_result.pcd

为了方便调试 我把物品检测部分提取出来为一个动态链接库 同时有一个测试入口PCLtest 源文件同名 用于检查检测效果

      // 优化轨迹速度
      robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
      rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);

      // 应用速度和加速度缩放因子进行时间参数化
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      double velocity_scaling_factor = 1.5;  // 速度缩放因子（提高速度）
      double acceleration_scaling_factor = 1;  // 加速度缩放因子
      iptp.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);

      rt.getRobotTrajectoryMsg(my_plan.trajectory_); 

      这部分移动函数改变了机械臂速度 为了加快调试用的 最后提交记得删除


Theres a chance that trajectroy planning fails. It does not usually happen, but if it does it might affect pick and place