#include "cw2_class.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <fstream>



void saveTrajectoryToCSV(const robot_trajectory::RobotTrajectory& trajectory, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    const auto& joint_names = trajectory.getFirstWayPoint().getVariableNames();
    file << "time_from_start";
    for (const auto& name : joint_names)
        file << "," << name;
    file << "\n";

    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
        const auto& state = trajectory.getWayPoint(i);
        double time = trajectory.getWayPointDurationFromStart(i);
        file << time;
        for (const auto& name : joint_names)
            file << "," << state.getVariablePosition(name);
        file << "\n";
    }

    file.close();
    std::cout << "轨迹已保存到文件: " << filename << std::endl;
}