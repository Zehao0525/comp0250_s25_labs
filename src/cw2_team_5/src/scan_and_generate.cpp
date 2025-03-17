#include "cw2_class.h"


void cw2::reset_arm(){
    Init_Pose target_pose;
    target_pose.position.x = -0.1;
    target_pose.position.y = -0.1;
    target_pose.position.z = 0.5;
    move_arm(target_pose);
}
