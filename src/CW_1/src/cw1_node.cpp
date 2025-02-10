/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

ros::Publisher pub;

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // create an instance of the cw1 class
  cw1 cw_class(nh);

  // create the a Ros subscriber for the input point cloud
  ros::Subscriber sub_cloud_task_2 = 
                  nh.subscribe ("/r200/camera/depth_registered/points",
                  1,
                  &cw1::cloudCallBackOne,
                  &cw_class);

  ros::Subscriber sub_cloud_task_3 = 
                  nh.subscribe ("/r200/camera/depth_registered/points",
                  1,
                  &cw1::cloudCallBackTwo,
                  &cw_class);
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();

    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

