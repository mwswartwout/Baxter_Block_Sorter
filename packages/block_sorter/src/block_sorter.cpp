#include <ros/ros.h>
#include <block_sorter/block_sorter.h>
#include <geometry_msgs/PoseStamped.h>
#include <block_finder/final_pcl_utils.h>
#include <motion_planning/motion_planning_lib.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_sorter");
    ros::NodeHandle nh;

    Eigen::Vector3f goalPoint;
    Eigen::Vector3f goalOrientation;
    Eigen::Vector3d goalColor;

    FinalPclUtils final_pcl_utils(&nh);
    final_pcl_utils.find_block(goalPoint, goalOrientation, goalColor);
    
    MotionPlanning motion_planning(&nh);
    motion_planning.plan_move_to_pre_pose();
    motion_planning.rt_arm_execute_planned_path();

    geometry_msgs::PoseStamped goalPose = final_pcl_utils.eigenToPose(goalPoint);
    motion_planning.rt_arm_plan_path_current_to_goal_pose(goalPose); 
}
