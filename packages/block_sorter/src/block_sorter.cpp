#include <ros/ros.h>
#include <block_sorter/block_sorter.h>
#include <geometry_msgs/PoseStamped.h>
#include <block_finder/final_pcl_utils.h>
#include <motion_planning/motion_planning_lib.h>
#include <sensor_msgs/PointCloud2.h>
#include <gripper_controller/gripper_controller.h>
#include <color_move/color_move.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_sorter");
    ros::NodeHandle nh;

    Eigen::Vector3f goalPoint;
    Eigen::Vector3f goalOrientation;
    Eigen::Vector3d goalColor;

    FinalPclUtils final_pcl_utils(&nh);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_cloud_display", 1);
    pcl::PointCloud<pcl::PointXYZRGB> display_cloud; 
    sensor_msgs::PointCloud2 pcl2_display_cloud;

    if (!final_pcl_utils.got_kinect_cloud())
    {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    tf::StampedTransform tf_sensor_frame_to_torso_frame;
    tf::TransformListener tf_listener;

    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr)
    {
        tferr = false;
        try
        {
                tf_listener.lookupTransform("torso","camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
                //tf_listener.lookupTransform("torso","kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = final_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    ROS_INFO_STREAM("The found transform is\n" << A_sensor_wrt_torso.linear() << "\n" << A_sensor_wrt_torso.translation());

    final_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);

    ROS_INFO("Asking for block location");
    final_pcl_utils.find_block(goalPoint, goalOrientation, goalColor);
    ROS_INFO("Getting color cloud");
    final_pcl_utils.get_gen_purpose_clr_cloud(display_cloud);
    pcl::toROSMsg(display_cloud, pcl2_display_cloud);
    pcl2_display_cloud.header.stamp = ros::Time::now();
    pcl2_display_cloud.header.frame_id = "torso";

    MotionPlanning motion_planning(&nh);
    ROS_INFO("Moving to prepose");
    motion_planning.plan_move_to_pre_pose();
    motion_planning.rt_arm_execute_planned_path();

    ROS_INFO("Converting goal pose to geometry_msgs");
    ROS_INFO_STREAM(goalPoint(0) << "," << goalPoint(1) << "," << goalPoint(2));
    geometry_msgs::PoseStamped goalPose = final_pcl_utils.eigenToPose(goalPoint);
    int rtn_val = motion_planning.rt_arm_request_tool_pose_wrt_torso();
    geometry_msgs::PoseStamped rt_tool_pose = motion_planning.get_rt_tool_pose_stamped();
    rt_tool_pose.pose.position.x = goalPose.pose.position.x;
    rt_tool_pose.pose.position.y = goalPose.pose.position.y;
    rt_tool_pose.pose.position.z = goalPose.pose.position.z + .2;

    ROS_INFO("Going to goal pose");
    motion_planning.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose); 
    motion_planning.rt_arm_execute_planned_path();

    //GripperController gripper_controller(&nh);
    //gripper_controller.open();

    Eigen::Vector3d red;
        red << 191, 55, 96; 
    Eigen::Vector3d blue;
        blue << 140, 188, 226; 
    Eigen::Vector3d green;
        green << 150, 175, 70;

    ColorMove color_move(&nh);

    if ((goalColor - red).norm() < 50)
    {
        ROS_INFO("Moving to red block place");
        color_move.set_goal_color1();
    }
    else if ((goalColor - blue).norm() < 55)
    {
        ROS_INFO("Moving to blue block place");
        color_move.set_goal_color2();
    }
    else if ((goalColor - green).norm() < 75)
    {
        ROS_INFO("Moving to green block place");
        color_move.set_goal_color3();
    }
    else
    {
	ROS_WARN("Detected color is not red, green, or blue");
    }

    while (ros::ok())
    {
        pubCloud.publish(pcl2_display_cloud);
        ros::spinOnce();
    }
}
