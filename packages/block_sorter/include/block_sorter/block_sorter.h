#ifndef BLOCK_SORTER_BLOCK_SORTER_H
#define BLOCK_SORTER_BLOCK_SORTER_H

#include <gripper_controller/gripper_controller.h>
#include <color_move/color_move.h>
#include <motion_planning/motion_planning_lib.h>
#include <block_finder/final_pcl_utils.h>


class BlockSorter
{
public:
    BlockSorter();
    void doBlockSort();
private:
    ros::NodeHandle nh;
    FinalPclUtils final_pcl_utils;
    MotionPlanning motion_planning;
    ColorMove color_move;
    Eigen::Vector3f goalPoint;
    Eigen::Vector3f goalOrientation;
    Eigen::Vector3d goalColor;
    ros::Publisher pubCloud;
    pcl::PointCloud<pcl::PointXYZRGB> display_cloud; 
    sensor_msgs::PointCloud2 pcl2_display_cloud;
    tf::StampedTransform tf_sensor_frame_to_torso_frame;
    tf::TransformListener tf_listener;
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Vector3d red;
    Eigen::Vector3d blue;
    Eigen::Vector3d green;
};

#endif
