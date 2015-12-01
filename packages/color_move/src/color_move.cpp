#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_action/trajAction.h>
#include <color_move/color_move.h>
#include <vector>


// constructor
ColorMove::ColorMove(ros::NodeHandle *nh)
{
    nh_ = *nh;
}

void ColorMove::set_goal_color1()
{
    Vectorq7x1 q_drop_pose_1;
    q_drop_pose_1 << 0, 0, 0, 0, 0, 0, 0;
    find_and_send_trajectory(q_drop_pose_1);
    ROS_INFO("Block *** ready to drop.");
}

void ColorMove::set_goal_color2()
{
    Vectorq7x1 q_drop_pose_2;
    q_drop_pose_2 << 0, 0, 0, 0, 0, 0, 0;
    find_and_send_trajectory(q_drop_pose_2);
    ROS_INFO("Block *** ready to drop.");
}

void ColorMove::set_goal_color3()
{
    Vectorq7x1 q_drop_pose_3;
    q_drop_pose_3 << 0, 0, 0, 0, 0, 0, 0;
    find_and_send_trajectory(q_drop_pose_3);
    ROS_INFO("Block *** ready to drop.");
}

void ColorMove::set_goal_color4()
{
    Vectorq7x1 q_drop_pose_4;
    q_drop_pose_4 << 0, 0, 0, 0, 0, 0, 0;
    find_and_send_trajectory(q_drop_pose_4);
    ROS_INFO("Block *** ready to drop.");
}

void ColorMove::find_and_send_trajectory(Vectorq7x1 position)
{
    Vectorq7x1 q_pose;
    q_pose << position;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory;
    Baxter_traj_streamer baxter_traj_streamer(&nh_);


    ROS_INFO("warming up callbacks...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // find the current pose of the robot
    ROS_INFO("getting current right arm pose");
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();
    ROS_INFO_STREAM("r_arm state: " << q_vec_right_arm.transpose());

    // push back the current pose of the robot, followed by the desired pose for the robot
    q_in_vecxd = q_vec_right_arm;
    des_path.push_back(q_in_vecxd);
    q_in_vecxd = q_pose;
    des_path.push_back(q_in_vecxd);

    // convert this into a trajectory
    ROS_INFO("stuffing trajectory");
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory);

    // copy this trajectory into a goal message
    cwru_action::trajGoal goal;
    goal.trajectory = des_trajectory;

    // initialize this node as an action client
    actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));
    if (!server_exists)
    {
        ROS_WARN("could not connect to server");
        return;
    }
    // server_exists = action_client.waitForServer(); // wait forever
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    // send the goal to the server
    action_client.sendGoal(goal);

    bool finished_before_timeout = action_client.waitForResult();  // wait forever for result

    ros::Duration(2.0).sleep();
}
