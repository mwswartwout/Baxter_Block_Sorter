#ifndef COLOR_MOVE_H
#define COLOR_MOVE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

class ColorMove {
public:
	ros::NodeHandle nh_;
	int g_count;

	ColorMove(ros::NodeHandle *nh);

	void set_goal_color1();
	void set_goal_color2();
	void set_goal_color3();
	void set_goal_color4();
    void set_goal_safe_place();

private:
	void find_and_send_trajectory(Vectorq7x1 position);
};

#endif
