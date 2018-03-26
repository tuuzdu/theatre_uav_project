

#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_


#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
//#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <termios.h>
#include <stdio.h>




int getch ();

class TrajectoryPlanner {
	public:
		TrajectoryPlanner();
		bool init();
		void trajectoryLoop();
		void trajectorySquare();
		void trajectoryVisual();

	private:
		ros::NodeHandle node;
		double rate, duration;
		double altittude, x, y;

		visualization_msgs::Marker marker_msg;
		geometry_msgs::PoseStamped setpoint_msg;
		mavros_msgs::CommandBool arming_srv;

		// times
		double passed_sec, begin_sec;
		bool run_square;
		int id;

		ros::Publisher visual_pub;
		ros::Publisher local_setpoint_pub;
		ros::ServiceClient mavros_arming_client;

		KDL::Frame a, b, c, d;

		KDL::RotationalInterpolation_SingleAxis rot;
		KDL::Path_RoundedComposite *path;
		KDL::VelocityProfile_Spline vel_prof;
		KDL::Trajectory_Segment *trajectory;

};

TrajectoryPlanner::TrajectoryPlanner(){}



#endif /* TRAJECTORY_PLANNER_HPP_ */
