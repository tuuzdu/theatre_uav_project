#include "trajectory_planner2.hpp"

bool TrajectoryPlanner::init() {

	// Get Gait Settings From Parameter Server
	//node.param("trapezoid_low_radius", trap_low_r, 0.03);

	node.param("/theatre_uav_trajectory_planner/rate", rate, 10.0);
	node.param("/theatre_uav_trajectory_planner/duration", duration, 30.0);
	node.param("/theatre_uav_trajectory_planner/x_square", x, 0.5);
	node.param("/theatre_uav_trajectory_planner/y_square", y, 0.5);
	node.param("/theatre_uav_trajectory_planner/altittude", altittude, 0.65);
	node.param("/theatre_uav_trajectory_planner/altittude_square", altittude_square, 0.65);
	node.param("/theatre_uav_trajectory_planner/time_of_state", time_of_state, 5.0);
	node.param("/theatre_uav_trajectory_planner/disarming_z", disarming_z, 2.55);

	a.p = KDL::Vector (x, y, altittude_square);
	b.p = KDL::Vector (-x, y, altittude_square);
	c.p = KDL::Vector (-x, -y, altittude_square);
	d.p = KDL::Vector (x, -y, altittude_square);

	path = new KDL::Path_RoundedComposite (0.01,0.005,&rot);
	path -> Add(a);
	path -> Add(b);
	path -> Add(c);
	path -> Add(d);
	path -> Add(a);
	path -> Finish();

	vel_prof.SetProfileDuration(0,path->PathLength(), duration);

	trajectory = new KDL::Trajectory_Segment (path, &vel_prof);

	visual_pub = node.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
	local_setpoint_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

	sub_local_pose = node.subscribe("/odometry/filtered", 10, &TrajectoryPlanner::poseCallback, this);

	mavros_arming_client = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	run_square = false;
	run_prog = false;
	passed_sec = 0;

//	arming_srv.request.value = true;
//	mavros_arming_client.call(arming_srv);

	return true;
}

void TrajectoryPlanner::poseCallback(const nav_msgs::OdometryConstPtr& msg){
	local_z = msg->pose.pose.position.z;
}

void TrajectoryPlanner::trajectorySquare() {
	KDL::Frame frame;
	bool trj_finish = false;

	passed_sec_square = ros::Time::now().toSec() - begin_sec_square;
	if (passed_sec_square >= duration - 1/rate){
		passed_sec_square = duration - 0.05;
		trj_finish = true;
	}
	frame = trajectory -> Pos(passed_sec_square);
	if (trj_finish == true){
		trj_finish = false;
		begin_sec_square = ros::Time::now().toSec();
		passed_sec_square = 0;
	}


	setpoint_msg.pose.position.x = frame.p.data[0];
	setpoint_msg.pose.position.y = frame.p.data[1];
	setpoint_msg.pose.position.z = frame.p.data[2];
	frame.M.GetQuaternion(setpoint_msg.pose.orientation.x, setpoint_msg.pose.orientation.y, setpoint_msg.pose.orientation.z, setpoint_msg.pose.orientation.w);
}

void TrajectoryPlanner::trajectoryLoop() {
	while (node.ok()){
		int c = getch();   // call your non-blocking input function
		switch (c){
			case 'a':
				arming_srv.request.value = true;
				if (mavros_arming_client.call(arming_srv))
					ROS_INFO("Arming!");
				else ROS_ERROR("Arming service not available");
				break;
			case 'd':
				arming_srv.request.value = false;
				if (mavros_arming_client.call(arming_srv))
					ROS_INFO("Disarming!");
				else ROS_ERROR("Disarming service not available");
				break;
			case 'l':
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = 0;
				run_square = false;
				run_prog = false;
				if (local_z > disarming_z){
					arming_srv.request.value = false;
					if (mavros_arming_client.call(arming_srv))
						ROS_INFO("Disarming!");
					else ROS_ERROR("Disarming service not available");
				}
				break;
			case '1':
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude;
				run_square = false;
				run_prog = false;
				break;
			case '2':
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude_square;
				run_square = false;
				run_prog = false;
				break;
			case '3':
				setpoint_msg.pose.position.x = x;
				setpoint_msg.pose.position.y = y;
				setpoint_msg.pose.position.z = altittude_square;
				run_square = false;
				run_prog = false;
				break;
			case '4':
				begin_sec_square = ros::Time::now().toSec();
				passed_sec_square = 0;
				id = 0;
				run_square = true;
				run_prog = false;
				break;
			case 'g':
				begin_sec = ros::Time::now().toSec();
				passed_sec = 0;
				id = 0;
				run_square = false;
				run_prog = true;
				break;
		}

		if (run_prog == true){
			passed_sec = ros::Time::now().toSec() - begin_sec;
			if (passed_sec < time_of_state*2){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude_square;
			}
			if (passed_sec >= time_of_state*2 && passed_sec < time_of_state*3 ){
				setpoint_msg.pose.position.x = x;
				setpoint_msg.pose.position.y = y;
				setpoint_msg.pose.position.z = altittude_square;
				begin_sec_square = ros::Time::now().toSec();
				passed_sec_square = 0;
			}

			if (passed_sec >= time_of_state*3 && passed_sec < time_of_state*12 ){
				trajectorySquare();
			}

			if (passed_sec >= time_of_state*12 && passed_sec < time_of_state*13 ){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude_square;
			}

			if (passed_sec >= time_of_state*13 && passed_sec < time_of_state*15 ){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude;
			}
			if (passed_sec >= time_of_state*15 && passed_sec < time_of_state*17 ){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude_square;
			}
			if (passed_sec >= time_of_state*17 && passed_sec < time_of_state*19 ){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude;
			}
			if (passed_sec >= time_of_state*21 && passed_sec < time_of_state*22 ){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = altittude_square;
			}

			if (passed_sec >= time_of_state*22){
				setpoint_msg.pose.position.x = 0;
				setpoint_msg.pose.position.y = 0;
				setpoint_msg.pose.position.z = 0;
				ROS_INFO("\nx: %f\n", disarming_z);
				if (local_z > disarming_z){
					arming_srv.request.value = false;
					if (mavros_arming_client.call(arming_srv))
						ROS_INFO("Disarming!");
					else ROS_ERROR("Disarming service not available");
				}
			}

		}

		if (run_square == true) trajectorySquare();

		//setpoint_msg.pose.orientation.w = 0.0;
		//setpoint_msg.pose.orientation.z = 1.0;
		//setpoint_msg.pose.orientation.x = 0.0;
		//setpoint_msg.pose.orientation.y = 0.0;

		trajectoryVisual();
		ROS_INFO("\nx: %f\ny: %f\nz: %f\ntime: %f", setpoint_msg.pose.position.x, setpoint_msg.pose.position.y, setpoint_msg.pose.position.z, passed_sec);
		local_setpoint_pub.publish (setpoint_msg);

		ros::spinOnce();
		ros::Rate(rate).sleep();
	}
}

void TrajectoryPlanner::trajectoryVisual(){
	marker_msg.pose.position.x = setpoint_msg.pose.position.x;
	marker_msg.pose.position.y = setpoint_msg.pose.position.y;
	marker_msg.pose.position.z = setpoint_msg.pose.position.z;
	marker_msg.pose.orientation.x = setpoint_msg.pose.orientation.x;
	marker_msg.pose.orientation.y = setpoint_msg.pose.orientation.y;
	marker_msg.pose.orientation.z = setpoint_msg.pose.orientation.z;
	marker_msg.pose.orientation.w = setpoint_msg.pose.orientation.w;
	marker_msg.header.frame_id = "odom_link";
	marker_msg.scale.x = 0.2;
	marker_msg.scale.y = 0.02;
	marker_msg.scale.z = 0.02;
	marker_msg.color.a = 0.4;
	marker_msg.color.g = 1;
	marker_msg.id = id++;
	if (id > 50) id = 0;

	visual_pub.publish (marker_msg);
}

int getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0);
		//ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_planner");
	TrajectoryPlanner tp;
	if (tp.init()<0) {
	    ROS_ERROR("Could not initialize trajectory_planner node");
	    return -1;
	}
	tp.trajectoryLoop();
    ros::spin();
    return 0;
}
