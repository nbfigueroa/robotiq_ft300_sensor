#include "ros/ros.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"
#include <sstream>
#include "geometry_msgs/WrenchStamped.h"
#include "eigen3/Eigen/Core"

double force_dead_zone_   = 0.2;
double torque_dead_zone_  = 0.2;
double force_filter_rate_ = 0.25;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

Vector6d wrench_input_;
Vector6d wrench_filter_;

geometry_msgs::WrenchStamped wrenchMsg;

void UpdateInputWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	wrench_input_ << msg->wrench.force.x, msg->wrench.force.y,
	          msg->wrench.force.z, msg->wrench.torque.x,
	          msg->wrench.torque.y, msg->wrench.torque.z;


	ROS_INFO_STREAM_THROTTLE(0.1, "Input Wrench           : " <<
		                         wrench_input_(0) << " " <<
		                         wrench_input_(1) << " " <<
		                         wrench_input_(2) << " " <<
		                         wrench_input_(3) << " " <<
		                         wrench_input_(4) << " " <<
		                         wrench_input_(5) );

	// Dead zone for the FT sensor
	if (wrench_input_.topRows(3).norm() < force_dead_zone_) {
		wrench_input_.topRows(3).setZero();
	}
	if (wrench_input_.bottomRows(3).norm() < torque_dead_zone_) {
		wrench_input_.bottomRows(3).setZero();
	}

	// Filter and update
	// wrench_filter_ = wrench_input_;
	wrench_filter_ += (1 - force_filter_rate_) * (wrench_input_ - wrench_filter_);

	ROS_INFO_STREAM_THROTTLE(0.1, "Filtered Wrench          : " <<
		                         wrench_filter_(0) << " " <<
		                         wrench_filter_(1) << " " <<
		                         wrench_filter_(2) << " " <<
		                         wrench_filter_(3) << " " <<
		                         wrench_filter_(4) << " " <<
		                         wrench_filter_(5) );

	
	wrenchMsg.header.stamp = ros::Time::now();
	wrenchMsg.header.frame_id = "robotiq_force_torque_frame_id";
	wrenchMsg.wrench.force.x =  wrench_filter_(0);
	wrenchMsg.wrench.force.y =  wrench_filter_(1);
	wrenchMsg.wrench.force.z =  wrench_filter_(2);
	wrenchMsg.wrench.torque.x =  wrench_filter_(3);
	wrenchMsg.wrench.torque.y =  wrench_filter_(4);
	wrenchMsg.wrench.torque.z =  wrench_filter_(5);		
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "rq_stream_sensor");
	ros::NodeHandle n;

	/* Service to bias sensor */
	ros::ServiceClient client          = n.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc");	
	robotiq_force_torque_sensor::sensor_accessor srv;

	/* Subscriber and publisher to filter signal */
	ros::Subscriber  sub_input_wrench           = n.subscribe("robotiq_force_torque_wrench", 1000, UpdateInputWrench, ros::TransportHints().reliable().tcpNoDelay());
	ros::Publisher   pub_input_wrench_filtered  = n.advertise<geometry_msgs::WrenchStamped>("robotiq_force_torque_wrench_filtered", 1);

	/* Initializing */
	wrench_input_.setZero();
	wrench_filter_.setZero();
	
	/* Bias sensor readings */
	srv.request.command_id = srv.request.COMMAND_SET_ZERO;

	if(client.call(srv)){
		ROS_INFO("ret: %s", srv.response.res.c_str());
	}
	
	ros::Rate loop_rate(150);

	/* Filter and stream */
	while (ros::ok()){
		pub_input_wrench_filtered.publish(wrenchMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
