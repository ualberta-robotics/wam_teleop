#include <math.h>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/ForceTorqueTool.h"

const double MAX_FORCE=8;
const int CNTRL_FREQ = 10; // Frequency at which we will publish our control messages.

//WamTeleop Class
class WamTeleop {
	public:
		ros::NodeHandle n_, nw_, nh_; // NodeHandles for publishing / subscribing on topics "/... & /wam/..." & "/bhand/..."

		// Boolean statuses for commanded states
	bool cart_publish, home_publish;
	bool home_st;
	bool stop;

	
		//variables to describe buttons on Joystick and their assignments
	int deadman_btn, guardian_deadman_btn;
	int home_btn;
	int cartesian_force_mode_btn, tangential_force_mode_btn;
	int axis_x, axis_y, axis_z;
	
		//variables to describe velocity commands
	//     double cart_mag;
	double req_xdir, req_ydir, req_zdir;
	double torque_flag; //change force control mode
		//Subscribers
		ros::Subscriber joy_sub;

	//Services
	std_srvs::Empty go_home;

	//Service Clients

	ros::ServiceClient go_home_srv;
	ros::ServiceClient force_torque_base_srv;
	//Messages
	// wam_msgs::RTCartVel cart_vel;
	
		//Publishers
	// ros::Publisher cart_vel_pub;
		// Name our nodehandle "wam" to preceed our messages/services
		WamTeleop() : nw_("zeus/wam"), nh_("zeus/bhand") {}

		void init();
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
		void update();

		~WamTeleop() {}
};

// WAM Teleoperation Initialization Function
void WamTeleop::init() {


	//force_torque_base.request.force.resize(3);
	//force_torque_base.request.torque.resize(3);
		 
		// We will check the parameter server for WAM Teleoperation configuration variables
	n_.param("deadman_button", deadman_btn, 5);//10
		n_.param("guardian_deadman_button", guardian_deadman_btn, 11);
		//   n_.param("gripper_open_button", gpr_open_btn, 12);
		//  n_.param("gripper_close_button", gpr_close_btn, 14);
		// n_.param("spread_open_button", sprd_open_btn, 13);
		// n_.param("spread_close_button", sprd_close_btn, 15);
		// n_.param("orientation_control_button", ortn_btn, 8);
		n_.param("go_home_button", home_btn, 0);
		n_.param("cartesian_force_mode_button", cartesian_force_mode_btn, 8);
		n_.param("tangential_force_mode_button", tangential_force_mode_btn, 9);

		// n_.param("hold_joints_button", hold_btn, 3);
		//n_.param("grasp_max_velocity", max_grsp_vel, 1.0);
		//n_.param("spread_max_velocity", max_sprd_vel, 1.0);
		//n_.param("cartesian_magnitude", cart_mag, 0.15);
		//n_.param("orientation_magnitude", ortn_mag, 0.5);
		n_.param("cartesian_x_axis", axis_x, 3);
		n_.param("cartesian_y_axis", axis_y, 2);
		n_.param("cartesian_x_axis", axis_z, 1);
		//n_.param("orientation_roll_axis", axis_r, 3);
		//n_.param("orientation_pitch_axis", axis_p, 2);
		//n_.param("orientation_yaw_axis", axis_yaw, 1);

		//hold.request.hold = false; // Default Start for joint hold command is false
		cart_publish = false; // Setting publisher states to false
			stop = true;
			torque_flag=0.0;
		//Subscribers
		joy_sub = n_.subscribe < sensor_msgs::Joy > ("joy", 1, &WamTeleop::joyCallback, this); // /joy

		//Service Clients

			go_home_srv = nw_.serviceClient<std_srvs::Empty>("go_home");                  // /wam/go_home
	
		force_torque_base_srv=nw_.serviceClient<wam_srvs::ForceTorqueTool>("force_torque_base");


		//Publishers
		//   cart_vel_pub = nw_.advertise<wam_msgs::RTCartVel>("cart_vel_cmd", 1);         // /wam/cart_vel_cmd
	
}

void WamTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
		//Set our publishing states back to false for new commands
		stop = false;
		cart_publish = home_publish = false;
	
 if (joy_msg->buttons[cartesian_force_mode_btn]) 
			{
			
			torque_flag=0.0;



			}
		if(joy_msg->buttons[tangential_force_mode_btn]) 
			{

	
	torque_flag=8.0;
			}




		//Return with no deadman pressed or without first pose published yet.
		if (!joy_msg->buttons[deadman_btn] | joy_msg->buttons[guardian_deadman_btn])  {
				return;
		}

		// Go Home Command
		// Checking to see if go home button is pressed and if it was pressed last callback loop
		if (joy_msg->buttons[home_btn] && !home_st) {
				home_publish = true; // set true only when button is first pressed down
		}
		home_st = joy_msg->buttons[home_btn]; // setting the state for the next loop


		//change force mode

	 





		//Cartesian Velocity Portion
		if (joy_msg->axes[axis_x] > 0.25 || joy_msg->axes[axis_x] < -0.25) {
				req_xdir = joy_msg->axes[axis_x]*MAX_FORCE;
				cart_publish = true;
		} else {
				req_xdir = 0.0;
		}

		if (joy_msg->axes[axis_y] > 0.25 || joy_msg->axes[axis_y] < -0.25) {
				req_ydir = joy_msg->axes[axis_y]*MAX_FORCE;
				cart_publish = true;
		} else {
				req_ydir = 0.0;
		}

		if (joy_msg->axes[axis_z] > 0.25 || joy_msg->axes[axis_z] < -0.25) {
				req_zdir = joy_msg->axes[axis_z]*MAX_FORCE;
				cart_publish = true;
		} else {
				req_zdir = 0.0;
		}

	 
}

// Function for updating the commands and publishing
void WamTeleop::update() {


	wam_srvs::ForceTorqueTool force_torque_base;
	if (stop) {
			return;
	}


	//Check our publish go home state and act accordingly
	// if only home_publish state is set
	if(home_publish && !cart_publish) {
			ROS_INFO("GOING HOME");
			go_home_srv.call(go_home); // Command WAM to go home
	}
	//Check our published cartesian velocity state and act accordingly
	// if only cart_publish state is set
	if(cart_publish && !home_publish) {
		force_torque_base.request.force.push_back(req_xdir);
		force_torque_base.request.force.push_back(req_ydir);
		force_torque_base.request.force.push_back(req_zdir);
		force_torque_base.request.torque.push_back(0.0);
		force_torque_base.request.torque.push_back(0.0);
		force_torque_base.request.torque.push_back(torque_flag);
		force_torque_base_srv.call(force_torque_base);
		 ROS_INFO("Force:(%f,%f,%f) and Torque(%f,%f,%f)",req_xdir,req_ydir,req_zdir,0.0,0.0,torque_flag);
	}
	else{
force_torque_base.request.force.push_back(0.0);
		force_torque_base.request.force.push_back(0.0);
		force_torque_base.request.force.push_back(0.0);
		force_torque_base.request.torque.push_back(0.0);
		force_torque_base.request.torque.push_back(0.0);
		force_torque_base.request.torque.push_back(torque_flag);
		force_torque_base_srv.call(force_torque_base);
	 ROS_INFO("Force:(%f,%f,%f) and Torque(%f,%f,%f)",req_xdir,req_ydir,req_zdir,0.0,0.0,torque_flag);
	}

}

int main(int argc, char** argv)
{
		ros::init(argc, argv, "wam_teleop_force"); // Start our wam_node and name it "wam_teleop"
		WamTeleop wam_teleop; // Declare a member of WamTeleop "wam_teleop"
		wam_teleop.init(); // Initialize our teleoperation

		ros::Rate pub_rate(CNTRL_FREQ); // Setting the publishing rate to CNTRL_FREQ (50Hz by default)

		// Looping at the specified frequency while our ros node is ok
		while (wam_teleop.n_.ok()) {
				ros::spinOnce();
				wam_teleop.update(); // Update and send commands to the WAM
				pub_rate.sleep();
		}
		return 0;
}
