#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <iomanip>
#include <iostream>
#include <boost/thread.hpp> 

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"

#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTOrtnVel.h"
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"
#include "wam_srvs/ForceTorqueTool.h"
#include "wam_srvs/ForceTorqueBase.h"
#include "wam_srvs/ForceTorque.h"

#define MODE_DISCONNECT 0
#define MODE_FREE 1
#define MODE_BASE_CONNECT 2
#define MODE_BASE_UPDATE 3
#define MODE_TOOL_CONNECT 4
#define MODE_FORCE_UPDATE 5
#define MODE_TORQUE_UPDATE 6
#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_X 2
#define BUTTON_Y 3
#define BUTTON_LB 4
#define BUTTON_RB 5
#define BUTTON_BACK 6
#define BUTTON_START 7
#define BUTTON_LEFT_STICK 9
#define BUTTON_RIGHT_STICK 10

using namespace std;

const int CNTRL_FREQ = 50; // Frequency at which we will publish our control messages.
const double MAX_FORCE = 6.0;
const double MAX_TORQUE = 0.2;

//WamTeleop Class
class WamTeleop {
  public:
    ros::NodeHandle n_, nh_wam, nh_bhand; // NodeHandles for publishing / subscribing on topics "/... & /wam/..." & "/bhand/..."
    // Boolean statuses for commanded states
    // bool cart_publish, ortn_publish, home_publish;
    // bool hold_publish, home_st, hold_st, ortn_mode;
    // bool open_grasp, close_grasp;
    // bool open_spread, close_spread;
    // bool open_grasp_state, close_grasp_state;
    // bool open_spread_state, close_spread_state;
    // bool stop;
    bool grasp;
    bool spread;
    bool lock_joints;
    bool new_command;
    // bool rt_control;
    // Integer status for BarrettHand commanded state
    // int bh_cmd_st;
    int mode;
    //variables to describe buttons on Joystick and their assignments
    // int deadman_btn, guardian_deadman_btn;
    // int gpr_open_btn, gpr_close_btn;
    // int sprd_open_btn, sprd_close_btn;
    // int ortn_btn, home_btn, hold_btn;
    // int axis_x, axis_y, axis_z;
    // int axis_r, axis_p, axis_yaw;
    int button_pressed;
    //variables to describe velocity commands
    // double max_grsp_vel, max_sprd_vel;
    // double cart_mag, ortn_mag;
    double req_xdir, req_ydir, req_zdir;
    double req_rdir, req_pdir, req_yawdir;
    //Subscribers
    ros::Subscriber joy_sub;
    //Services
    wam_srvs::BHandGraspVel grasp_vel;
    wam_srvs::BHandSpreadVel spread_vel;
    wam_srvs::Hold hold;
    std_srvs::Empty go_home;
    std_srvs::Empty empty;
    //Service Clients
    ros::ServiceClient bhand_init_srv;
    ros::ServiceClient grasp_open_srv;
    ros::ServiceClient grasp_close_srv;
    ros::ServiceClient grasp_vel_srv;
    ros::ServiceClient spread_open_srv;
    ros::ServiceClient spread_close_srv;
    ros::ServiceClient spread_vel_srv;
    ros::ServiceClient go_home_srv;
    ros::ServiceClient hold_srv;
    ros::ServiceClient force_torque_base_srv;
    ros::ServiceClient joy_force_torque_base_srv;
    ros::ServiceClient joy_force_torque_tool_srv;
    ros::ServiceClient disconnect_srv;
    //Messages
    // wam_msgs::RTCartVel cart_vel;
    // wam_msgs::RTOrtnVel ortn_vel;
    //Publishers
    // ros::Publisher cart_vel_pub, ortn_vel_pub;
    // Name our nodehandle "wam" to preceed our messages/services
    WamTeleop() : nh_wam("zeus/wam"), nh_bhand("zeus/bhand") {}

    void init();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void updateRT();
    void updateCommand();

    ~WamTeleop() {}
};

// WAM Teleoperation Initialization Function
void WamTeleop::init() {
    hold.request.hold = false; // Default Start for joint hold command is false
    mode = MODE_DISCONNECT;
    lock_joints = false;
    grasp = true;
    spread = false;
    new_command = false;

    joy_sub = n_.subscribe < sensor_msgs::Joy > ("joy", 1, &WamTeleop::joyCallback, this); // /joy

    bhand_init_srv = nh_bhand.serviceClient<std_srvs::Empty>("initialize");
    grasp_open_srv = nh_bhand.serviceClient<std_srvs::Empty>("open_grasp");
    grasp_close_srv = nh_bhand.serviceClient<std_srvs::Empty>("close_grasp");
    spread_open_srv = nh_bhand.serviceClient<std_srvs::Empty>("open_spread");
    spread_close_srv = nh_bhand.serviceClient<std_srvs::Empty>("close_spread");
    spread_vel_srv = nh_bhand.serviceClient<wam_srvs::BHandSpreadVel>("spread_vel");
    grasp_vel_srv = nh_bhand.serviceClient<wam_srvs::BHandGraspVel>("grasp_vel");
    go_home_srv = nh_wam.serviceClient<std_srvs::Empty>("go_home");                  // /wam/go_home
    hold_srv = nh_wam.serviceClient<wam_srvs::Hold>("hold_joint_pos");               // /wam/hold_joint_pos
    force_torque_base_srv = nh_wam.serviceClient<wam_srvs::ForceTorqueBase>("force_torque_base");
    joy_force_torque_base_srv = nh_wam.serviceClient<wam_srvs::ForceTorque>("joy_force_torque_base");
    joy_force_torque_tool_srv = nh_wam.serviceClient<wam_srvs::ForceTorque>("joy_force_torque_tool");
    disconnect_srv = nh_wam.serviceClient<std_srvs::Empty>("disconnect_systems");
    // cout << "MODE_FREE" << endl;
    // cart_vel_pub = nh_wam.advertise<wam_msgs::RTCartVel>("cart_vel_cmd", 1);         // /wam/cart_vel_cmd
    // ortn_vel_pub = nh_wam.advertise<wam_msgs::RTOrtnVel>("ortn_vel_cmd", 1);         // /wam/ortn_vel_cmd
}

void WamTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    //Set our publishing states back to false for new commands
    std::vector<int> input;
    input = joy_msg->buttons;
    for (int i = 0; i < joy_msg->buttons.size(); ++i) {
        if (!new_command && joy_msg->buttons[i]) { 
            button_pressed = i;
            new_command = true;
        }
    }
    if (-0.25 < joy_msg->axes[1] && joy_msg->axes[1] < 0.25) {
        req_xdir = 0.0;
    } else {
        req_xdir = joy_msg->axes[1];
    }

    if (-0.25 < joy_msg->axes[0] && joy_msg->axes[0] < 0.25) {
        req_ydir = 0.0;
    } else {
        req_ydir = joy_msg->axes[0];
    }

    if (-0.25 < joy_msg->axes[4] && joy_msg->axes[4] < 0.25) {
        req_zdir = 0.0;
    } else {
        req_zdir = joy_msg->axes[4];
    }
    // //RPY Velocity Portion
    // if ((joy_msg->axes[axis_r] > 0.25 || joy_msg->axes[axis_r] < -0.25) && ortn_mode) {
    //     req_rdir = -joy_msg->axes[axis_r];
    //     ortn_publish = true;
    // } else {
    //     req_rdir = 0.0;
    // }

    // if ((joy_msg->axes[axis_y] > 0.25 || joy_msg->axes[axis_y] < -0.25) && ortn_mode) {
    //     req_pdir = -joy_msg->axes[axis_p];
    //     ortn_publish = true;
    // } else {
    //     req_pdir = 0.0;
    // }
    // if ((joy_msg->axes[axis_z] > 0.25 || joy_msg->axes[axis_z] < -0.25) && ortn_mode) {
    //     req_yawdir = joy_msg->axes[axis_yaw];
    //     ortn_publish = true;
    // } else {
    //     req_yawdir = 0.0;
    // }

}

void WamTeleop::updateRT() {
    ros::Rate r(CNTRL_FREQ); // Setting the publishing rate to CNTRL_FREQ (50Hz by default)
    while (ros::ok()) {
        wam_srvs::ForceTorque ft;
        // cout << "Mode: " << mode << endl;
        switch (mode) {
            case MODE_DISCONNECT:
                disconnect_srv.call(empty);
                mode = MODE_FREE;
                cout << "\tFREE" << endl;
                break;
            case MODE_FREE:
                break;
            case MODE_BASE_CONNECT:
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.force.push_back(req_xdir);
                ft.request.force.push_back(req_ydir);
                ft.request.force.push_back(req_zdir);
                ft.request.kp.push_back(4.0);
                ft.request.kp.push_back(4.0);
                ft.request.kp.push_back(4.0);
                ft.request.kd.push_back(0.2);
                ft.request.kd.push_back(0.2);
                ft.request.kd.push_back(0.2);
                ft.request.initialize = true;
                joy_force_torque_base_srv.call(ft);
                mode = MODE_BASE_UPDATE;
                cout << "\tBASE FORCE UPDATE" << endl;
                break;
            case MODE_BASE_UPDATE:
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.force.push_back(req_xdir);
                ft.request.force.push_back(req_ydir);
                ft.request.force.push_back(req_zdir);
                ft.request.kp.push_back(150.0);                    
                ft.request.kp.push_back(150.0);                    
                ft.request.kp.push_back(150.0);  
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.initialize = false;                
                joy_force_torque_base_srv.call(ft);
                break;      
            case MODE_TOOL_CONNECT:
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.force.push_back(0.0);
                ft.request.force.push_back(0.0);
                ft.request.force.push_back(0.0);
                ft.request.kp.push_back(0.0);
                ft.request.kp.push_back(0.0);
                ft.request.kp.push_back(0.0);
                ft.request.kd.push_back(0.0);
                ft.request.kd.push_back(0.0);
                ft.request.kd.push_back(0.0);
                ft.request.initialize = true;
                joy_force_torque_tool_srv.call(ft);
                mode = MODE_FORCE_UPDATE;
                cout << "\tTOOL FORCE UPDATE" << endl;
                break;
            case MODE_FORCE_UPDATE:
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.torque.push_back(0.0);
                ft.request.force.push_back(req_xdir);
                ft.request.force.push_back(req_ydir);
                ft.request.force.push_back(req_zdir);
                ft.request.kp.push_back(50.0);
                ft.request.kp.push_back(50.0);
                ft.request.kp.push_back(50.0);
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.initialize = false;
                joy_force_torque_tool_srv.call(ft);
                break;
            case MODE_TORQUE_UPDATE:
                ft.request.torque.push_back(req_ydir);
                ft.request.torque.push_back(req_xdir);
                ft.request.torque.push_back(req_zdir);
                ft.request.force.push_back(0.0);
                ft.request.force.push_back(0.0);
                ft.request.force.push_back(0.0);
                ft.request.kp.push_back(50.0);
                ft.request.kp.push_back(50.0);
                ft.request.kp.push_back(50.0);
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.kd.push_back(5.0);
                ft.request.initialize = false;
                joy_force_torque_tool_srv.call(ft);              
                break;
            }
        r.sleep();
    }
}

// Function for updating the commands and publishing
void WamTeleop::updateCommand() {
    if (new_command) {
        switch (button_pressed) {
            case BUTTON_A:
                grasp = !grasp;
                if (grasp) {
                    cout << "\t\tcalling grasp open" << endl;
                    grasp_open_srv.call(empty);
                } else {
                    cout << "\t\tcalling grasp close" << endl;
                    grasp_close_srv.call(empty);
                }
                break;
            case BUTTON_B:
                spread = !spread;
                if (spread) {
                    cout << "\t\tcalling spread open" << endl;
                    spread_open_srv.call(empty);
                } else {
                    cout << "\t\tcalling spread close" << endl;
                    spread_close_srv.call(empty);
                }
                break;
            case BUTTON_X:
                break; 
            case BUTTON_Y:
                break;
            case BUTTON_LB:
                mode = (mode + 1) % 7;
                switch (mode) {
                    case MODE_DISCONNECT:
                        // cout << "MODE_DISCONNECT" << endl;
                        break;
                    case MODE_FREE:
                        cout << "\tFREE" << endl;
                        break;
                    case MODE_BASE_CONNECT:
                        // cout << "MODE_BASE_CONNECT" << endl;
                        break;
                    case MODE_BASE_UPDATE:
                        cout << "\tBASE FORCE UPDATE" << endl;
                        break;      
                    case MODE_TOOL_CONNECT:
                        // cout << "MODE_TOOL_CONNECT" << endl;
                        break;
                    case MODE_FORCE_UPDATE:
                        cout << "\tTOOL FORCE UPDATE" << endl;
                        break;
                    case MODE_TORQUE_UPDATE:
                        cout << "\tTOOL TORQUE UPDATE" << endl;          
                        break;
                }
                break;
            case BUTTON_RB:
                lock_joints = !lock_joints;
                cout << "\t\tjoints locked: " << boolalpha << lock_joints << endl;
                hold.request.hold = lock_joints;
                hold_srv.call(hold); 
                break;            
            case BUTTON_BACK:
                disconnect_srv.call(empty);
                mode = MODE_FREE;
                cout << "\t\tcalling go home" << endl;
                go_home_srv.call(empty); 
                lock_joints = true;
                break;
            case BUTTON_START:
                cout << "\t\tinitializing hand and setting velocities to 16" << endl;
                bhand_init_srv.call(empty);
                spread_vel.request.velocity = 16;
                spread_vel_srv.call(spread_vel);
                grasp_vel.request.velocity = 16;
                grasp_vel_srv.call(grasp_vel);
                spread_close_srv.call(empty);
                break;
            case BUTTON_LEFT_STICK:
                break;
            case BUTTON_RIGHT_STICK:
                break;
        }
        new_command = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wam_teleop"); // Start our wam_node and name it "wam_teleop"
    // don't remove spinner lines or arm won't subscribe to wam topics
    ros::AsyncSpinner spinner(0);
    spinner.start();

    WamTeleop wam_teleop; // Declare a member of WamTeleop "wam_teleop"
    wam_teleop.init(); // Initialize our teleoperation
    boost::thread feedback_thread(&WamTeleop::updateRT, &wam_teleop);

    ros::Rate pub_rate(CNTRL_FREQ); // Setting the publishing rate to CNTRL_FREQ (50Hz by default)
    // Looping at the specified frequency while our ros node is ok
    cout << "******************************************************************" << endl;
    cout << "* WAM Teleoperation Node with the Logitech Controller            *" << endl;
    cout << "******************************************************************" << endl;
    cout << "* LB BUTTON:     Mode Switch                                     *" << endl;
    cout << "* RB BUTTON:     Toggle Hold Joints                              *" << endl;
    cout << "* START BUTTON:  Initialize Hand                                 *" << endl;
    cout << "* BACK BUTTON:   Go Home                                         *" << endl;
    cout << "* A BUTTON:      Toggle Grasp                                    *" << endl;
    cout << "* B BUTTON:      Toggle Spread                                   *" << endl;
    cout << "* LEFT STICK:    X & Y Axis Motion                               *" << endl;
    cout << "* RIGHT STICK:   Z Axis Motion                                   *" << endl;
    cout << "******************************************************************" << endl;
    cout << "Current Mode:" << endl;
    while (wam_teleop.n_.ok()) {
        ros::spinOnce();
        wam_teleop.updateCommand();
        pub_rate.sleep();
    }
    return 0;
}
