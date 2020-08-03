#include "ros/ros.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/ObjectCmd.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include <iostream>
#include "Dynamics.h"
#include <rbdl/rbdl.h>
#include <vector>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
// #include "four_bar_linkage_transmission.h"

class Ambf_sub_pub
{
public:
    //constructor
    Ambf_sub_pub(const ros::Publisher &pub) : pub_(pub), rbdl_obj_(), pos_d{0., 0., 0., 0., 0., 0., 0., 0., 0.}, cmd_msg(), header_()
    {
        cmd_msg.enable_position_controller = 0;
        cmd_msg.position_controller_mask = {0};
    }
    //methods
    void StateCallback(const ambf_msgs::ObjectState::ConstPtr &msg)
    {
        std::vector<float> pos = msg->joint_positions;
        // ROS_INFO("joint 1 angle is: %f ", pos.at(0));
        pos_d.at(0) = static_cast<double>(pos.at(0));
        pos_d.at(1) = static_cast<double>(pos.at(1));
        pos_d.at(2) = static_cast<double>(pos.at(2));
        pos_d.at(3) = static_cast<double>(pos.at(3));
        pos_d.at(4) = static_cast<double>(pos.at(4));
        pos_d.at(5) = static_cast<double>(pos.at(5));
        pos_d.at(6) = static_cast<double>(pos.at(6));
        pos_d.at(7) = static_cast<double>(pos.at(7));
        pos_d.at(8) = static_cast<double>(pos.at(8));

        std::vector<float> tau_cmd{0., 0., 0., 0., 0., 0., 0., 0., 0.};
        RigidBodyDynamics::Math::VectorNd tau_d;
        // tau_d = rbdl_obj_.get_G(pos_d);
        // tau_d = rbdl_obj_.get_G_ClosedLoop(pos_d);
        tau_d = rbdl_obj_.get_G_MTM(pos_d);

        tau_cmd.at(0) = static_cast<float>(tau_d(0));
        tau_cmd.at(1) = static_cast<float>(tau_d(1));
        tau_cmd.at(7) = static_cast<float>(tau_d(2));
        tau_cmd.at(8) = static_cast<float>(tau_d(3));
        tau_cmd.at(2) = static_cast<float>(tau_d(5));
        tau_cmd.at(3) = static_cast<float>(tau_d(6));
        tau_cmd.at(4) = static_cast<float>(tau_d(7));
        tau_cmd.at(5) = static_cast<float>(tau_d(8));
        tau_cmd.at(6) = static_cast<float>(tau_d(9));
        header_.stamp = ros::Time::now();
        cmd_msg.header = header_;
        cmd_msg.joint_cmds = tau_cmd;
        pub_.publish(cmd_msg);
    }

private:
    //attributes
    std::vector<double> pos_d;
    ros::Publisher pub_;
    Dynamics rbdl_obj_;
    ambf_msgs::ObjectCmd cmd_msg;
    std_msgs::Header header_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mtm_gravity");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ambf_msgs::ObjectCmd>("/ambf/env/mtm/TopPanel/Command", 1000);
    Ambf_sub_pub obj1(pub);
    ros::Subscriber sub = nh.subscribe("/ambf/env/mtm/TopPanel/State", 1, &Ambf_sub_pub::StateCallback, &obj1);

    ros::spin();

    return 0;
}