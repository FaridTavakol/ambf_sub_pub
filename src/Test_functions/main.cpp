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

class Ambf_sub_pub
{
public:
    //constructor
    Ambf_sub_pub(const ros::Publisher &pub) : pub_(pub), rbdl_obj_(), pos_d(rbdl_obj_.cl_obj_.model.dof_count), cmd_msg(), header_()
    {
        cmd_msg.enable_position_controller = 0;
        cmd_msg.position_controller_mask = {0};
    }
    //methods
    void StateCallback(const ambf_msgs::ObjectState::ConstPtr &msg)
    {
        std::vector<float> pos = msg->joint_positions;
        // ROS_INFO("joint 1 angle is: %f ", pos.at(0));
        pos_d(0) = static_cast<double>(pos.at(0));
        pos_d(1) = static_cast<double>(pos.at(1));
        pos_d(2) = static_cast<double>(pos.at(2));
        std::vector<float> tau_cmd{0., 0., 0.};
        VectorNd tau_d(rbdl_obj_.cl_obj_.model.dof_count);
        // MatrixNd qdot;
        // qdot = rbdl_obj_.calc_M(pos_d);
        // std::cout << qdot << std::endl;
        tau_d = rbdl_obj_.calc_G(pos_d);
        std::cout << tau_d << std::endl;
        tau_cmd.at(0) = static_cast<float>(tau_d(0));
        tau_cmd.at(1) = static_cast<float>(tau_d(1));
        tau_cmd.at(2) = static_cast<float>(tau_d(2));
        header_.stamp = ros::Time::now();
        cmd_msg.header = header_;
        cmd_msg.joint_cmds = tau_cmd;
        pub_.publish(cmd_msg);
    }

private:
    //attributes
    VectorNd pos_d;
    ros::Publisher pub_;
    Dynamics rbdl_obj_;
    ambf_msgs::ObjectCmd cmd_msg;
    std_msgs::Header header_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FourBar_GravityComp");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<ambf_msgs::ObjectCmd>("/ambf/env/l1/Command", 1000);
    Ambf_sub_pub obj1(pub);
    ros::Subscriber sub = nh.subscribe("/ambf/env/l1/State", 1, &Ambf_sub_pub::StateCallback, &obj1);
    ros::spin();

    return 0;
}