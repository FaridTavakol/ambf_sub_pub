#include "ros/ros.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/ObjectCmd.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include "Dynamics.h"
#include <rbdl/rbdl.h>
#include <vector>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
// #include "four_bar_linkage_transmission.h"

void StateCallback(const ambf_msgs::ObjectState::ConstPtr &msg)
{
    Dynamics dynamics_obj;
    std::vector<double>
        pos_d{0., 0., 0.};
    std::vector<float> pos = msg->joint_positions;
    ROS_INFO("joint 1 angle is: %f ", pos.at(0));
    pos_d.at(0) = static_cast<double>(pos.at(0));
    pos_d.at(1) = static_cast<double>(pos.at(1));
    pos_d.at(2) = static_cast<double>(pos.at(2));
    dynamics_obj.get_G(pos_d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;
    std::cout << "I've been read" << std::endl;
    ros::Subscriber sub = nh.subscribe("/ambf/env/l1/State", 1, StateCallback);

    ros::spin();

    return 0;
}