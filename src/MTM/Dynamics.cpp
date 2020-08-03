// /*
//  * Copyright (c) 2020-2024 Farid Tavakkolmoghaddam <ftavakkolmoghadd@wpi.edu>
//  *
//  * Licensed under the zlib license. See LICENSE for more details.
//  */

#include <iostream>
#include "Dynamics.h"
#include "/usr/local/include/rbdl/rbdl.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
Dynamics::Dynamics()
{
    Four_bar_linkage farid;
    Four_bar_linkage_closed cl_obj_;

    std::cout << "Constructor called" << std::endl;
}

VectorNd Dynamics::get_G(std::vector<double> pos)
{
    farid.Q(0) = pos.at(0);
    farid.Q(1) = pos.at(1);
    farid.Q(2) = pos.at(2);
    InverseDynamics(farid.model, farid.Q, farid.QDot, farid.QDDot, farid.Tau);
    std::cout << farid.Tau << std::endl;
    return farid.Tau;
}
VectorNd Dynamics::get_G_ClosedLoop(std::vector<double> pos)
{
    cl_obj_.Q(0) = pos.at(0);
    cl_obj_.Q(1) = pos.at(1);
    cl_obj_.Q(2) = pos.at(2);
    cl_obj_.Q(3) = -pos.at(1); // Since this is the loop joint, no matter the joint position the Tau corresponding to this joint is always zero.
    InverseDynamics(cl_obj_.model, cl_obj_.Q, cl_obj_.QDot, cl_obj_.QDDot, cl_obj_.Tau);
    std::cout << cl_obj_.Tau << std::endl;
    return cl_obj_.Tau;
}
VectorNd Dynamics::get_G_MTM(std::vector<double> pos)
{
    std::cout << "something" << std::endl;
    mtm_obj_.Q(0) = pos.at(0); // toppanel outpitch
    mtm_obj_.Q(1) = pos.at(1); // outpitch armparallel
    mtm_obj_.Q(2) = pos.at(2); // armparallel armparallel1
    mtm_obj_.Q(3) = pos.at(3); // armParallel1 armfront
    mtm_obj_.Q(4) = 0;         //pos.at(0);  // armfront virtual
    mtm_obj_.Q(5) = pos.at(4);
    mtm_obj_.Q(6) = pos.at(5);
    mtm_obj_.Q(7) = pos.at(6);
    mtm_obj_.Q(8) = pos.at(7);
    mtm_obj_.Q(9) = pos.at(8);

    // InverseDynamicsConstraintsRelaxed(mtm_obj_.model, mtm_obj_.Q, mtm_obj_.QDot, mtm_obj_.QDDot, mtm_obj_.cs, mtm_obj_.QDDot, mtm_obj_.Tau);
    std::cout << isConstrainedSystemFullyActuated(mtm_obj_.model, mtm_obj_.Q, mtm_obj_.QDot, mtm_obj_.cs) << std::endl;
    std::vector<bool> actuatedDof(10, true);
    actuatedDof.at(4) = false;
    mtm_obj_.cs.SetActuationMap(mtm_obj_.model, actuatedDof);
    InverseDynamicsConstraints(mtm_obj_.model, mtm_obj_.Q, mtm_obj_.QDot, mtm_obj_.QDDot, mtm_obj_.cs, mtm_obj_.QDDot, mtm_obj_.Tau);
    // InverseDynamics(mtm_obj_.model, mtm_obj_.Q, mtm_obj_.QDot, mtm_obj_.QDDot, mtm_obj_.Tau);

    std::cout << mtm_obj_.Tau << std::endl;
    return mtm_obj_.Tau;
}