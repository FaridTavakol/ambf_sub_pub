// /*
//  * Copyright (c) 2020-2024 Farid Tavakkolmoghaddam <ftavakkolmoghadd@wpi.edu>
//  *
//  * Licensed under the zlib license. See LICENSE for more details.
//  */

#include <iostream>
#include "Dynamics.h"
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
Dynamics::Dynamics()
{
    Four_bar_linkage farid;

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
