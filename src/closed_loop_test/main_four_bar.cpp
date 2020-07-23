#include <iostream>
#include "Dynamics.h"
#include <rbdl/rbdl.h>
#include <vector>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
int main()
{
    Dynamics dynamics_obj;
    std::vector<double> pos{.1, .1, .1};
    dynamics_obj.get_G(pos);
    return 0;
}
