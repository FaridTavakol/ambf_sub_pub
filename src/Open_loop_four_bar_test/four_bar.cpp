/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main(int argc, char *argv[])
{
    rbdl_check_api_version(RBDL_API_VERSION);

    Model *model = NULL;

    unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
    Body body_a, body_b, body_c, body_d;
    Joint joint_a, joint_b, joint_c, joint_d;

    // Initializing Body properties
    double mass{1.};
    Vector3d com(0., -0.344, 0.);
    Vector3d inertia(0.058907, 0.003295, 0.058907);

    model = new Model();

    model->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

    body_a = Body(mass, com, inertia); /* mass, com, inertia*/
    joint_a = Joint(JointTypeFixed);

    Matrix3_t body_a_rot;
    body_a_rot << 0., 0., 1.,
        0., -1., 0.,
        1., 0., 0.;
    Vector3d body_a_trans(0.001, -0.36, -0.222);
    SpatialTransform body_a_tf(body_a_rot, body_a_trans);
    body_a_id = model->AddBody(0, body_a_tf, joint_a, body_a);

    body_b = Body(mass, com, inertia);
    joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

    Matrix3_t body_b_rot;
    body_b_rot << 0., -1., 0.,
        1., 0., 0.,
        0., 0., 1.;
    Vector3d body_b_trans(0.139, 0.138, 0.);
    SpatialTransform body_b_tf(body_b_rot, body_b_trans);

    body_b_id = model->AddBody(body_a_id, body_b_tf, joint_b, body_b);

    body_c = Body(mass, com, inertia);
    joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

    Matrix3_t body_c_rot;
    body_c_rot << 0., 1., 0.,
        -1., 0., 0.,
        0., 0., 1.;
    Vector3d body_c_trans(-0.141, -0.832, 0.);
    SpatialTransform body_c_tf(body_c_rot, body_c_trans);

    body_c_id = model->AddBody(body_b_id, body_c_tf, joint_c, body_c);

    body_d = Body(mass, com, inertia);
    joint_d = Joint(
        JointTypeRevolute,
        Vector3d(0., 0., 1.));

    Matrix3_t body_d_rot;
    body_d_rot << 0., 1., 0.,
        -1., 0., 0.,
        0., 0., 1.;
    Vector3d body_d_trans(-0.14, -0.83, 0.);
    SpatialTransform body_d_tf(body_d_rot, body_d_trans);

    body_d_id = model->AddBody(body_c_id, body_d_tf, joint_d, body_d);

    VectorNd Q = VectorNd::Zero(model->dof_count);
    // Q(0) = 0.2;
    // Q(1) = 0.2;
    // Q(2) = 0.2;
    VectorNd QDot = VectorNd::Zero(model->dof_count);
    VectorNd Tau = VectorNd::Zero(model->dof_count);
    VectorNd QDDot = VectorNd::Zero(model->dof_count);

    // ForwardDynamics(*model, Q, QDot, Tau, QDDot);

    // std::cout << QDDot.transpose() << std::endl;

    InverseDynamics(*model, Q, QDot, QDDot, Tau);
    std::cout << Tau << std::endl;

    delete model;

    return 0;
}
