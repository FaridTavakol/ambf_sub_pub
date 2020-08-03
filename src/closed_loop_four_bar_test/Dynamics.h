// /*
//  * RBDL - Rigid Body Dynamics Library
//  * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
//  *
//  * Licensed under the zlib license. See LICENSE for more details.
//  */
#pragma once

#include <rbdl/rbdl.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
struct Four_bar_linkage
{ /*constructor for the struct*/
    Four_bar_linkage() : model(), Q(), QDot(), QDDot(), mass(1), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id()
    {
        using namespace RigidBodyDynamics;
        using namespace RigidBodyDynamics::Math;
        model.gravity = Vector3d(0., 0., -9.81);

        Vector3d com(0., -0.344, 0.);
        Vector3d inertia(0.058907, 0.003295, 0.058907);
        Body body_a, body_b, body_c, body_d;
        Joint joint_a, joint_b, joint_c, joint_d;
        // body a
        joint_a = Joint(JointTypeFixed);
        body_a = Body(mass, com, inertia); /* mass, com, inertia*/
        Matrix3_t body_a_rot;
        body_a_rot << 0., 0., 1.,
            0., -1., 0.,
            1., 0., 0.;
        Vector3d body_a_trans(0.001, -0.36, -0.222);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_a, body_a);
        // body b
        body_b = Body(mass, com, inertia);
        joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_b_rot;
        body_b_rot << 0., 1., 0.,
            -1., 0., 0.,
            0., 0., 1.;
        Vector3d body_b_trans(0.139, 0.138, 0.);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_b, body_b);
        // body c
        body_c = Body(mass, com, inertia);
        joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_c_rot;
        body_c_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_c_trans(-0.141, -0.832, 0.);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_c, body_c);
        // body d
        body_d = Body(mass, com, inertia);
        joint_d = Joint(
            JointTypeRevolute,
            Vector3d(0., 0., 1.));

        Matrix3_t body_d_rot;
        body_d_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_d_trans(-0.14, -0.83, 0.);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_c_id, body_d_tf, joint_d, body_d);
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
    }

    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Math::VectorNd Q;
    RigidBodyDynamics::Math::VectorNd QDot;
    RigidBodyDynamics::Math::VectorNd QDDot;
    RigidBodyDynamics::Math::VectorNd Tau;
    double mass;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
};
struct Four_bar_linkage_closed
{ /*constructor for the struct*/
    Four_bar_linkage_closed() : model(), Q(), QDot(), QDDot(), mass(1), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_virtual_id(), X_s(Xtrans(Vector3d(0., 0., 0.))), X_p(), cs(), virtual_mass(0)
    {
        using namespace RigidBodyDynamics;
        using namespace RigidBodyDynamics::Math;
        model.gravity = Vector3d(0., 0., -9.81);

        Vector3d com(0., -0.344, 0.);
        Vector3d inertia(0.0589073140693, 0.00329549139377, 0.0589073140693);
        Body body_a, body_b, body_c, body_d;
        Joint joint_a, joint_b, joint_c, joint_d, joint_v;

        // body a
        joint_a = Joint(JointTypeFixed);
        body_a = Body(mass, com, inertia); /* mass, com, inertia*/
        Matrix3_t body_a_rot;
        body_a_rot << 0., 0., 1.,
            0., -1., 0.,
            1., 0., 0.;
        Vector3d body_a_trans(0.001, -0.36, -0.222);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_a, body_a);
        // body b
        body_b = Body(mass, com, inertia);
        joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_b_rot;
        body_b_rot << 0., 1., 0.,
            -1., 0., 0.,
            0., 0., 1.;
        Vector3d body_b_trans(0.139, 0.138, 0.);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_b, body_b);
        // body c
        body_c = Body(mass, com, inertia);
        joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_c_rot;
        body_c_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_c_trans(-0.141, -0.832, 0.);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_c, body_c);
        // body d
        body_d = Body(mass, com, inertia);
        joint_d = Joint(
            JointTypeRevolute,
            Vector3d(0., 0., 1.));

        Matrix3_t body_d_rot;
        body_d_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_d_trans(-0.14, -0.83, 0.);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_c_id, body_d_tf, joint_d, body_d);
        // virtual body
        Vector3d vector3d_zero = Vector3d::Zero();
        Body body_v(virtual_mass, vector3d_zero, vector3d_zero);  // creating the virtual body
        joint_v = Joint(JointTypeRevolute, Vector3d(0., 0., 1.)); // revolute about z, also Joint(JointTypeRevoluteZ); is OK.
        Matrix3_t body_v_rot;
        body_v_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d body_v_trans(-0.062, -0.761, 0.); // child pivot is used Joint l1-l4
        SpatialTransform body_v_tf(body_v_rot, body_v_trans);
        body_virtual_id = model.AddBody(body_d_id, body_v_tf, joint_v, body_v);
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        Matrix3_t p_rot;
        p_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d p_trans(0.07, -0.77, 0.);
        SpatialTransform p_tf(p_rot, p_trans); // predecessor body is l1
        X_p = p_tf;

        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 1, 0, 0), true, 0.05);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 0, 1, 0), true, 0.05);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 1, 0, 0, 0), true, 0.05);

        cs.Bind(model);
        //
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
    }

    RigidBodyDynamics::Model model;
    ConstraintSet cs;
    RigidBodyDynamics::Math::VectorNd Q;
    RigidBodyDynamics::Math::VectorNd QDot;
    RigidBodyDynamics::Math::VectorNd QDDot;
    RigidBodyDynamics::Math::VectorNd Tau;
    SpatialTransform X_p;
    SpatialTransform X_s;
    double mass, virtual_mass;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_virtual_id;
};

class Dynamics
{
public:
    //constructor
    Dynamics();
    Four_bar_linkage farid;
    Four_bar_linkage_closed cl_obj_;

    //methods
    RigidBodyDynamics::Math::VectorNd get_G(std::vector<double> pos);
    VectorNd get_G_ClosedLoop(std::vector<double> pos);
};

// struct FourBarLinkage
// {

//     FourBarLinkage()
//         : model(), cs(), q(), qd(), qdd(), tau(), l1(2.), l2(2.), m1(2.), m2(2.), idB1(0), idB2(0), idB3(0), idB4(0), idB5(0), X_p(Xtrans(Vector3d(l2, 0., 0.))), X_s(Xtrans(Vector3d(0., 0., 0.)))
//     {

//         Body link1 = Body(m1, Vector3d(0.5 * l1, 0., 0.), Vector3d(0., 0., m1 * l1 * l1 / 3.));
//         Body link2 = Body(m2, Vector3d(0.5 * l2, 0., 0.), Vector3d(0., 0., m2 * l2 * l2 / 3.));
//         Vector3d vector3d_zero = Vector3d::Zero();
//         Body virtual_body(0., vector3d_zero, vector3d_zero);
//         Joint joint_rev_z(JointTypeRevoluteZ);

//         idB1 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)),
//                              joint_rev_z, link1);
//         idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)),
//                              joint_rev_z, link2);
//         idB3 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)),
//                              joint_rev_z, link1);
//         idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)),
//                              joint_rev_z, link2);
//         idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)),
//                              joint_rev_z, virtual_body);

//         cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
//                              SpatialVector(0, 0, 0, 1, 0, 0), true, 0.1);
//         cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
//                              SpatialVector(0, 0, 0, 0, 1, 0), true, 0.1);
//         cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
//                              SpatialVector(0, 0, 1, 0, 0, 0), true, 0.1);

//         cs.Bind(model);
//         q = VectorNd::Zero(model.dof_count);
//         qd = VectorNd::Zero(model.dof_count);
//         qdd = VectorNd::Zero(model.dof_count);
//         tau = VectorNd::Zero(model.dof_count);
//     }

//     Model model;
//     ConstraintSet cs;

//     VectorNd q;
//     VectorNd qd;
//     VectorNd qdd;
//     VectorNd tau;

//     double l1;
//     double l2;
//     double m1;
//     double m2;

//     unsigned int idB1;
//     unsigned int idB2;
//     unsigned int idB3;
//     unsigned int idB4;
//     unsigned int idB5;

//     SpatialTransform X_p;
//     SpatialTransform X_s;
// };