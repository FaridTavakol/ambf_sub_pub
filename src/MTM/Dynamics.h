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
        Vector3d inertia(0.058907, 0.003295, 0.058907);
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
                             SpatialVector(0, 0, 0, 1, 0, 0), true, 0.1);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 0, 1, 0), true, 0.1);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 1, 0, 0, 0), true, 0.1);

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

struct MTM_closed
{ /*constructor for the struct*/
    MTM_closed() : model(), Q(), QDot(), QDDot(), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_e_id(), body_f_id(), body_g_id(), body_h_id(), body_i_id(), body_j_id(), body_virtual_id(), X_s(Xtrans(Vector3d(0., 0., 0.))), X_p(), cs(), virtual_mass(0)
    {
        using namespace RigidBodyDynamics;
        using namespace RigidBodyDynamics::Math;
        model.gravity = Vector3d(0., 0., -9.81);
        Body body_a, body_b, body_c, body_d, body_e, body_f, body_g, body_h, body_i, body_j;
        Joint joint_rev_z, joint_fix, joint_v;
        joint_fix = Joint(JointTypeFixed);
        joint_rev_z = Joint(JointTypeRevoluteZ);
        joint_v = Joint(JointTypeRevoluteZ);

        // body a top panel
        Vector3d com_a(-0.008, 0.064, 0.028);
        Vector3d inertia_a(0.01, 0.01, 0.01);
        body_a = Body(1., com_a, inertia_a); /* mass, com, inertia*/
        Matrix3_t body_a_rot;
        body_a_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d body_a_trans(0.0, 0.0, 0.0);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_fix, body_a);
        // body b shoulder pitch
        Vector3d com_b(0.003, 0.035, -0.119);
        Vector3d inertia_b(0.00917612905846, 0.00935165844593, 0.00447358060957);
        body_b = Body(1., com_b, inertia_b);
        Matrix3_t body_b_rot;
        body_b_rot << 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_b_trans(0.0, 0.0, 0.);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_rev_z, body_b);
        // body c arm parallel
        Vector3d com_c(-0.109, 0.012, 0.02);
        Vector3d inertia_c(0.0028003998026, 0.0134169293445, 0.0113575925399);
        body_c = Body(0.9, com_c, inertia_c);
        Matrix3_t body_c_rot;
        body_c_rot << 0.0, 0.0, 1.0,
            1.0, 0.005, 0.0,
            -0.005, 1.0, 0.0;
        Vector3d body_c_trans(0.0, 0.0, -0.19);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_rev_z, body_c);
        // body d arm parallel 1
        Vector3d com_d(0.038, 0.0, -0.007);
        Vector3d inertia_d(0.000104513350282, 0.000324014608013, 0.000373281422509);
        body_c = Body(0.15, com_d, inertia_d);
        Matrix3_t body_d_rot;
        body_d_rot << -0.0002, -1.0, 0.0,
            1.0, -0.0002, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_d_trans(0.0, 0.0, 0.065);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_c_id, body_d_tf, joint_rev_z, body_d);
        // body e arm front
        Vector3d com_e(0.0, -0.14, -0.007);
        Vector3d inertia_e(0.00110227006662, 0.00000772914692154, 0.00110194245711);
        body_e = Body(0.15, com_e, inertia_e);
        Matrix3_t body_e_rot;
        body_e_rot << 1.0, 0.0, 0.,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_e_trans(0.101, 0.0, -0.03);
        SpatialTransform body_e_tf(body_e_rot, body_e_trans);
        body_e_id = model.AddBody(body_d_id, body_e_tf, joint_rev_z, body_e);
        // virtual body end of arm front joint location with bottom arm
        Vector3d vector3d_zero = Vector3d::Zero();
        Body body_v(virtual_mass, vector3d_zero, vector3d_zero);  // creating the virtual body
        joint_v = Joint(JointTypeRevolute, Vector3d(0., 0., 1.)); // revolute about z, also Joint(JointTypeRevoluteZ); is OK.
        Matrix3_t body_v_rot;                                     //changed
        body_v_rot << -1.0, -0.0004, -0.002,
            0.0004, -1.0, 0.0,
            -0.002, 0.0, 1.0;
        Vector3d body_v_trans(0.0, -0.28, -0.035);
        SpatialTransform body_v_tf(body_v_rot, body_v_trans);
        body_virtual_id = model.AddBody(body_e_id, body_v_tf, joint_v, body_v);
        // body f(bottom Arm)
        Vector3d com_f(-0.188, -0.008, 0.0);
        Vector3d inertia_f(0.000255941352746, 0.0105760140742, 0.0105499806308);
        body_f = Body(0.8, com_f, inertia_f);
        Matrix3_t body_f_rot;
        body_f_rot << -0.0002, 1.0, 0.0,
            -1.0, -0.0002, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_f_trans(-0.279, -0.0, 0.0);
        SpatialTransform body_f_tf(body_f_rot, body_f_trans);
        body_f_id = model.AddBody(body_c_id, body_f_tf, joint_rev_z, body_f);
        // body g wrist platform
        Vector3d com_g(0.0, -0.055, -0.054);
        Vector3d inertia_g(0.00154079803571, 0.000741331628791, 0.000993883092147);
        body_g = Body(0.4, com_g, inertia_g);
        Matrix3_t body_g_rot;
        body_g_rot << 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, -1.0, 0.0;
        Vector3d body_g_trans(-0.364, -0.15, -0.0);
        SpatialTransform body_g_tf(body_g_rot, body_g_trans);
        body_g_id = model.AddBody(body_f_id, body_g_tf, joint_rev_z, body_g);
        // body h wrist pitch
        Vector3d com_h(0.0, 0.041, -0.037);
        Vector3d inertia_h(0.000397858856814, 0.000210744513123, 0.000223359835719);
        body_h = Body(0.2, com_h, inertia_h);
        Matrix3_t body_h_rot;
        body_h_rot << 1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
            0.0, 1.0, 0.0;
        Vector3d body_h_trans(0.0, 0.0, 0.0);
        SpatialTransform body_h_tf(body_h_rot, body_h_trans);
        body_h_id = model.AddBody(body_g_id, body_h_tf, joint_rev_z, body_h);
        // body i wrist yaw
        Vector3d com_i(-0.109, 0.012, 0.02);
        Vector3d inertia_i(0.000249325233144, 0.000131620327094, 0.000131620327094);
        body_i = Body(0.2, com_i, inertia_i);
        Matrix3_t body_i_rot;
        body_i_rot << -0.0002, 0.0, 1.0,
            -1.0, 0.0, -0.0002,
            0.0, -1.0, 0.0;
        Vector3d body_i_trans(0.0, 0.002, 0.);
        SpatialTransform body_i_tf(body_i_rot, body_i_trans);
        body_i_id = model.AddBody(body_h_id, body_i_tf, joint_rev_z, body_i);
        // body j wrist roll
        Vector3d com_j(0.0, 0.0, -0.036);
        Vector3d inertia_j(0.00006, 0.000056, 0.000029);
        body_j = Body(0.1, com_j, inertia_j);
        Matrix3_t body_j_rot;
        body_j_rot << -0.0002, 0.0, -1.0,
            1.0, 0.009, -0.0002,
            0.009, -1.0, 0.0;
        Vector3d body_j_trans(0.0, -0.039, -0.0);
        SpatialTransform body_j_tf(body_j_rot, body_j_trans);
        body_j_id = model.AddBody(body_i_id, body_j_tf, joint_rev_z, body_j);

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        Matrix3_t p_rot;
        p_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d p_trans(-0.101, 0.001, 0.0);
        SpatialTransform p_tf(p_rot, p_trans); // predecessor body is Bottom arm
        X_p = p_tf;

        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 1, 0, 0), true, 0.1);
        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 0, 1, 0), true, 0.1);
        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 1, 0, 0, 0), true, 0.1);

        cs.Bind(model);
        //
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
        std::cout << model.dof_count << "is dof size" << std::endl;
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
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_virtual_id, body_e_id, body_f_id, body_g_id, body_h_id, body_i_id, body_j_id;
};
class Dynamics
{
public:
    //constructor
    Dynamics();
    Four_bar_linkage farid;
    Four_bar_linkage_closed cl_obj_;
    MTM_closed mtm_obj_;

    //methods
    VectorNd get_G(std::vector<double> pos);
    VectorNd get_G_ClosedLoop(std::vector<double> pos);
    VectorNd get_G_MTM(std::vector<double> pos);
};