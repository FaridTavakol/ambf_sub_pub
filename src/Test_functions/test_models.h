#pragma once
#include <rbdl/rbdl.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
struct Four_bar_linkage
{
    Four_bar_linkage() : model(), Q(), QDot(), QDDot(), mass(1), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_virtual_id(), X_s(Xtrans(Vector3d(0., 0., 0.))), X_p(), cs(), virtual_mass(0)
    {

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

    Model model;
    ConstraintSet cs;
    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    SpatialTransform X_p;
    SpatialTransform X_s;
    double mass, virtual_mass;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_virtual_id;
};