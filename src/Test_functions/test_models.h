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
                                           /* **note** the rotation matrix is a 3*3 concise representation of a 6*6 spacial matrix.
          The rotation matrix should be inserted as a transpose of the rotation matrix.*/
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
struct KUKA_LBR
{
    KUKA_LBR() : model(), mass(1), Q(), QDot(), QDDot(), Tau(), body_base_id(), body_l1_id(), body_l2_id(), body_l3_id(), body_l4_id(), body_l5_id(), body_l6_id(), body_l7_id()
    {

        model.gravity = Vector3d(0., 0., -9.81);
        Body body_base, body_l1, body_l2, body_l3, body_l4, body_l5, body_l6, body_l7;
        Joint Joint_RevZ, Joint_Fixed;
        Joint_Fixed = Joint(JointTypeFixed);
        Joint_RevZ = Joint(JointTypeRevoluteZ);

        // body base
        body_base = Body(mass, Vector3d(0., 0., 0.), Vector3d(0., 0., 0.)); /* mass, com, inertia*/
        Matrix3_t world_base_rot;
        /* **note** the rotation matrix is a 3*3 concise representation of a 6*6 spacial matrix.
          The rotation matrix should be inserted as a transpose of the rotation matrix.*/
        world_base_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d world_base_trans(0., 0., 0.);
        SpatialTransform world_base_tf(world_base_rot, world_base_trans);
        body_base_id = model.AddBody(0, world_base_tf, Joint_Fixed, body_base);
        // body l1
        body_l1 = Body(mass, Vector3d(0.0, -0.017, 0.134), Vector3d(0.00815814, 0.007363868, 0.003293455)); /* mass, com, inertia*/
        Matrix3_t base_l1_rot;
        base_l1_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d base_l1_trans(0., 0., 0.103);
        SpatialTransform base_l1_tf(base_l1_rot, base_l1_trans);
        body_l1_id = model.AddBody(body_base_id, base_l1_tf, Joint_RevZ, body_l1);
        // body l2
        body_l2 = Body(mass, Vector3d(0.0, -0.074, 0.009), Vector3d(0.00812252, 0.00329668, 0.00733904)); /* mass, com, inertia*/
        Matrix3_t l1_l2_rot;
        l1_l2_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l1_l2_trans(0.0, 0.013, 0.209);
        SpatialTransform l1_l2_tf(l1_l2_rot, l1_l2_trans);
        body_l2_id = model.AddBody(body_l1_id, l1_l2_tf, Joint_RevZ, body_l2);
        // body l3
        body_l3 = Body(mass, Vector3d(0.0, 0.017, 0.134), Vector3d(0.008159, 0.007421, 0.00330)); /* mass, com, inertia*/
        Matrix3_t l2_l3_rot;
        l2_l3_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l2_l3_trans(0.0, -0.194, -0.009);
        SpatialTransform l2_l3_tf(l2_l3_rot, l2_l3_trans);
        body_l3_id = model.AddBody(body_l2_id, l2_l3_tf, Joint_RevZ, body_l3);
        // body l4
        body_l4 = Body(mass, Vector3d(-0.001, 0.081, 0.008), Vector3d(0.0081471, 0.003297, 0.0073715)); /* mass, com, inertia*/
        Matrix3_t l3_l4_rot;
        l3_l4_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l3_l4_trans(0.0, -0.013, 0.202);
        SpatialTransform l3_l4_tf(l3_l4_rot, l3_l4_trans);
        body_l4_id = model.AddBody(body_l3_id, l3_l4_tf, Joint_RevZ, body_l4);
        // body l5
        body_l5 = Body(mass, Vector3d(0.0, -0.017, 0.129), Vector3d(0.0077265, 0.006950, 0.00329)); /* mass, com, inertia*/
        Matrix3_t l4_l5_rot;
        l4_l5_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l4_l5_trans(-0.002, 0.202, -0.008);
        SpatialTransform l4_l5_tf(l4_l5_rot, l4_l5_trans);
        body_l5_id = model.AddBody(body_l4_id, l4_l5_tf, Joint_RevZ, body_l5);
        // body l6
        body_l6 = Body(mass, Vector3d(0.0, 0.007, 0.068), Vector3d(0.002983, 0.003299, 0.003146)); /* mass, com, inertia*/
        Matrix3_t l5_l6_rot;
        l5_l6_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l5_l6_trans(0.002, -0.052, 0.204);
        SpatialTransform l5_l6_tf(l5_l6_rot, l5_l6_trans);
        body_l6_id = model.AddBody(body_l5_id, l5_l6_tf, Joint_RevZ, body_l6);
        // body l7
        body_l7 = Body(mass, Vector3d(0.006, 0.0, 0.015), Vector3d(0.000651, 0.0006512, 0.00112)); /* mass, com, inertia*/
        Matrix3_t l6_l7_rot;
        l6_l7_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l6_l7_trans(-0.003, -0.05, 0.053);
        SpatialTransform l6_l7_tf(l6_l7_rot, l6_l7_trans);
        body_l7_id = model.AddBody(body_l6_id, l6_l7_tf, Joint_RevZ, body_l7);
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
    }

    Model model;
    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    double mass;
    unsigned int body_l1_id, body_l2_id, body_l3_id, body_l4_id, body_l5_id, body_l6_id, body_l7_id, body_base_id;
};