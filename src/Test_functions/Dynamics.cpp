#include <iostream>
#include <rbdl/rbdl.h>
#include "Dynamics.h"
#include "test_models.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Dynamics::Dynamics()
{
    Four_bar_linkage cl_obj_;
};

unsigned int Dynamics::get_num_joints()
{
    return cl_obj_.model.q_size;
};

MatrixNd Dynamics::calc_EndEffectorJacobian(const VectorNd &Q, unsigned int body_id, const Vector3d &body_point_position)
{
    MatrixNd J = MatrixNd::Zero(3, cl_obj_.model.qdot_size);
    CalcPointJacobian(cl_obj_.model, Q, body_id, body_point_position, J);
    return J;
};

Vector3d Dynamics::calc_EndEffectorVel(const VectorNd &Q, const VectorNd &QDot, unsigned int body_id, const Vector3d &body_point_position)
{
    return CalcPointVelocity(cl_obj_.model, Q, QDot, body_id, body_point_position);
};

VectorNd Dynamics::calc_InverseKinematics(const VectorNd &Qinit, const std::vector<unsigned int> body_id, const std::vector<Vector3d> &body_point, const std::vector<Vector3d> &target_pos, VectorNd &Qres)
{
    bool success = InverseKinematics(cl_obj_.model, Qinit, body_id, body_point, target_pos, Qres);
    if (success == 0)
    {
        std::cout << "The operation was not successful\n";
        exit(0);
    }
    else if (success == 1)
    {
        std::cout << "The operation was successful\n";
    }
    return Qres;
};

Vector3d Dynamics::calc_ForwardKinematics(const Math::VectorNd &Q, unsigned int body_id, const Math::Vector3d &body_point_position)
{
    return CalcBodyToBaseCoordinates(cl_obj_.model, Q, body_id, body_point_position);
};

MatrixNd Dynamics::calc_M(const VectorNd &Q)
{
    Math::MatrixNd M = Math::MatrixNd::Zero(cl_obj_.model.qdot_size, cl_obj_.model.qdot_size);
    CompositeRigidBodyAlgorithm(cl_obj_.model, Q, M);
    return M;
};

VectorNd Dynamics::calc_G(const VectorNd &Q)
{
    cl_obj_.QDot = VectorNd::Zero(cl_obj_.model.qdot_size);
    cl_obj_.QDDot = VectorNd::Zero(cl_obj_.model.qdot_size);
    InverseDynamics(cl_obj_.model, Q, cl_obj_.QDot, cl_obj_.QDDot, cl_obj_.Tau);
    return cl_obj_.Tau;
};

VectorNd Dynamics::calc_InverseDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDot, VectorNd &Tau)
{
    InverseDynamics(cl_obj_.model, Q, QDot, QDDot, Tau);
    return cl_obj_.Tau;
};

VectorNd Dynamics::calc_ForwardDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, VectorNd &QDDot)
{

    ForwardDynamics(cl_obj_.model, Q, QDot, Tau, cl_obj_.QDDot);
    return cl_obj_.QDDot;
};

VectorNd Dynamics::calc_InverseDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDotDesired, ConstraintSet &CS, VectorNd &QDDotOutput, VectorNd &TauOutput)
{
    bool is_fully_actuated = isConstrainedSystemFullyActuated(cl_obj_.model, cl_obj_.Q, cl_obj_.QDot, cl_obj_.cs); //returns success if the model is fully actuated.;
    if (is_fully_actuated == 1)
    {
        std::vector<bool> actuatedDof(cl_obj_.model.q_size, true);
        actuatedDof.at(3) = false; // setting the virual joint(loop constraint) as the passive joint
        cl_obj_.cs.SetActuationMap(cl_obj_.model, actuatedDof);

        for (unsigned int i = 0; i < cl_obj_.model.q_size; i++)
        {
            cl_obj_.Q(i) = Q(i);
        }
        // cl_obj_.Q(3) = 0; // Since this is the loop joint, no matter the joint position the Tau corresponding to this joint is always zero.
        InverseDynamicsConstraints(cl_obj_.model, cl_obj_.Q, cl_obj_.QDot, cl_obj_.QDDot, cl_obj_.cs, cl_obj_.QDDot, cl_obj_.Tau);
        std::cout << cl_obj_.Tau << std::endl;
        return cl_obj_.Tau;
    }

    else
    {
        std::cout << "The model is not fully actuated! Try using other InverseDynamicsRelaxed method.\n";
        exit(0);
    }
};

VectorNd Dynamics::calc_ForwardDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, ConstraintSet &CS, VectorNd &QDDotOutput)
{
    ForwardDynamicsConstraintsDirect(cl_obj_.model, Q, QDot, Tau, CS, QDDotOutput);
    return QDDotOutput;
};

VectorNd Dynamics::calc_MInvTimesTau(const VectorNd &Q, const VectorNd &Tau, VectorNd &QDDot)
{
    CalcMInvTimesTau(cl_obj_.model, Q, Tau, QDDot);
    return QDDot;
};
