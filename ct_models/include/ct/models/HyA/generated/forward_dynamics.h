#ifndef IIT_ROBOT_CT_HYA_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_CT_HYA_FORWARD_DYNAMICS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace ct_HyA {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot ct_HyA.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl{

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar SCALAR;

    typedef iit::rbd::Core<SCALAR> CoreS;

    typedef LinkDataMap<typename CoreS::ForceVector> ExtForces;
    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename iit::ct_HyA::tpl::JointState<SCALAR> JointState;
    typedef typename CoreS::Matrix66 Matrix66S;
    
    typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;
    typedef iit::ct_HyA::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ct_HyA, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(iit::ct_HyA::dyn::tpl::InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    iit::ct_HyA::dyn::tpl::InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'Shoulder_AA' :
    Matrix66S Shoulder_AA_AI;
    Velocity Shoulder_AA_a;
    Velocity Shoulder_AA_v;
    Velocity Shoulder_AA_c;
    Force    Shoulder_AA_p;

    Column6DS Shoulder_AA_U;
    SCALAR Shoulder_AA_D;
    SCALAR Shoulder_AA_u;
    // Link 'Shoulder_FE' :
    Matrix66S Shoulder_FE_AI;
    Velocity Shoulder_FE_a;
    Velocity Shoulder_FE_v;
    Velocity Shoulder_FE_c;
    Force    Shoulder_FE_p;

    Column6DS Shoulder_FE_U;
    SCALAR Shoulder_FE_D;
    SCALAR Shoulder_FE_u;
    // Link 'Humerus_R' :
    Matrix66S Humerus_R_AI;
    Velocity Humerus_R_a;
    Velocity Humerus_R_v;
    Velocity Humerus_R_c;
    Force    Humerus_R_p;

    Column6DS Humerus_R_U;
    SCALAR Humerus_R_D;
    SCALAR Humerus_R_u;
    // Link 'Elbow_FE' :
    Matrix66S Elbow_FE_AI;
    Velocity Elbow_FE_a;
    Velocity Elbow_FE_v;
    Velocity Elbow_FE_c;
    Force    Elbow_FE_p;

    Column6DS Elbow_FE_U;
    SCALAR Elbow_FE_D;
    SCALAR Elbow_FE_u;
    // Link 'Wrist_R' :
    Matrix66S Wrist_R_AI;
    Velocity Wrist_R_a;
    Velocity Wrist_R_v;
    Velocity Wrist_R_c;
    Force    Wrist_R_p;

    Column6DS Wrist_R_U;
    SCALAR Wrist_R_D;
    SCALAR Wrist_R_u;
    // Link 'Wrist_FE' :
    Matrix66S Wrist_FE_AI;
    Velocity Wrist_FE_a;
    Velocity Wrist_FE_v;
    Velocity Wrist_FE_c;
    Force    Wrist_FE_p;

    Column6DS Wrist_FE_U;
    SCALAR Wrist_FE_D;
    SCALAR Wrist_FE_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_Shoulder_AA_X_fr_HyABase)(q);
    (motionTransforms-> fr_Shoulder_FE_X_fr_Shoulder_AA)(q);
    (motionTransforms-> fr_Humerus_R_X_fr_Shoulder_FE)(q);
    (motionTransforms-> fr_Elbow_FE_X_fr_Humerus_R)(q);
    (motionTransforms-> fr_Wrist_R_X_fr_Elbow_FE)(q);
    (motionTransforms-> fr_Wrist_FE_X_fr_Wrist_R)(q);
}

template<typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

} // namespace tpl

typedef tpl::ForwardDynamics<rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
