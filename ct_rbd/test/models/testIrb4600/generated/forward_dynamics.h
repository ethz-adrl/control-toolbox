#ifndef IIT_ROBOT_TESTIRB4600_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_TESTIRB4600_FORWARD_DYNAMICS_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace testirb4600 {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot testirb4600.
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
    typedef typename iit::testirb4600::tpl::JointState<SCALAR> JointState;
    typedef typename CoreS::Matrix66 Matrix66S;
    
    typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;
    typedef iit::testirb4600::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot testirb4600, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(iit::testirb4600::dyn::tpl::InertiaProperties<TRAIT>& in, MTransforms& tr);
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
    iit::testirb4600::dyn::tpl::InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'link1' :
    Matrix66S link1_AI;
    Velocity link1_a;
    Velocity link1_v;
    Velocity link1_c;
    Force    link1_p;

    Column6DS link1_U;
    SCALAR link1_D;
    SCALAR link1_u;
    // Link 'link2' :
    Matrix66S link2_AI;
    Velocity link2_a;
    Velocity link2_v;
    Velocity link2_c;
    Force    link2_p;

    Column6DS link2_U;
    SCALAR link2_D;
    SCALAR link2_u;
    // Link 'link3' :
    Matrix66S link3_AI;
    Velocity link3_a;
    Velocity link3_v;
    Velocity link3_c;
    Force    link3_p;

    Column6DS link3_U;
    SCALAR link3_D;
    SCALAR link3_u;
    // Link 'link4' :
    Matrix66S link4_AI;
    Velocity link4_a;
    Velocity link4_v;
    Velocity link4_c;
    Force    link4_p;

    Column6DS link4_U;
    SCALAR link4_D;
    SCALAR link4_u;
    // Link 'link5' :
    Matrix66S link5_AI;
    Velocity link5_a;
    Velocity link5_v;
    Velocity link5_c;
    Force    link5_p;

    Column6DS link5_U;
    SCALAR link5_D;
    SCALAR link5_u;
    // Link 'link6' :
    Matrix66S link6_AI;
    Velocity link6_a;
    Velocity link6_v;
    Velocity link6_c;
    Force    link6_p;

    Column6DS link6_U;
    SCALAR link6_D;
    SCALAR link6_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_link1_X_fr_link0)(q);
    (motionTransforms-> fr_link2_X_fr_link1)(q);
    (motionTransforms-> fr_link3_X_fr_link2)(q);
    (motionTransforms-> fr_link4_X_fr_link3)(q);
    (motionTransforms-> fr_link5_X_fr_link4)(q);
    (motionTransforms-> fr_link6_X_fr_link5)(q);
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
