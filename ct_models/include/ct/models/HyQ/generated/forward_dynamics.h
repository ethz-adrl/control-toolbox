#ifndef IIT_ROBOT_HYQ_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_HYQ_FORWARD_DYNAMICS_H_

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
namespace HyQ {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot HyQ.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;
    
    typedef typename CoreS::ForceVector Force;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename iit::HyQ::tpl::JointState<Scalar> JointState;
    typedef typename CoreS::Matrix66 Matrix66S;
    
    typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
    typedef iit::HyQ::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot HyQ, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(iit::HyQ::dyn::tpl::InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param trunk_a
     * \param trunk_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& trunk_a, // output parameters,
       const Velocity& trunk_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& trunk_a, // output parameters,
        const Velocity& trunk_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    iit::HyQ::dyn::tpl::InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'trunk'
    Matrix66S trunk_AI;
    Force trunk_p;

    // Link 'LF_hipassembly' :
    Matrix66S LF_hipassembly_AI;
    Velocity LF_hipassembly_a;
    Velocity LF_hipassembly_v;
    Velocity LF_hipassembly_c;
    Force    LF_hipassembly_p;

    Column6DS LF_hipassembly_U;
    Scalar LF_hipassembly_D;
    Scalar LF_hipassembly_u;
    // Link 'LF_upperleg' :
    Matrix66S LF_upperleg_AI;
    Velocity LF_upperleg_a;
    Velocity LF_upperleg_v;
    Velocity LF_upperleg_c;
    Force    LF_upperleg_p;

    Column6DS LF_upperleg_U;
    Scalar LF_upperleg_D;
    Scalar LF_upperleg_u;
    // Link 'LF_lowerleg' :
    Matrix66S LF_lowerleg_AI;
    Velocity LF_lowerleg_a;
    Velocity LF_lowerleg_v;
    Velocity LF_lowerleg_c;
    Force    LF_lowerleg_p;

    Column6DS LF_lowerleg_U;
    Scalar LF_lowerleg_D;
    Scalar LF_lowerleg_u;
    // Link 'RF_hipassembly' :
    Matrix66S RF_hipassembly_AI;
    Velocity RF_hipassembly_a;
    Velocity RF_hipassembly_v;
    Velocity RF_hipassembly_c;
    Force    RF_hipassembly_p;

    Column6DS RF_hipassembly_U;
    Scalar RF_hipassembly_D;
    Scalar RF_hipassembly_u;
    // Link 'RF_upperleg' :
    Matrix66S RF_upperleg_AI;
    Velocity RF_upperleg_a;
    Velocity RF_upperleg_v;
    Velocity RF_upperleg_c;
    Force    RF_upperleg_p;

    Column6DS RF_upperleg_U;
    Scalar RF_upperleg_D;
    Scalar RF_upperleg_u;
    // Link 'RF_lowerleg' :
    Matrix66S RF_lowerleg_AI;
    Velocity RF_lowerleg_a;
    Velocity RF_lowerleg_v;
    Velocity RF_lowerleg_c;
    Force    RF_lowerleg_p;

    Column6DS RF_lowerleg_U;
    Scalar RF_lowerleg_D;
    Scalar RF_lowerleg_u;
    // Link 'LH_hipassembly' :
    Matrix66S LH_hipassembly_AI;
    Velocity LH_hipassembly_a;
    Velocity LH_hipassembly_v;
    Velocity LH_hipassembly_c;
    Force    LH_hipassembly_p;

    Column6DS LH_hipassembly_U;
    Scalar LH_hipassembly_D;
    Scalar LH_hipassembly_u;
    // Link 'LH_upperleg' :
    Matrix66S LH_upperleg_AI;
    Velocity LH_upperleg_a;
    Velocity LH_upperleg_v;
    Velocity LH_upperleg_c;
    Force    LH_upperleg_p;

    Column6DS LH_upperleg_U;
    Scalar LH_upperleg_D;
    Scalar LH_upperleg_u;
    // Link 'LH_lowerleg' :
    Matrix66S LH_lowerleg_AI;
    Velocity LH_lowerleg_a;
    Velocity LH_lowerleg_v;
    Velocity LH_lowerleg_c;
    Force    LH_lowerleg_p;

    Column6DS LH_lowerleg_U;
    Scalar LH_lowerleg_D;
    Scalar LH_lowerleg_u;
    // Link 'RH_hipassembly' :
    Matrix66S RH_hipassembly_AI;
    Velocity RH_hipassembly_a;
    Velocity RH_hipassembly_v;
    Velocity RH_hipassembly_c;
    Force    RH_hipassembly_p;

    Column6DS RH_hipassembly_U;
    Scalar RH_hipassembly_D;
    Scalar RH_hipassembly_u;
    // Link 'RH_upperleg' :
    Matrix66S RH_upperleg_AI;
    Velocity RH_upperleg_a;
    Velocity RH_upperleg_v;
    Velocity RH_upperleg_c;
    Force    RH_upperleg_p;

    Column6DS RH_upperleg_U;
    Scalar RH_upperleg_D;
    Scalar RH_upperleg_u;
    // Link 'RH_lowerleg' :
    Matrix66S RH_lowerleg_AI;
    Velocity RH_lowerleg_a;
    Velocity RH_lowerleg_v;
    Velocity RH_lowerleg_c;
    Force    RH_lowerleg_p;

    Column6DS RH_lowerleg_U;
    Scalar RH_lowerleg_D;
    Scalar RH_lowerleg_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_LF_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly)(q);
    (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg)(q);
    (motionTransforms-> fr_RF_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly)(q);
    (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg)(q);
    (motionTransforms-> fr_LH_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_LH_upperleg_X_fr_LH_hipassembly)(q);
    (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg)(q);
    (motionTransforms-> fr_RH_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_RH_upperleg_X_fr_RH_hipassembly)(q);
    (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg)(q);
}

template<typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd, Acceleration& trunk_a, // output parameters,
    const Velocity& trunk_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, trunk_a, trunk_v, g, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
