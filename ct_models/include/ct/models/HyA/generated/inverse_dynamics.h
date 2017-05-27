#ifndef IIT_CT_HYA_INVERSE_DYNAMICS_H_
#define IIT_CT_HYA_INVERSE_DYNAMICS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace ct_HyA {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot ct_HyA.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

namespace tpl {

template <typename TRAIT>
class InverseDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename TRAIT::Scalar SCALAR;

    typedef iit::rbd::Core<SCALAR> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;
    typedef iit::ct_HyA::tpl::JointState<SCALAR> JointState;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::ct_HyA::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;            

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ct_HyA, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(IProperties& in, MTransforms& tr);

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JointState& jForces,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JointState& jForces, const JointState& q);
    void G_terms(JointState& jForces);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
    void C_terms(JointState& jForces, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Velocity& getVelocity_Shoulder_AA() const { return Shoulder_AA_v; }
    const Acceleration& getAcceleration_Shoulder_AA() const { return Shoulder_AA_a; }
    const Force& getForce_Shoulder_AA() const { return Shoulder_AA_f; }
    const Velocity& getVelocity_Shoulder_FE() const { return Shoulder_FE_v; }
    const Acceleration& getAcceleration_Shoulder_FE() const { return Shoulder_FE_a; }
    const Force& getForce_Shoulder_FE() const { return Shoulder_FE_f; }
    const Velocity& getVelocity_Humerus_R() const { return Humerus_R_v; }
    const Acceleration& getAcceleration_Humerus_R() const { return Humerus_R_a; }
    const Force& getForce_Humerus_R() const { return Humerus_R_f; }
    const Velocity& getVelocity_Elbow_FE() const { return Elbow_FE_v; }
    const Acceleration& getAcceleration_Elbow_FE() const { return Elbow_FE_a; }
    const Force& getForce_Elbow_FE() const { return Elbow_FE_f; }
    const Velocity& getVelocity_Wrist_R() const { return Wrist_R_v; }
    const Acceleration& getAcceleration_Wrist_R() const { return Wrist_R_a; }
    const Force& getForce_Wrist_R() const { return Wrist_R_f; }
    const Velocity& getVelocity_Wrist_FE() const { return Wrist_FE_v; }
    const Acceleration& getAcceleration_Wrist_FE() const { return Wrist_FE_a; }
    const Force& getForce_Wrist_FE() const { return Wrist_FE_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'Shoulder_AA' :
    const InertiaMatrix& Shoulder_AA_I;
    Velocity      Shoulder_AA_v;
    Acceleration  Shoulder_AA_a;
    Force         Shoulder_AA_f;
    // Link 'Shoulder_FE' :
    const InertiaMatrix& Shoulder_FE_I;
    Velocity      Shoulder_FE_v;
    Acceleration  Shoulder_FE_a;
    Force         Shoulder_FE_f;
    // Link 'Humerus_R' :
    const InertiaMatrix& Humerus_R_I;
    Velocity      Humerus_R_v;
    Acceleration  Humerus_R_a;
    Force         Humerus_R_f;
    // Link 'Elbow_FE' :
    const InertiaMatrix& Elbow_FE_I;
    Velocity      Elbow_FE_v;
    Acceleration  Elbow_FE_a;
    Force         Elbow_FE_f;
    // Link 'Wrist_R' :
    const InertiaMatrix& Wrist_R_I;
    Velocity      Wrist_R_v;
    Acceleration  Wrist_R_a;
    Force         Wrist_R_f;
    // Link 'Wrist_FE' :
    const InertiaMatrix& Wrist_FE_I;
    Velocity      Wrist_FE_v;
    Acceleration  Wrist_FE_a;
    Force         Wrist_FE_f;


private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_Shoulder_AA_X_fr_HyABase)(q);
    (xm->fr_Shoulder_FE_X_fr_Shoulder_AA)(q);
    (xm->fr_Humerus_R_X_fr_Shoulder_FE)(q);
    (xm->fr_Elbow_FE_X_fr_Humerus_R)(q);
    (xm->fr_Wrist_R_X_fr_Elbow_FE)(q);
    (xm->fr_Wrist_FE_X_fr_Wrist_R)(q);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::G_terms(JointState& jForces, const JointState& q)
{
    setJointStatus(q);
    G_terms(jForces);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms(jForces, qd);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, qd, qdd, fext);
}

} // namespace tpl

typedef tpl::InverseDynamics<rbd::DoubleTrait> InverseDynamics;

}
}

}

#include "inverse_dynamics.impl.h"

#endif
