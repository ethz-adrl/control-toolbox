#ifndef IIT_TESTIRB4600_INVERSE_DYNAMICS_H_
#define IIT_TESTIRB4600_INVERSE_DYNAMICS_H_

#include <Eigen/Dense>
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
namespace testirb4600 {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot testirb4600.
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
    typedef iit::testirb4600::tpl::JointState<SCALAR> JointState;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::testirb4600::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;            

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot testirb4600, which will be used by this instance
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
    const Velocity& getVelocity_link1() const { return link1_v; }
    const Acceleration& getAcceleration_link1() const { return link1_a; }
    const Force& getForce_link1() const { return link1_f; }
    const Velocity& getVelocity_link2() const { return link2_v; }
    const Acceleration& getAcceleration_link2() const { return link2_a; }
    const Force& getForce_link2() const { return link2_f; }
    const Velocity& getVelocity_link3() const { return link3_v; }
    const Acceleration& getAcceleration_link3() const { return link3_a; }
    const Force& getForce_link3() const { return link3_f; }
    const Velocity& getVelocity_link4() const { return link4_v; }
    const Acceleration& getAcceleration_link4() const { return link4_a; }
    const Force& getForce_link4() const { return link4_f; }
    const Velocity& getVelocity_link5() const { return link5_v; }
    const Acceleration& getAcceleration_link5() const { return link5_a; }
    const Force& getForce_link5() const { return link5_f; }
    const Velocity& getVelocity_link6() const { return link6_v; }
    const Acceleration& getAcceleration_link6() const { return link6_a; }
    const Force& getForce_link6() const { return link6_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'link1' :
    const InertiaMatrix& link1_I;
    Velocity      link1_v;
    Acceleration  link1_a;
    Force         link1_f;
    // Link 'link2' :
    const InertiaMatrix& link2_I;
    Velocity      link2_v;
    Acceleration  link2_a;
    Force         link2_f;
    // Link 'link3' :
    const InertiaMatrix& link3_I;
    Velocity      link3_v;
    Acceleration  link3_a;
    Force         link3_f;
    // Link 'link4' :
    const InertiaMatrix& link4_I;
    Velocity      link4_v;
    Acceleration  link4_a;
    Force         link4_f;
    // Link 'link5' :
    const InertiaMatrix& link5_I;
    Velocity      link5_v;
    Acceleration  link5_a;
    Force         link5_f;
    // Link 'link6' :
    const InertiaMatrix& link6_I;
    Velocity      link6_v;
    Acceleration  link6_a;
    Force         link6_f;


private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_link1_X_fr_link0)(q);
    (xm->fr_link2_X_fr_link1)(q);
    (xm->fr_link3_X_fr_link2)(q);
    (xm->fr_link4_X_fr_link3)(q);
    (xm->fr_link5_X_fr_link4)(q);
    (xm->fr_link6_X_fr_link5)(q);
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
