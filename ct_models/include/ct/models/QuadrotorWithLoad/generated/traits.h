#ifndef IIT_ROBOGEN__CT_QUADROTOR_TRAITS_H_
#define IIT_ROBOGEN__CT_QUADROTOR_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ct_quadrotor {

namespace tpl{

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ct_quadrotor::tpl::JointState<SCALAR> JointState;

    typedef typename ct_quadrotor::JointIdentifiers JointID;
    typedef typename ct_quadrotor::LinkIdentifiers  LinkID;

    typedef typename ct_quadrotor::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ct_quadrotor::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ct_quadrotor::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ct_quadrotor::tpl::Jacobians<Trait> Jacobians;

    typedef typename ct_quadrotor::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename ct_quadrotor::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename ct_quadrotor::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename ct_quadrotor::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ct_quadrotor::jointsCount;
    static const int links_count  = ct_quadrotor::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ct_quadrotor::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ct_quadrotor::orderedLinkIDs;
}

} // namespace tpl

typedef tpl::Traits<double> Traits;

}
}

#endif
