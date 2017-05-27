#ifndef IIT_ROBOGEN__TESTIRB4600_TRAITS_H_
#define IIT_ROBOGEN__TESTIRB4600_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace testirb4600 {

namespace tpl{

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename testirb4600::tpl::JointState<SCALAR> JointState;

    typedef typename testirb4600::JointIdentifiers JointID;
    typedef typename testirb4600::LinkIdentifiers  LinkID;

    typedef typename testirb4600::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename testirb4600::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename testirb4600::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename testirb4600::tpl::Jacobians<Trait> Jacobians;

    typedef typename testirb4600::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename testirb4600::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename testirb4600::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename testirb4600::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = testirb4600::jointsCount;
    static const int links_count  = testirb4600::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return testirb4600::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return testirb4600::orderedLinkIDs;
}

} // namespace tpl

typedef tpl::Traits<double> Traits;

}
}

#endif
