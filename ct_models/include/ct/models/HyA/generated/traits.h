#ifndef IIT_ROBOGEN__CT_HYA_TRAITS_H_
#define IIT_ROBOGEN__CT_HYA_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ct_HyA {

namespace tpl{

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ct_HyA::tpl::JointState<SCALAR> JointState;

    typedef typename ct_HyA::JointIdentifiers JointID;
    typedef typename ct_HyA::LinkIdentifiers  LinkID;

    typedef typename ct_HyA::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ct_HyA::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ct_HyA::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ct_HyA::tpl::Jacobians<Trait> Jacobians;

    typedef typename ct_HyA::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename ct_HyA::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename ct_HyA::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename ct_HyA::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ct_HyA::jointsCount;
    static const int links_count  = ct_HyA::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ct_HyA::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ct_HyA::orderedLinkIDs;
}

} // namespace tpl

typedef tpl::Traits<double> Traits;

}
}

#endif
