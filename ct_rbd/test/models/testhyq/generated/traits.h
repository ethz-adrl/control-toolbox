#ifndef IIT_ROBOGEN__TESTHYQ_TRAITS_H_
#define IIT_ROBOGEN__TESTHYQ_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace TestHyQ {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename iit::TestHyQ::tpl::JointState<SCALAR> JointState;

    typedef typename iit::TestHyQ::JointIdentifiers JointID;
    typedef typename iit::TestHyQ::LinkIdentifiers  LinkID;

    typedef typename iit::TestHyQ::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename iit::TestHyQ::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename iit::TestHyQ::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename iit::TestHyQ::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::TestHyQ::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::TestHyQ::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::TestHyQ::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::TestHyQ::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = iit::TestHyQ::jointsCount;
    static const int links_count  = iit::TestHyQ::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return iit::TestHyQ::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return iit::TestHyQ::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits;

}
}

#endif
