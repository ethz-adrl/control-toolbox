#ifndef IIT_ROBOGEN__HYQ_TRAITS_H_
#define IIT_ROBOGEN__HYQ_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace HyQ {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename iit::HyQ::tpl::JointState<SCALAR> JointState;

    typedef typename iit::HyQ::JointIdentifiers JointID;
    typedef typename iit::HyQ::LinkIdentifiers  LinkID;

    typedef typename iit::HyQ::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename iit::HyQ::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename iit::HyQ::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename iit::HyQ::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::HyQ::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::HyQ::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::HyQ::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::HyQ::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = iit::HyQ::jointsCount;
    static const int links_count  = iit::HyQ::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return iit::HyQ::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return iit::HyQ::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits;

}
}

#endif
