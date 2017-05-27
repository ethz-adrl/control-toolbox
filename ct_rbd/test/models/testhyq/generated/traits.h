#ifndef IIT_ROBOGEN__HYQ_TRAITS_H_
#define IIT_ROBOGEN__HYQ_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

#include <iit/rbd/traits/TraitSelector.h>

namespace iit {
namespace HyQ {
namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SCALAR S;

    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait; 

    typedef typename HyQ::tpl::JointState<SCALAR> JointState;

    typedef typename HyQ::JointIdentifiers JointID;
    typedef typename HyQ::LinkIdentifiers  LinkID;

    typedef typename HyQ::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename HyQ::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename HyQ::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename HyQ::tpl::Jacobians<Trait> Jacobians;


    typedef typename HyQ::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename HyQ::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename HyQ::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename HyQ::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = HyQ::jointsCount;
    static const int links_count  = HyQ::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return HyQ::orderedJointIDs;
}

template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return HyQ::orderedLinkIDs;
}


} // namespace tpl

typedef tpl::Traits<double> Traits;



} //namespace HyQ
} //namespace iit

#endif
