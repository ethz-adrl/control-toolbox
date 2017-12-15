#ifndef IIT_ROBOGEN__CT_INVERTEDPENDULUM_TRAITS_H_
#define IIT_ROBOGEN__CT_INVERTEDPENDULUM_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ct_InvertedPendulum {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename ct_InvertedPendulum::JointIdentifiers JointID;
    typedef typename ct_InvertedPendulum::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ct_InvertedPendulum::tpl::JointState<SCALAR> JointState;



    typedef typename ct_InvertedPendulum::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ct_InvertedPendulum::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ct_InvertedPendulum::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ct_InvertedPendulum::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::ct_InvertedPendulum::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::ct_InvertedPendulum::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::ct_InvertedPendulum::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ct_InvertedPendulum::jointsCount;
    static const int links_count  = ct_InvertedPendulum::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ct_InvertedPendulum::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ct_InvertedPendulum::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
