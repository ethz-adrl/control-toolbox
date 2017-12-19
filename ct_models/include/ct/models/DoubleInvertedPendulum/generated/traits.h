#ifndef IIT_ROBOGEN__CT_DOUBLEINVERTEDPENDULUM_TRAITS_H_
#define IIT_ROBOGEN__CT_DOUBLEINVERTEDPENDULUM_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ct_DoubleInvertedPendulum {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename ct_DoubleInvertedPendulum::JointIdentifiers JointID;
    typedef typename ct_DoubleInvertedPendulum::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ct_DoubleInvertedPendulum::tpl::JointState<SCALAR> JointState;



    typedef typename ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ct_DoubleInvertedPendulum::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ct_DoubleInvertedPendulum::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ct_DoubleInvertedPendulum::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::ct_DoubleInvertedPendulum::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::ct_DoubleInvertedPendulum::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ct_DoubleInvertedPendulum::jointsCount;
    static const int links_count  = ct_DoubleInvertedPendulum::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ct_DoubleInvertedPendulum::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ct_DoubleInvertedPendulum::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
