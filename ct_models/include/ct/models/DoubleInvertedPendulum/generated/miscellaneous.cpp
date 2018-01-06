#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ct_DoubleInvertedPendulum;
using namespace iit::ct_DoubleInvertedPendulum::dyn;

iit::rbd::Vector3d iit::ct_DoubleInvertedPendulum::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_DoubleInvertedPendulumBase_X_fr_Link1;
    tmpSum += inertiaProps.getMass_Link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link1()));
    
    tmpX = tmpX * ht.fr_Link1_X_fr_Link2;
    tmpSum += inertiaProps.getMass_Link2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link2()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ct_DoubleInvertedPendulum::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_DoubleInvertedPendulumBase_X_fr_Link1(q);
    ht.fr_Link1_X_fr_Link2(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
