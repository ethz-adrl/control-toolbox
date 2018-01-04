#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ct_InvertedPendulum;
using namespace iit::ct_InvertedPendulum::dyn;

iit::rbd::Vector3d iit::ct_InvertedPendulum::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_InvertedPendulumBase_X_fr_Link1;
    tmpSum += inertiaProps.getMass_Link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link1()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ct_InvertedPendulum::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_InvertedPendulumBase_X_fr_Link1(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
