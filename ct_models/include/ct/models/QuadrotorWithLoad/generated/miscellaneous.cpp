#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ct_quadrotor;
using namespace iit::ct_quadrotor::dyn;

iit::rbd::Vector3d iit::ct_quadrotor::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_body() * inertiaProps.getMass_body();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_body_X_fr_link1;
    tmpSum += inertiaProps.getMass_link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link1()));
    
    tmpX = tmpX * ht.fr_link1_X_fr_link2;
    tmpSum += inertiaProps.getMass_link2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link2()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ct_quadrotor::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_body_X_fr_link1(q);
    ht.fr_link1_X_fr_link2(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
