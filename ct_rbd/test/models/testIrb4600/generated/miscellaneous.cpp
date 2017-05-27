#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::testirb4600;
using namespace iit::testirb4600::dyn;

iit::rbd::Vector3d iit::testirb4600::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_link0_X_fr_link1;
    tmpSum += inertiaProps.getMass_link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link1()));
    
    tmpX = tmpX * ht.fr_link1_X_fr_link2;
    tmpSum += inertiaProps.getMass_link2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link2()));
    
    tmpX = tmpX * ht.fr_link2_X_fr_link3;
    tmpSum += inertiaProps.getMass_link3() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link3()));
    
    tmpX = tmpX * ht.fr_link3_X_fr_link4;
    tmpSum += inertiaProps.getMass_link4() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link4()));
    
    tmpX = tmpX * ht.fr_link4_X_fr_link5;
    tmpSum += inertiaProps.getMass_link5() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link5()));
    
    tmpX = tmpX * ht.fr_link5_X_fr_link6;
    tmpSum += inertiaProps.getMass_link6() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link6()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::testirb4600::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_link0_X_fr_link1(q);
    ht.fr_link1_X_fr_link2(q);
    ht.fr_link2_X_fr_link3(q);
    ht.fr_link3_X_fr_link4(q);
    ht.fr_link4_X_fr_link5(q);
    ht.fr_link5_X_fr_link6(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
