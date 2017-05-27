#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::HyQ;
using namespace iit::HyQ::dyn;

iit::rbd::Vector3d iit::HyQ::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_trunk() * inertiaProps.getMass_trunk();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_LF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_LH_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RH_HAA_chain;
    
    
    base_X_LF_HAA_chain = tmpX * ht.fr_trunk_X_fr_LF_hipassembly;
    tmpSum += inertiaProps.getMass_LF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_hipassembly()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_hipassembly_X_fr_LF_upperleg;
    tmpSum += inertiaProps.getMass_LF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_upperleg()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_upperleg_X_fr_LF_lowerleg;
    tmpSum += inertiaProps.getMass_LF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_lowerleg()));
    
    base_X_RF_HAA_chain = tmpX * ht.fr_trunk_X_fr_RF_hipassembly;
    tmpSum += inertiaProps.getMass_RF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_hipassembly()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_hipassembly_X_fr_RF_upperleg;
    tmpSum += inertiaProps.getMass_RF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_upperleg()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_upperleg_X_fr_RF_lowerleg;
    tmpSum += inertiaProps.getMass_RF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_lowerleg()));
    
    base_X_LH_HAA_chain = tmpX * ht.fr_trunk_X_fr_LH_hipassembly;
    tmpSum += inertiaProps.getMass_LH_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_hipassembly()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_hipassembly_X_fr_LH_upperleg;
    tmpSum += inertiaProps.getMass_LH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_upperleg()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_upperleg_X_fr_LH_lowerleg;
    tmpSum += inertiaProps.getMass_LH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_lowerleg()));
    
    base_X_RH_HAA_chain = tmpX * ht.fr_trunk_X_fr_RH_hipassembly;
    tmpSum += inertiaProps.getMass_RH_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_hipassembly()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_hipassembly_X_fr_RH_upperleg;
    tmpSum += inertiaProps.getMass_RH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_upperleg()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_upperleg_X_fr_RH_lowerleg;
    tmpSum += inertiaProps.getMass_RH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_lowerleg()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::HyQ::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_trunk_X_fr_LF_hipassembly(q);
    ht.fr_trunk_X_fr_RF_hipassembly(q);
    ht.fr_trunk_X_fr_LH_hipassembly(q);
    ht.fr_trunk_X_fr_RH_hipassembly(q);
    ht.fr_LF_hipassembly_X_fr_LF_upperleg(q);
    ht.fr_LF_upperleg_X_fr_LF_lowerleg(q);
    ht.fr_RF_hipassembly_X_fr_RF_upperleg(q);
    ht.fr_RF_upperleg_X_fr_RF_lowerleg(q);
    ht.fr_LH_hipassembly_X_fr_LH_upperleg(q);
    ht.fr_LH_upperleg_X_fr_LH_lowerleg(q);
    ht.fr_RH_hipassembly_X_fr_RH_upperleg(q);
    ht.fr_RH_upperleg_X_fr_RH_lowerleg(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
