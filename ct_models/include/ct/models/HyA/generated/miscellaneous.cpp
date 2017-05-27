#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ct_HyA;
using namespace iit::ct_HyA::dyn;

iit::rbd::Vector3d iit::ct_HyA::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_HyABase_X_fr_Shoulder_AA;
    tmpSum += inertiaProps.getMass_Shoulder_AA() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Shoulder_AA()));
    
    tmpX = tmpX * ht.fr_Shoulder_AA_X_fr_Shoulder_FE;
    tmpSum += inertiaProps.getMass_Shoulder_FE() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Shoulder_FE()));
    
    tmpX = tmpX * ht.fr_Shoulder_FE_X_fr_Humerus_R;
    tmpSum += inertiaProps.getMass_Humerus_R() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Humerus_R()));
    
    tmpX = tmpX * ht.fr_Humerus_R_X_fr_Elbow_FE;
    tmpSum += inertiaProps.getMass_Elbow_FE() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Elbow_FE()));
    
    tmpX = tmpX * ht.fr_Elbow_FE_X_fr_Wrist_R;
    tmpSum += inertiaProps.getMass_Wrist_R() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Wrist_R()));
    
    tmpX = tmpX * ht.fr_Wrist_R_X_fr_Wrist_FE;
    tmpSum += inertiaProps.getMass_Wrist_FE() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Wrist_FE()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ct_HyA::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_HyABase_X_fr_Shoulder_AA(q);
    ht.fr_Shoulder_AA_X_fr_Shoulder_FE(q);
    ht.fr_Shoulder_FE_X_fr_Humerus_R(q);
    ht.fr_Humerus_R_X_fr_Elbow_FE(q);
    ht.fr_Elbow_FE_X_fr_Wrist_R(q);
    ht.fr_Wrist_R_X_fr_Wrist_FE(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
