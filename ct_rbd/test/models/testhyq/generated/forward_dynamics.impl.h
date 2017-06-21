
// Initialization of static-const data
template <typename TRAIT>
const typename iit::TestHyQ::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::TestHyQ::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::TestHyQ::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::TestHyQ::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    LF_hipassembly_v.setZero();
    LF_hipassembly_c.setZero();
    LF_upperleg_v.setZero();
    LF_upperleg_c.setZero();
    LF_lowerleg_v.setZero();
    LF_lowerleg_c.setZero();
    RF_hipassembly_v.setZero();
    RF_hipassembly_c.setZero();
    RF_upperleg_v.setZero();
    RF_upperleg_c.setZero();
    RF_lowerleg_v.setZero();
    RF_lowerleg_c.setZero();
    LH_hipassembly_v.setZero();
    LH_hipassembly_c.setZero();
    LH_upperleg_v.setZero();
    LH_upperleg_c.setZero();
    LH_lowerleg_v.setZero();
    LH_lowerleg_c.setZero();
    RH_hipassembly_v.setZero();
    RH_hipassembly_c.setZero();
    RH_upperleg_v.setZero();
    RH_upperleg_c.setZero();
    RH_lowerleg_v.setZero();
    RH_lowerleg_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::TestHyQ::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& trunk_a,
    const Velocity& trunk_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    trunk_AI = inertiaProps->getTensor_trunk();
    trunk_p = - fext[TRUNK];
    LF_hipassembly_AI = inertiaProps->getTensor_LF_hipassembly();
    LF_hipassembly_p = - fext[LF_HIPASSEMBLY];
    LF_upperleg_AI = inertiaProps->getTensor_LF_upperleg();
    LF_upperleg_p = - fext[LF_UPPERLEG];
    LF_lowerleg_AI = inertiaProps->getTensor_LF_lowerleg();
    LF_lowerleg_p = - fext[LF_LOWERLEG];
    RF_hipassembly_AI = inertiaProps->getTensor_RF_hipassembly();
    RF_hipassembly_p = - fext[RF_HIPASSEMBLY];
    RF_upperleg_AI = inertiaProps->getTensor_RF_upperleg();
    RF_upperleg_p = - fext[RF_UPPERLEG];
    RF_lowerleg_AI = inertiaProps->getTensor_RF_lowerleg();
    RF_lowerleg_p = - fext[RF_LOWERLEG];
    LH_hipassembly_AI = inertiaProps->getTensor_LH_hipassembly();
    LH_hipassembly_p = - fext[LH_HIPASSEMBLY];
    LH_upperleg_AI = inertiaProps->getTensor_LH_upperleg();
    LH_upperleg_p = - fext[LH_UPPERLEG];
    LH_lowerleg_AI = inertiaProps->getTensor_LH_lowerleg();
    LH_lowerleg_p = - fext[LH_LOWERLEG];
    RH_hipassembly_AI = inertiaProps->getTensor_RH_hipassembly();
    RH_hipassembly_p = - fext[RH_HIPASSEMBLY];
    RH_upperleg_AI = inertiaProps->getTensor_RH_upperleg();
    RH_upperleg_p = - fext[RH_UPPERLEG];
    RH_lowerleg_AI = inertiaProps->getTensor_RH_lowerleg();
    RH_lowerleg_p = - fext[RH_LOWERLEG];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link LF_hipassembly
    //  - The spatial velocity:
    LF_hipassembly_v = (motionTransforms-> fr_LF_hipassembly_X_fr_trunk) * trunk_v;
    LF_hipassembly_v(iit::rbd::AZ) += qd(LF_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_hipassembly_v, vcross);
    LF_hipassembly_c = vcross.col(iit::rbd::AZ) * qd(LF_HAA);
    
    //  - The bias force term:
    LF_hipassembly_p += iit::rbd::vxIv(LF_hipassembly_v, LF_hipassembly_AI);
    
    // + Link LF_upperleg
    //  - The spatial velocity:
    LF_upperleg_v = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_v;
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_upperleg_v, vcross);
    LF_upperleg_c = vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    
    //  - The bias force term:
    LF_upperleg_p += iit::rbd::vxIv(LF_upperleg_v, LF_upperleg_AI);
    
    // + Link LF_lowerleg
    //  - The spatial velocity:
    LF_lowerleg_v = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v;
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_lowerleg_v, vcross);
    LF_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    
    //  - The bias force term:
    LF_lowerleg_p += iit::rbd::vxIv(LF_lowerleg_v, LF_lowerleg_AI);
    
    // + Link RF_hipassembly
    //  - The spatial velocity:
    RF_hipassembly_v = (motionTransforms-> fr_RF_hipassembly_X_fr_trunk) * trunk_v;
    RF_hipassembly_v(iit::rbd::AZ) += qd(RF_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_hipassembly_v, vcross);
    RF_hipassembly_c = vcross.col(iit::rbd::AZ) * qd(RF_HAA);
    
    //  - The bias force term:
    RF_hipassembly_p += iit::rbd::vxIv(RF_hipassembly_v, RF_hipassembly_AI);
    
    // + Link RF_upperleg
    //  - The spatial velocity:
    RF_upperleg_v = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_v;
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_upperleg_v, vcross);
    RF_upperleg_c = vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    
    //  - The bias force term:
    RF_upperleg_p += iit::rbd::vxIv(RF_upperleg_v, RF_upperleg_AI);
    
    // + Link RF_lowerleg
    //  - The spatial velocity:
    RF_lowerleg_v = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v;
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_lowerleg_v, vcross);
    RF_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    
    //  - The bias force term:
    RF_lowerleg_p += iit::rbd::vxIv(RF_lowerleg_v, RF_lowerleg_AI);
    
    // + Link LH_hipassembly
    //  - The spatial velocity:
    LH_hipassembly_v = (motionTransforms-> fr_LH_hipassembly_X_fr_trunk) * trunk_v;
    LH_hipassembly_v(iit::rbd::AZ) += qd(LH_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_hipassembly_v, vcross);
    LH_hipassembly_c = vcross.col(iit::rbd::AZ) * qd(LH_HAA);
    
    //  - The bias force term:
    LH_hipassembly_p += iit::rbd::vxIv(LH_hipassembly_v, LH_hipassembly_AI);
    
    // + Link LH_upperleg
    //  - The spatial velocity:
    LH_upperleg_v = (motionTransforms-> fr_LH_upperleg_X_fr_LH_hipassembly) * LH_hipassembly_v;
    LH_upperleg_v(iit::rbd::AZ) += qd(LH_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_upperleg_v, vcross);
    LH_upperleg_c = vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    
    //  - The bias force term:
    LH_upperleg_p += iit::rbd::vxIv(LH_upperleg_v, LH_upperleg_AI);
    
    // + Link LH_lowerleg
    //  - The spatial velocity:
    LH_lowerleg_v = (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_v;
    LH_lowerleg_v(iit::rbd::AZ) += qd(LH_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_lowerleg_v, vcross);
    LH_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    
    //  - The bias force term:
    LH_lowerleg_p += iit::rbd::vxIv(LH_lowerleg_v, LH_lowerleg_AI);
    
    // + Link RH_hipassembly
    //  - The spatial velocity:
    RH_hipassembly_v = (motionTransforms-> fr_RH_hipassembly_X_fr_trunk) * trunk_v;
    RH_hipassembly_v(iit::rbd::AZ) += qd(RH_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_hipassembly_v, vcross);
    RH_hipassembly_c = vcross.col(iit::rbd::AZ) * qd(RH_HAA);
    
    //  - The bias force term:
    RH_hipassembly_p += iit::rbd::vxIv(RH_hipassembly_v, RH_hipassembly_AI);
    
    // + Link RH_upperleg
    //  - The spatial velocity:
    RH_upperleg_v = (motionTransforms-> fr_RH_upperleg_X_fr_RH_hipassembly) * RH_hipassembly_v;
    RH_upperleg_v(iit::rbd::AZ) += qd(RH_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_upperleg_v, vcross);
    RH_upperleg_c = vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    
    //  - The bias force term:
    RH_upperleg_p += iit::rbd::vxIv(RH_upperleg_v, RH_upperleg_AI);
    
    // + Link RH_lowerleg
    //  - The spatial velocity:
    RH_lowerleg_v = (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_v;
    RH_lowerleg_v(iit::rbd::AZ) += qd(RH_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_lowerleg_v, vcross);
    RH_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    
    //  - The bias force term:
    RH_lowerleg_p += iit::rbd::vxIv(RH_lowerleg_v, RH_lowerleg_AI);
    
    // + The floating base body
    trunk_p += iit::rbd::vxIv(trunk_v, trunk_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link RH_lowerleg
    RH_lowerleg_u = tau(RH_KFE) - RH_lowerleg_p(iit::rbd::AZ);
    RH_lowerleg_U = RH_lowerleg_AI.col(iit::rbd::AZ);
    RH_lowerleg_D = RH_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_lowerleg_AI, RH_lowerleg_U, RH_lowerleg_D, Ia_r);  // same as: Ia_r = RH_lowerleg_AI - RH_lowerleg_U/RH_lowerleg_D * RH_lowerleg_U.transpose();
    pa = RH_lowerleg_p + Ia_r * RH_lowerleg_c + RH_lowerleg_U * RH_lowerleg_u/RH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg, IaB);
    RH_upperleg_AI += IaB;
    RH_upperleg_p += (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg).transpose() * pa;
    
    // + Link RH_upperleg
    RH_upperleg_u = tau(RH_HFE) - RH_upperleg_p(iit::rbd::AZ);
    RH_upperleg_U = RH_upperleg_AI.col(iit::rbd::AZ);
    RH_upperleg_D = RH_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_upperleg_AI, RH_upperleg_U, RH_upperleg_D, Ia_r);  // same as: Ia_r = RH_upperleg_AI - RH_upperleg_U/RH_upperleg_D * RH_upperleg_U.transpose();
    pa = RH_upperleg_p + Ia_r * RH_upperleg_c + RH_upperleg_U * RH_upperleg_u/RH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_upperleg_X_fr_RH_hipassembly, IaB);
    RH_hipassembly_AI += IaB;
    RH_hipassembly_p += (motionTransforms-> fr_RH_upperleg_X_fr_RH_hipassembly).transpose() * pa;
    
    // + Link RH_hipassembly
    RH_hipassembly_u = tau(RH_HAA) - RH_hipassembly_p(iit::rbd::AZ);
    RH_hipassembly_U = RH_hipassembly_AI.col(iit::rbd::AZ);
    RH_hipassembly_D = RH_hipassembly_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_hipassembly_AI, RH_hipassembly_U, RH_hipassembly_D, Ia_r);  // same as: Ia_r = RH_hipassembly_AI - RH_hipassembly_U/RH_hipassembly_D * RH_hipassembly_U.transpose();
    pa = RH_hipassembly_p + Ia_r * RH_hipassembly_c + RH_hipassembly_U * RH_hipassembly_u/RH_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_RH_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link LH_lowerleg
    LH_lowerleg_u = tau(LH_KFE) - LH_lowerleg_p(iit::rbd::AZ);
    LH_lowerleg_U = LH_lowerleg_AI.col(iit::rbd::AZ);
    LH_lowerleg_D = LH_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_lowerleg_AI, LH_lowerleg_U, LH_lowerleg_D, Ia_r);  // same as: Ia_r = LH_lowerleg_AI - LH_lowerleg_U/LH_lowerleg_D * LH_lowerleg_U.transpose();
    pa = LH_lowerleg_p + Ia_r * LH_lowerleg_c + LH_lowerleg_U * LH_lowerleg_u/LH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg, IaB);
    LH_upperleg_AI += IaB;
    LH_upperleg_p += (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg).transpose() * pa;
    
    // + Link LH_upperleg
    LH_upperleg_u = tau(LH_HFE) - LH_upperleg_p(iit::rbd::AZ);
    LH_upperleg_U = LH_upperleg_AI.col(iit::rbd::AZ);
    LH_upperleg_D = LH_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_upperleg_AI, LH_upperleg_U, LH_upperleg_D, Ia_r);  // same as: Ia_r = LH_upperleg_AI - LH_upperleg_U/LH_upperleg_D * LH_upperleg_U.transpose();
    pa = LH_upperleg_p + Ia_r * LH_upperleg_c + LH_upperleg_U * LH_upperleg_u/LH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_upperleg_X_fr_LH_hipassembly, IaB);
    LH_hipassembly_AI += IaB;
    LH_hipassembly_p += (motionTransforms-> fr_LH_upperleg_X_fr_LH_hipassembly).transpose() * pa;
    
    // + Link LH_hipassembly
    LH_hipassembly_u = tau(LH_HAA) - LH_hipassembly_p(iit::rbd::AZ);
    LH_hipassembly_U = LH_hipassembly_AI.col(iit::rbd::AZ);
    LH_hipassembly_D = LH_hipassembly_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_hipassembly_AI, LH_hipassembly_U, LH_hipassembly_D, Ia_r);  // same as: Ia_r = LH_hipassembly_AI - LH_hipassembly_U/LH_hipassembly_D * LH_hipassembly_U.transpose();
    pa = LH_hipassembly_p + Ia_r * LH_hipassembly_c + LH_hipassembly_U * LH_hipassembly_u/LH_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_LH_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link RF_lowerleg
    RF_lowerleg_u = tau(RF_KFE) - RF_lowerleg_p(iit::rbd::AZ);
    RF_lowerleg_U = RF_lowerleg_AI.col(iit::rbd::AZ);
    RF_lowerleg_D = RF_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_lowerleg_AI, RF_lowerleg_U, RF_lowerleg_D, Ia_r);  // same as: Ia_r = RF_lowerleg_AI - RF_lowerleg_U/RF_lowerleg_D * RF_lowerleg_U.transpose();
    pa = RF_lowerleg_p + Ia_r * RF_lowerleg_c + RF_lowerleg_U * RF_lowerleg_u/RF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg, IaB);
    RF_upperleg_AI += IaB;
    RF_upperleg_p += (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * pa;
    
    // + Link RF_upperleg
    RF_upperleg_u = tau(RF_HFE) - RF_upperleg_p(iit::rbd::AZ);
    RF_upperleg_U = RF_upperleg_AI.col(iit::rbd::AZ);
    RF_upperleg_D = RF_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_upperleg_AI, RF_upperleg_U, RF_upperleg_D, Ia_r);  // same as: Ia_r = RF_upperleg_AI - RF_upperleg_U/RF_upperleg_D * RF_upperleg_U.transpose();
    pa = RF_upperleg_p + Ia_r * RF_upperleg_c + RF_upperleg_U * RF_upperleg_u/RF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly, IaB);
    RF_hipassembly_AI += IaB;
    RF_hipassembly_p += (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly).transpose() * pa;
    
    // + Link RF_hipassembly
    RF_hipassembly_u = tau(RF_HAA) - RF_hipassembly_p(iit::rbd::AZ);
    RF_hipassembly_U = RF_hipassembly_AI.col(iit::rbd::AZ);
    RF_hipassembly_D = RF_hipassembly_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_hipassembly_AI, RF_hipassembly_U, RF_hipassembly_D, Ia_r);  // same as: Ia_r = RF_hipassembly_AI - RF_hipassembly_U/RF_hipassembly_D * RF_hipassembly_U.transpose();
    pa = RF_hipassembly_p + Ia_r * RF_hipassembly_c + RF_hipassembly_U * RF_hipassembly_u/RF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_RF_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link LF_lowerleg
    LF_lowerleg_u = tau(LF_KFE) - LF_lowerleg_p(iit::rbd::AZ);
    LF_lowerleg_U = LF_lowerleg_AI.col(iit::rbd::AZ);
    LF_lowerleg_D = LF_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_lowerleg_AI, LF_lowerleg_U, LF_lowerleg_D, Ia_r);  // same as: Ia_r = LF_lowerleg_AI - LF_lowerleg_U/LF_lowerleg_D * LF_lowerleg_U.transpose();
    pa = LF_lowerleg_p + Ia_r * LF_lowerleg_c + LF_lowerleg_U * LF_lowerleg_u/LF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg, IaB);
    LF_upperleg_AI += IaB;
    LF_upperleg_p += (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * pa;
    
    // + Link LF_upperleg
    LF_upperleg_u = tau(LF_HFE) - LF_upperleg_p(iit::rbd::AZ);
    LF_upperleg_U = LF_upperleg_AI.col(iit::rbd::AZ);
    LF_upperleg_D = LF_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_upperleg_AI, LF_upperleg_U, LF_upperleg_D, Ia_r);  // same as: Ia_r = LF_upperleg_AI - LF_upperleg_U/LF_upperleg_D * LF_upperleg_U.transpose();
    pa = LF_upperleg_p + Ia_r * LF_upperleg_c + LF_upperleg_U * LF_upperleg_u/LF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly, IaB);
    LF_hipassembly_AI += IaB;
    LF_hipassembly_p += (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly).transpose() * pa;
    
    // + Link LF_hipassembly
    LF_hipassembly_u = tau(LF_HAA) - LF_hipassembly_p(iit::rbd::AZ);
    LF_hipassembly_U = LF_hipassembly_AI.col(iit::rbd::AZ);
    LF_hipassembly_D = LF_hipassembly_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_hipassembly_AI, LF_hipassembly_U, LF_hipassembly_D, Ia_r);  // same as: Ia_r = LF_hipassembly_AI - LF_hipassembly_U/LF_hipassembly_D * LF_hipassembly_U.transpose();
    pa = LF_hipassembly_p + Ia_r * LF_hipassembly_c + LF_hipassembly_U * LF_hipassembly_u/LF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_LF_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + The acceleration of the floating base trunk, without gravity
    trunk_a = - TRAIT::solve(trunk_AI, trunk_p);  // trunk_a = - IA^-1 * trunk_p
    
    // ---------------------- THIRD PASS ---------------------- //
    LF_hipassembly_a = (motionTransforms-> fr_LF_hipassembly_X_fr_trunk) * trunk_a + LF_hipassembly_c;
    qdd(LF_HAA) = (LF_hipassembly_u - LF_hipassembly_U.dot(LF_hipassembly_a)) / LF_hipassembly_D;
    LF_hipassembly_a(iit::rbd::AZ) += qdd(LF_HAA);
    
    LF_upperleg_a = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a + LF_upperleg_c;
    qdd(LF_HFE) = (LF_upperleg_u - LF_upperleg_U.dot(LF_upperleg_a)) / LF_upperleg_D;
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_HFE);
    
    LF_lowerleg_a = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + LF_lowerleg_c;
    qdd(LF_KFE) = (LF_lowerleg_u - LF_lowerleg_U.dot(LF_lowerleg_a)) / LF_lowerleg_D;
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    RF_hipassembly_a = (motionTransforms-> fr_RF_hipassembly_X_fr_trunk) * trunk_a + RF_hipassembly_c;
    qdd(RF_HAA) = (RF_hipassembly_u - RF_hipassembly_U.dot(RF_hipassembly_a)) / RF_hipassembly_D;
    RF_hipassembly_a(iit::rbd::AZ) += qdd(RF_HAA);
    
    RF_upperleg_a = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a + RF_upperleg_c;
    qdd(RF_HFE) = (RF_upperleg_u - RF_upperleg_U.dot(RF_upperleg_a)) / RF_upperleg_D;
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_HFE);
    
    RF_lowerleg_a = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + RF_lowerleg_c;
    qdd(RF_KFE) = (RF_lowerleg_u - RF_lowerleg_U.dot(RF_lowerleg_a)) / RF_lowerleg_D;
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    LH_hipassembly_a = (motionTransforms-> fr_LH_hipassembly_X_fr_trunk) * trunk_a + LH_hipassembly_c;
    qdd(LH_HAA) = (LH_hipassembly_u - LH_hipassembly_U.dot(LH_hipassembly_a)) / LH_hipassembly_D;
    LH_hipassembly_a(iit::rbd::AZ) += qdd(LH_HAA);
    
    LH_upperleg_a = (motionTransforms-> fr_LH_upperleg_X_fr_LH_hipassembly) * LH_hipassembly_a + LH_upperleg_c;
    qdd(LH_HFE) = (LH_upperleg_u - LH_upperleg_U.dot(LH_upperleg_a)) / LH_upperleg_D;
    LH_upperleg_a(iit::rbd::AZ) += qdd(LH_HFE);
    
    LH_lowerleg_a = (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a + LH_lowerleg_c;
    qdd(LH_KFE) = (LH_lowerleg_u - LH_lowerleg_U.dot(LH_lowerleg_a)) / LH_lowerleg_D;
    LH_lowerleg_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    RH_hipassembly_a = (motionTransforms-> fr_RH_hipassembly_X_fr_trunk) * trunk_a + RH_hipassembly_c;
    qdd(RH_HAA) = (RH_hipassembly_u - RH_hipassembly_U.dot(RH_hipassembly_a)) / RH_hipassembly_D;
    RH_hipassembly_a(iit::rbd::AZ) += qdd(RH_HAA);
    
    RH_upperleg_a = (motionTransforms-> fr_RH_upperleg_X_fr_RH_hipassembly) * RH_hipassembly_a + RH_upperleg_c;
    qdd(RH_HFE) = (RH_upperleg_u - RH_upperleg_U.dot(RH_upperleg_a)) / RH_upperleg_D;
    RH_upperleg_a(iit::rbd::AZ) += qdd(RH_HFE);
    
    RH_lowerleg_a = (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a + RH_lowerleg_c;
    qdd(RH_KFE) = (RH_lowerleg_u - RH_lowerleg_U.dot(RH_lowerleg_a)) / RH_lowerleg_D;
    RH_lowerleg_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    
    // + Add gravity to the acceleration of the floating base
    trunk_a += g;
}
