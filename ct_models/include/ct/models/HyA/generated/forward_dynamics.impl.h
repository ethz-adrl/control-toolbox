

// Initialization of static-const data
template<typename TRAIT>
const typename iit::ct_HyA::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ct_HyA::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template<typename TRAIT>
iit::ct_HyA::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ct_HyA::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    Shoulder_AA_v.setZero();
    Shoulder_AA_c.setZero();
    Shoulder_FE_v.setZero();
    Shoulder_FE_c.setZero();
    Humerus_R_v.setZero();
    Humerus_R_c.setZero();
    Elbow_FE_v.setZero();
    Elbow_FE_c.setZero();
    Wrist_R_v.setZero();
    Wrist_R_c.setZero();
    Wrist_FE_v.setZero();
    Wrist_FE_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    Shoulder_AA_AI = inertiaProps->getTensor_Shoulder_AA();
    Shoulder_AA_p = - fext[SHOULDER_AA];
    Shoulder_FE_AI = inertiaProps->getTensor_Shoulder_FE();
    Shoulder_FE_p = - fext[SHOULDER_FE];
    Humerus_R_AI = inertiaProps->getTensor_Humerus_R();
    Humerus_R_p = - fext[HUMERUS_R];
    Elbow_FE_AI = inertiaProps->getTensor_Elbow_FE();
    Elbow_FE_p = - fext[ELBOW_FE];
    Wrist_R_AI = inertiaProps->getTensor_Wrist_R();
    Wrist_R_p = - fext[WRIST_R];
    Wrist_FE_AI = inertiaProps->getTensor_Wrist_FE();
    Wrist_FE_p = - fext[WRIST_FE];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link Shoulder_AA
    //  - The spatial velocity:
    Shoulder_AA_v(rbd::AZ) = qd(SAA);
    
    //  - The bias force term:
    Shoulder_AA_p += iit::rbd::vxIv(qd(SAA), Shoulder_AA_AI);
    
    // + Link Shoulder_FE
    //  - The spatial velocity:
    Shoulder_FE_v = (motionTransforms-> fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_v;
    Shoulder_FE_v(rbd::AZ) += qd(SFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(Shoulder_FE_v, vcross);
    Shoulder_FE_c = vcross.col(rbd::AZ) * qd(SFE);
    
    //  - The bias force term:
    Shoulder_FE_p += iit::rbd::vxIv(Shoulder_FE_v, Shoulder_FE_AI);
    
    // + Link Humerus_R
    //  - The spatial velocity:
    Humerus_R_v = (motionTransforms-> fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_v;
    Humerus_R_v(rbd::AZ) += qd(HR);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(Humerus_R_v, vcross);
    Humerus_R_c = vcross.col(rbd::AZ) * qd(HR);
    
    //  - The bias force term:
    Humerus_R_p += iit::rbd::vxIv(Humerus_R_v, Humerus_R_AI);
    
    // + Link Elbow_FE
    //  - The spatial velocity:
    Elbow_FE_v = (motionTransforms-> fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_v;
    Elbow_FE_v(rbd::AZ) += qd(EFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(Elbow_FE_v, vcross);
    Elbow_FE_c = vcross.col(rbd::AZ) * qd(EFE);
    
    //  - The bias force term:
    Elbow_FE_p += iit::rbd::vxIv(Elbow_FE_v, Elbow_FE_AI);
    
    // + Link Wrist_R
    //  - The spatial velocity:
    Wrist_R_v = (motionTransforms-> fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_v;
    Wrist_R_v(rbd::AZ) += qd(WR);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_R_v, vcross);
    Wrist_R_c = vcross.col(rbd::AZ) * qd(WR);
    
    //  - The bias force term:
    Wrist_R_p += iit::rbd::vxIv(Wrist_R_v, Wrist_R_AI);
    
    // + Link Wrist_FE
    //  - The spatial velocity:
    Wrist_FE_v = (motionTransforms-> fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_v;
    Wrist_FE_v(rbd::AZ) += qd(WFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_FE_v, vcross);
    Wrist_FE_c = vcross.col(rbd::AZ) * qd(WFE);
    
    //  - The bias force term:
    Wrist_FE_p += iit::rbd::vxIv(Wrist_FE_v, Wrist_FE_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link Wrist_FE
    Wrist_FE_u = tau(WFE) - Wrist_FE_p(rbd::AZ);
    Wrist_FE_U = Wrist_FE_AI.col(rbd::AZ);
    Wrist_FE_D = Wrist_FE_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Wrist_FE_AI, Wrist_FE_U, Wrist_FE_D, Ia_r);  // same as: Ia_r = Wrist_FE_AI - Wrist_FE_U/Wrist_FE_D * Wrist_FE_U.transpose();
    pa = Wrist_FE_p + Ia_r * Wrist_FE_c + Wrist_FE_U * Wrist_FE_u/Wrist_FE_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Wrist_FE_X_fr_Wrist_R, IaB);
    Wrist_R_AI += IaB;
    Wrist_R_p += (motionTransforms-> fr_Wrist_FE_X_fr_Wrist_R).transpose() * pa;
    
    // + Link Wrist_R
    Wrist_R_u = tau(WR) - Wrist_R_p(rbd::AZ);
    Wrist_R_U = Wrist_R_AI.col(rbd::AZ);
    Wrist_R_D = Wrist_R_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Wrist_R_AI, Wrist_R_U, Wrist_R_D, Ia_r);  // same as: Ia_r = Wrist_R_AI - Wrist_R_U/Wrist_R_D * Wrist_R_U.transpose();
    pa = Wrist_R_p + Ia_r * Wrist_R_c + Wrist_R_U * Wrist_R_u/Wrist_R_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Wrist_R_X_fr_Elbow_FE, IaB);
    Elbow_FE_AI += IaB;
    Elbow_FE_p += (motionTransforms-> fr_Wrist_R_X_fr_Elbow_FE).transpose() * pa;
    
    // + Link Elbow_FE
    Elbow_FE_u = tau(EFE) - Elbow_FE_p(rbd::AZ);
    Elbow_FE_U = Elbow_FE_AI.col(rbd::AZ);
    Elbow_FE_D = Elbow_FE_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Elbow_FE_AI, Elbow_FE_U, Elbow_FE_D, Ia_r);  // same as: Ia_r = Elbow_FE_AI - Elbow_FE_U/Elbow_FE_D * Elbow_FE_U.transpose();
    pa = Elbow_FE_p + Ia_r * Elbow_FE_c + Elbow_FE_U * Elbow_FE_u/Elbow_FE_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Elbow_FE_X_fr_Humerus_R, IaB);
    Humerus_R_AI += IaB;
    Humerus_R_p += (motionTransforms-> fr_Elbow_FE_X_fr_Humerus_R).transpose() * pa;
    
    // + Link Humerus_R
    Humerus_R_u = tau(HR) - Humerus_R_p(rbd::AZ);
    Humerus_R_U = Humerus_R_AI.col(rbd::AZ);
    Humerus_R_D = Humerus_R_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Humerus_R_AI, Humerus_R_U, Humerus_R_D, Ia_r);  // same as: Ia_r = Humerus_R_AI - Humerus_R_U/Humerus_R_D * Humerus_R_U.transpose();
    pa = Humerus_R_p + Ia_r * Humerus_R_c + Humerus_R_U * Humerus_R_u/Humerus_R_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Humerus_R_X_fr_Shoulder_FE, IaB);
    Shoulder_FE_AI += IaB;
    Shoulder_FE_p += (motionTransforms-> fr_Humerus_R_X_fr_Shoulder_FE).transpose() * pa;
    
    // + Link Shoulder_FE
    Shoulder_FE_u = tau(SFE) - Shoulder_FE_p(rbd::AZ);
    Shoulder_FE_U = Shoulder_FE_AI.col(rbd::AZ);
    Shoulder_FE_D = Shoulder_FE_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Shoulder_FE_AI, Shoulder_FE_U, Shoulder_FE_D, Ia_r);  // same as: Ia_r = Shoulder_FE_AI - Shoulder_FE_U/Shoulder_FE_D * Shoulder_FE_U.transpose();
    pa = Shoulder_FE_p + Ia_r * Shoulder_FE_c + Shoulder_FE_U * Shoulder_FE_u/Shoulder_FE_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Shoulder_FE_X_fr_Shoulder_AA, IaB);
    Shoulder_AA_AI += IaB;
    Shoulder_AA_p += (motionTransforms-> fr_Shoulder_FE_X_fr_Shoulder_AA).transpose() * pa;
    
    // + Link Shoulder_AA
    Shoulder_AA_u = tau(SAA) - Shoulder_AA_p(rbd::AZ);
    Shoulder_AA_U = Shoulder_AA_AI.col(rbd::AZ);
    Shoulder_AA_D = Shoulder_AA_U(rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    Shoulder_AA_a = (motionTransforms-> fr_Shoulder_AA_X_fr_HyABase).col(rbd::LZ) * SCALAR(iit::rbd::g);
    qdd(SAA) = (Shoulder_AA_u - Shoulder_AA_U.dot(Shoulder_AA_a)) / Shoulder_AA_D;
    Shoulder_AA_a(rbd::AZ) += qdd(SAA);
    
    Shoulder_FE_a = (motionTransforms-> fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_a + Shoulder_FE_c;
    qdd(SFE) = (Shoulder_FE_u - Shoulder_FE_U.dot(Shoulder_FE_a)) / Shoulder_FE_D;
    Shoulder_FE_a(rbd::AZ) += qdd(SFE);
    
    Humerus_R_a = (motionTransforms-> fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_a + Humerus_R_c;
    qdd(HR) = (Humerus_R_u - Humerus_R_U.dot(Humerus_R_a)) / Humerus_R_D;
    Humerus_R_a(rbd::AZ) += qdd(HR);
    
    Elbow_FE_a = (motionTransforms-> fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_a + Elbow_FE_c;
    qdd(EFE) = (Elbow_FE_u - Elbow_FE_U.dot(Elbow_FE_a)) / Elbow_FE_D;
    Elbow_FE_a(rbd::AZ) += qdd(EFE);
    
    Wrist_R_a = (motionTransforms-> fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_a + Wrist_R_c;
    qdd(WR) = (Wrist_R_u - Wrist_R_U.dot(Wrist_R_a)) / Wrist_R_D;
    Wrist_R_a(rbd::AZ) += qdd(WR);
    
    Wrist_FE_a = (motionTransforms-> fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_a + Wrist_FE_c;
    qdd(WFE) = (Wrist_FE_u - Wrist_FE_U.dot(Wrist_FE_a)) / Wrist_FE_D;
    Wrist_FE_a(rbd::AZ) += qdd(WFE);
    
    
}
