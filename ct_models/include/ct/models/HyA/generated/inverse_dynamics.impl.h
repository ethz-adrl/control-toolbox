
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    Shoulder_AA_I(inertiaProps->getTensor_Shoulder_AA() ),
    Shoulder_FE_I(inertiaProps->getTensor_Shoulder_FE() ),
    Humerus_R_I(inertiaProps->getTensor_Humerus_R() ),
    Elbow_FE_I(inertiaProps->getTensor_Elbow_FE() ),
    Wrist_R_I(inertiaProps->getTensor_Wrist_R() ),
    Wrist_FE_I(inertiaProps->getTensor_Wrist_FE() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ct_HyA, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    Shoulder_AA_v.setZero();
    Shoulder_FE_v.setZero();
    Humerus_R_v.setZero();
    Elbow_FE_v.setZero();
    Wrist_R_v.setZero();
    Wrist_FE_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'Shoulder_AA'
    Shoulder_AA_a = (xm->fr_Shoulder_AA_X_fr_HyABase).col(iit::rbd::LZ) * SCALAR(iit::rbd::g);
    Shoulder_AA_f = Shoulder_AA_I * Shoulder_AA_a;
    // Link 'Shoulder_FE'
    Shoulder_FE_a = (xm->fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_a;
    Shoulder_FE_f = Shoulder_FE_I * Shoulder_FE_a;
    // Link 'Humerus_R'
    Humerus_R_a = (xm->fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_a;
    Humerus_R_f = Humerus_R_I * Humerus_R_a;
    // Link 'Elbow_FE'
    Elbow_FE_a = (xm->fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_a;
    Elbow_FE_f = Elbow_FE_I * Elbow_FE_a;
    // Link 'Wrist_R'
    Wrist_R_a = (xm->fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_a;
    Wrist_R_f = Wrist_R_I * Wrist_R_a;
    // Link 'Wrist_FE'
    Wrist_FE_a = (xm->fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_a;
    Wrist_FE_f = Wrist_FE_I * Wrist_FE_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'Shoulder_AA'
    Shoulder_AA_v(iit::rbd::AZ) = qd(SAA);   // Shoulder_AA_v = vJ, for the first link of a fixed base robot
    
    Shoulder_AA_f = iit::rbd::vxIv(qd(SAA), Shoulder_AA_I);
    
    // Link 'Shoulder_FE'
    Shoulder_FE_v = ((xm->fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_v);
    Shoulder_FE_v(iit::rbd::AZ) += qd(SFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Shoulder_FE_v, vcross);
    
    Shoulder_FE_a = (vcross.col(iit::rbd::AZ) * qd(SFE));
    
    Shoulder_FE_f = Shoulder_FE_I * Shoulder_FE_a + iit::rbd::vxIv(Shoulder_FE_v, Shoulder_FE_I);
    
    // Link 'Humerus_R'
    Humerus_R_v = ((xm->fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_v);
    Humerus_R_v(iit::rbd::AZ) += qd(HR);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Humerus_R_v, vcross);
    
    Humerus_R_a = (xm->fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_a + vcross.col(iit::rbd::AZ) * qd(HR);
    
    Humerus_R_f = Humerus_R_I * Humerus_R_a + iit::rbd::vxIv(Humerus_R_v, Humerus_R_I);
    
    // Link 'Elbow_FE'
    Elbow_FE_v = ((xm->fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_v);
    Elbow_FE_v(iit::rbd::AZ) += qd(EFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Elbow_FE_v, vcross);
    
    Elbow_FE_a = (xm->fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_a + vcross.col(iit::rbd::AZ) * qd(EFE);
    
    Elbow_FE_f = Elbow_FE_I * Elbow_FE_a + iit::rbd::vxIv(Elbow_FE_v, Elbow_FE_I);
    
    // Link 'Wrist_R'
    Wrist_R_v = ((xm->fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_v);
    Wrist_R_v(iit::rbd::AZ) += qd(WR);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_R_v, vcross);
    
    Wrist_R_a = (xm->fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_a + vcross.col(iit::rbd::AZ) * qd(WR);
    
    Wrist_R_f = Wrist_R_I * Wrist_R_a + iit::rbd::vxIv(Wrist_R_v, Wrist_R_I);
    
    // Link 'Wrist_FE'
    Wrist_FE_v = ((xm->fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_v);
    Wrist_FE_v(iit::rbd::AZ) += qd(WFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_FE_v, vcross);
    
    Wrist_FE_a = (xm->fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_a + vcross.col(iit::rbd::AZ) * qd(WFE);
    
    Wrist_FE_f = Wrist_FE_I * Wrist_FE_a + iit::rbd::vxIv(Wrist_FE_v, Wrist_FE_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'Shoulder_AA'
    Shoulder_AA_a = (xm->fr_Shoulder_AA_X_fr_HyABase).col(iit::rbd::LZ) * SCALAR(iit::rbd::g);
    Shoulder_AA_a(iit::rbd::AZ) += qdd(SAA);
    Shoulder_AA_v(iit::rbd::AZ) = qd(SAA);   // Shoulder_AA_v = vJ, for the first link of a fixed base robot
    
    Shoulder_AA_f = Shoulder_AA_I * Shoulder_AA_a + iit::rbd::vxIv(qd(SAA), Shoulder_AA_I)  - fext[SHOULDER_AA];
    
    // First pass, link 'Shoulder_FE'
    Shoulder_FE_v = ((xm->fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_v);
    Shoulder_FE_v(iit::rbd::AZ) += qd(SFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Shoulder_FE_v, vcross);
    
    Shoulder_FE_a = (xm->fr_Shoulder_FE_X_fr_Shoulder_AA) * Shoulder_AA_a + vcross.col(iit::rbd::AZ) * qd(SFE);
    Shoulder_FE_a(iit::rbd::AZ) += qdd(SFE);
    
    Shoulder_FE_f = Shoulder_FE_I * Shoulder_FE_a + iit::rbd::vxIv(Shoulder_FE_v, Shoulder_FE_I) - fext[SHOULDER_FE];
    
    // First pass, link 'Humerus_R'
    Humerus_R_v = ((xm->fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_v);
    Humerus_R_v(iit::rbd::AZ) += qd(HR);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Humerus_R_v, vcross);
    
    Humerus_R_a = (xm->fr_Humerus_R_X_fr_Shoulder_FE) * Shoulder_FE_a + vcross.col(iit::rbd::AZ) * qd(HR);
    Humerus_R_a(iit::rbd::AZ) += qdd(HR);
    
    Humerus_R_f = Humerus_R_I * Humerus_R_a + iit::rbd::vxIv(Humerus_R_v, Humerus_R_I) - fext[HUMERUS_R];
    
    // First pass, link 'Elbow_FE'
    Elbow_FE_v = ((xm->fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_v);
    Elbow_FE_v(iit::rbd::AZ) += qd(EFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Elbow_FE_v, vcross);
    
    Elbow_FE_a = (xm->fr_Elbow_FE_X_fr_Humerus_R) * Humerus_R_a + vcross.col(iit::rbd::AZ) * qd(EFE);
    Elbow_FE_a(iit::rbd::AZ) += qdd(EFE);
    
    Elbow_FE_f = Elbow_FE_I * Elbow_FE_a + iit::rbd::vxIv(Elbow_FE_v, Elbow_FE_I) - fext[ELBOW_FE];
    
    // First pass, link 'Wrist_R'
    Wrist_R_v = ((xm->fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_v);
    Wrist_R_v(iit::rbd::AZ) += qd(WR);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_R_v, vcross);
    
    Wrist_R_a = (xm->fr_Wrist_R_X_fr_Elbow_FE) * Elbow_FE_a + vcross.col(iit::rbd::AZ) * qd(WR);
    Wrist_R_a(iit::rbd::AZ) += qdd(WR);
    
    Wrist_R_f = Wrist_R_I * Wrist_R_a + iit::rbd::vxIv(Wrist_R_v, Wrist_R_I) - fext[WRIST_R];
    
    // First pass, link 'Wrist_FE'
    Wrist_FE_v = ((xm->fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_v);
    Wrist_FE_v(iit::rbd::AZ) += qd(WFE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(Wrist_FE_v, vcross);
    
    Wrist_FE_a = (xm->fr_Wrist_FE_X_fr_Wrist_R) * Wrist_R_a + vcross.col(iit::rbd::AZ) * qd(WFE);
    Wrist_FE_a(iit::rbd::AZ) += qdd(WFE);
    
    Wrist_FE_f = Wrist_FE_I * Wrist_FE_a + iit::rbd::vxIv(Wrist_FE_v, Wrist_FE_I) - fext[WRIST_FE];
    
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'Wrist_FE'
    jForces(WFE) = Wrist_FE_f(iit::rbd::AZ);
    Wrist_R_f += xm->fr_Wrist_FE_X_fr_Wrist_R.transpose() * Wrist_FE_f;
    // Link 'Wrist_R'
    jForces(WR) = Wrist_R_f(iit::rbd::AZ);
    Elbow_FE_f += xm->fr_Wrist_R_X_fr_Elbow_FE.transpose() * Wrist_R_f;
    // Link 'Elbow_FE'
    jForces(EFE) = Elbow_FE_f(iit::rbd::AZ);
    Humerus_R_f += xm->fr_Elbow_FE_X_fr_Humerus_R.transpose() * Elbow_FE_f;
    // Link 'Humerus_R'
    jForces(HR) = Humerus_R_f(iit::rbd::AZ);
    Shoulder_FE_f += xm->fr_Humerus_R_X_fr_Shoulder_FE.transpose() * Humerus_R_f;
    // Link 'Shoulder_FE'
    jForces(SFE) = Shoulder_FE_f(iit::rbd::AZ);
    Shoulder_AA_f += xm->fr_Shoulder_FE_X_fr_Shoulder_AA.transpose() * Shoulder_FE_f;
    // Link 'Shoulder_AA'
    jForces(SAA) = Shoulder_AA_f(iit::rbd::AZ);
}
