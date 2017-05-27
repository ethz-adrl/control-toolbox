
// Initialization of static-const data
template <typename TRAIT>
const typename iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    link1_I(inertiaProps->getTensor_link1() ),
    link2_I(inertiaProps->getTensor_link2() ),
    link3_I(inertiaProps->getTensor_link3() ),
    link4_I(inertiaProps->getTensor_link4() ),
    link5_I(inertiaProps->getTensor_link5() ),
    link6_I(inertiaProps->getTensor_link6() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot testirb4600, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    link1_v.setZero();
    link2_v.setZero();
    link3_v.setZero();
    link4_v.setZero();
    link5_v.setZero();
    link6_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'link1'
    link1_a = (xm->fr_link1_X_fr_link0).col(iit::rbd::LZ) * SCALAR(iit::rbd::g);
    link1_f = link1_I * link1_a;
    // Link 'link2'
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a;
    link2_f = link2_I * link2_a;
    // Link 'link3'
    link3_a = (xm->fr_link3_X_fr_link2) * link2_a;
    link3_f = link3_I * link3_a;
    // Link 'link4'
    link4_a = (xm->fr_link4_X_fr_link3) * link3_a;
    link4_f = link4_I * link4_a;
    // Link 'link5'
    link5_a = (xm->fr_link5_X_fr_link4) * link4_a;
    link5_f = link5_I * link5_a;
    // Link 'link6'
    link6_a = (xm->fr_link6_X_fr_link5) * link5_a;
    link6_f = link6_I * link6_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'link1'
    link1_v(iit::rbd::AZ) = qd(JA);   // link1_v = vJ, for the first link of a fixed base robot
    
    link1_f = iit::rbd::vxIv(qd(JA), link1_I);
    
    // Link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::AZ) += qd(JB);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    
    link2_a = (vcross.col(iit::rbd::AZ) * qd(JB));
    
    link2_f = link2_I * link2_a + iit::rbd::vxIv(link2_v, link2_I);
    
    // Link 'link3'
    link3_v = ((xm->fr_link3_X_fr_link2) * link2_v);
    link3_v(iit::rbd::AZ) += qd(JC);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link3_v, vcross);
    
    link3_a = (xm->fr_link3_X_fr_link2) * link2_a + vcross.col(iit::rbd::AZ) * qd(JC);
    
    link3_f = link3_I * link3_a + iit::rbd::vxIv(link3_v, link3_I);
    
    // Link 'link4'
    link4_v = ((xm->fr_link4_X_fr_link3) * link3_v);
    link4_v(iit::rbd::AZ) += qd(JD);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link4_v, vcross);
    
    link4_a = (xm->fr_link4_X_fr_link3) * link3_a + vcross.col(iit::rbd::AZ) * qd(JD);
    
    link4_f = link4_I * link4_a + iit::rbd::vxIv(link4_v, link4_I);
    
    // Link 'link5'
    link5_v = ((xm->fr_link5_X_fr_link4) * link4_v);
    link5_v(iit::rbd::AZ) += qd(JE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link5_v, vcross);
    
    link5_a = (xm->fr_link5_X_fr_link4) * link4_a + vcross.col(iit::rbd::AZ) * qd(JE);
    
    link5_f = link5_I * link5_a + iit::rbd::vxIv(link5_v, link5_I);
    
    // Link 'link6'
    link6_v = ((xm->fr_link6_X_fr_link5) * link5_v);
    link6_v(iit::rbd::AZ) += qd(JF);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link6_v, vcross);
    
    link6_a = (xm->fr_link6_X_fr_link5) * link5_a + vcross.col(iit::rbd::AZ) * qd(JF);
    
    link6_f = link6_I * link6_a + iit::rbd::vxIv(link6_v, link6_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'link1'
    link1_a = (xm->fr_link1_X_fr_link0).col(iit::rbd::LZ) * SCALAR(iit::rbd::g);
    link1_a(iit::rbd::AZ) += qdd(JA);
    link1_v(iit::rbd::AZ) = qd(JA);   // link1_v = vJ, for the first link of a fixed base robot
    
    link1_f = link1_I * link1_a + iit::rbd::vxIv(qd(JA), link1_I)  - fext[LINK1];
    
    // First pass, link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::AZ) += qd(JB);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a + vcross.col(iit::rbd::AZ) * qd(JB);
    link2_a(iit::rbd::AZ) += qdd(JB);
    
    link2_f = link2_I * link2_a + iit::rbd::vxIv(link2_v, link2_I) - fext[LINK2];
    
    // First pass, link 'link3'
    link3_v = ((xm->fr_link3_X_fr_link2) * link2_v);
    link3_v(iit::rbd::AZ) += qd(JC);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link3_v, vcross);
    
    link3_a = (xm->fr_link3_X_fr_link2) * link2_a + vcross.col(iit::rbd::AZ) * qd(JC);
    link3_a(iit::rbd::AZ) += qdd(JC);
    
    link3_f = link3_I * link3_a + iit::rbd::vxIv(link3_v, link3_I) - fext[LINK3];
    
    // First pass, link 'link4'
    link4_v = ((xm->fr_link4_X_fr_link3) * link3_v);
    link4_v(iit::rbd::AZ) += qd(JD);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link4_v, vcross);
    
    link4_a = (xm->fr_link4_X_fr_link3) * link3_a + vcross.col(iit::rbd::AZ) * qd(JD);
    link4_a(iit::rbd::AZ) += qdd(JD);
    
    link4_f = link4_I * link4_a + iit::rbd::vxIv(link4_v, link4_I) - fext[LINK4];
    
    // First pass, link 'link5'
    link5_v = ((xm->fr_link5_X_fr_link4) * link4_v);
    link5_v(iit::rbd::AZ) += qd(JE);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link5_v, vcross);
    
    link5_a = (xm->fr_link5_X_fr_link4) * link4_a + vcross.col(iit::rbd::AZ) * qd(JE);
    link5_a(iit::rbd::AZ) += qdd(JE);
    
    link5_f = link5_I * link5_a + iit::rbd::vxIv(link5_v, link5_I) - fext[LINK5];
    
    // First pass, link 'link6'
    link6_v = ((xm->fr_link6_X_fr_link5) * link5_v);
    link6_v(iit::rbd::AZ) += qd(JF);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link6_v, vcross);
    
    link6_a = (xm->fr_link6_X_fr_link5) * link5_a + vcross.col(iit::rbd::AZ) * qd(JF);
    link6_a(iit::rbd::AZ) += qdd(JF);
    
    link6_f = link6_I * link6_a + iit::rbd::vxIv(link6_v, link6_I) - fext[LINK6];
    
}

template <typename TRAIT>
void iit::testirb4600::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'link6'
    jForces(JF) = link6_f(iit::rbd::AZ);
    link5_f += xm->fr_link6_X_fr_link5.transpose() * link6_f;
    // Link 'link5'
    jForces(JE) = link5_f(iit::rbd::AZ);
    link4_f += xm->fr_link5_X_fr_link4.transpose() * link5_f;
    // Link 'link4'
    jForces(JD) = link4_f(iit::rbd::AZ);
    link3_f += xm->fr_link4_X_fr_link3.transpose() * link4_f;
    // Link 'link3'
    jForces(JC) = link3_f(iit::rbd::AZ);
    link2_f += xm->fr_link3_X_fr_link2.transpose() * link3_f;
    // Link 'link2'
    jForces(JB) = link2_f(iit::rbd::AZ);
    link1_f += xm->fr_link2_X_fr_link1.transpose() * link2_f;
    // Link 'link1'
    jForces(JA) = link1_f(iit::rbd::AZ);
}
