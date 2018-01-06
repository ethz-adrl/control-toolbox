// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    Link1_I(inertiaProps->getTensor_Link1() ),
    Link2_I(inertiaProps->getTensor_Link2() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ct_DoubleInvertedPendulum, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    Link1_v.setZero();
    Link2_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'Link1'
    Link1_a = (xm->fr_Link1_X_fr_DoubleInvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    Link1_f = Link1_I * Link1_a;
    // Link 'Link2'
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a;
    Link2_f = Link2_I * Link2_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'Link1'
    Link1_v(iit::rbd::AZ) = qd(JOINT1);   // Link1_v = vJ, for the first link of a fixed base robot
    
    Link1_f = iit::rbd::vxIv(qd(JOINT1), Link1_I);
    
    // Link 'Link2'
    Link2_v = ((xm->fr_Link2_X_fr_Link1) * Link1_v);
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(Link2_v, vcross);
    
    Link2_a = (vcross.col(iit::rbd::AZ) * qd(JOINT2));
    
    Link2_f = Link2_I * Link2_a + iit::rbd::vxIv(Link2_v, Link2_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'Link1'
    Link1_a = (xm->fr_Link1_X_fr_DoubleInvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    Link1_v(iit::rbd::AZ) = qd(JOINT1);   // Link1_v = vJ, for the first link of a fixed base robot
    
    Link1_f = Link1_I * Link1_a + iit::rbd::vxIv(qd(JOINT1), Link1_I)  - fext[LINK1];
    
    // First pass, link 'Link2'
    Link2_v = ((xm->fr_Link2_X_fr_Link1) * Link1_v);
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(Link2_v, vcross);
    
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    Link2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    Link2_f = Link2_I * Link2_a + iit::rbd::vxIv(Link2_v, Link2_I) - fext[LINK2];
    
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'Link2'
    jForces(JOINT2) = Link2_f(iit::rbd::AZ);
    Link1_f += xm->fr_Link2_X_fr_Link1.transpose() * Link2_f;
    // Link 'Link1'
    jForces(JOINT1) = Link1_f(iit::rbd::AZ);
}
