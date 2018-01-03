// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    Link1_I(inertiaProps->getTensor_Link1() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ct_InvertedPendulum, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    Link1_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'Link1'
    Link1_a = (xm->fr_Link1_X_fr_InvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    Link1_f = Link1_I * Link1_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'Link1'
    Link1_v(iit::rbd::AZ) = qd(JOINT1);   // Link1_v = vJ, for the first link of a fixed base robot
    
    Link1_f = iit::rbd::vxIv(qd(JOINT1), Link1_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'Link1'
    Link1_a = (xm->fr_Link1_X_fr_InvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    Link1_v(iit::rbd::AZ) = qd(JOINT1);   // Link1_v = vJ, for the first link of a fixed base robot
    
    Link1_f = Link1_I * Link1_a + iit::rbd::vxIv(qd(JOINT1), Link1_I)  - fext[LINK1];
    
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'Link1'
    jForces(JOINT1) = Link1_f(iit::rbd::AZ);
}
