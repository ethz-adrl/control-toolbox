
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ct_InvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_InvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ct_InvertedPendulum::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    Link1_v.setZero();
    Link1_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    Link1_AI = inertiaProps->getTensor_Link1();
    Link1_p = - fext[LINK1];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link Link1
    //  - The spatial velocity:
    Link1_v(iit::rbd::AZ) = qd(JOINT1);
    
    //  - The bias force term:
    Link1_p += iit::rbd::vxIv(qd(JOINT1), Link1_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link Link1
    Link1_u = tau(JOINT1) - Link1_p(iit::rbd::AZ);
    Link1_U = Link1_AI.col(iit::rbd::AZ);
    Link1_D = Link1_U(iit::rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    Link1_a = (motionTransforms-> fr_Link1_X_fr_InvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JOINT1) = (Link1_u - Link1_U.dot(Link1_a)) / Link1_D;
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    
}
