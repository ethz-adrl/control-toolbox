
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ct_DoubleInvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ct_DoubleInvertedPendulum::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    Link1_v.setZero();
    Link1_c.setZero();
    Link2_v.setZero();
    Link2_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    Link1_AI = inertiaProps->getTensor_Link1();
    Link1_p = - fext[LINK1];
    Link2_AI = inertiaProps->getTensor_Link2();
    Link2_p = - fext[LINK2];
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
    
    // + Link Link2
    //  - The spatial velocity:
    Link2_v = (motionTransforms-> fr_Link2_X_fr_Link1) * Link1_v;
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(Link2_v, vcross);
    Link2_c = vcross.col(iit::rbd::AZ) * qd(JOINT2);
    
    //  - The bias force term:
    Link2_p += iit::rbd::vxIv(Link2_v, Link2_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link Link2
    Link2_u = tau(JOINT2) - Link2_p(iit::rbd::AZ);
    Link2_U = Link2_AI.col(iit::rbd::AZ);
    Link2_D = Link2_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(Link2_AI, Link2_U, Link2_D, Ia_r);  // same as: Ia_r = Link2_AI - Link2_U/Link2_D * Link2_U.transpose();
    pa = Link2_p + Ia_r * Link2_c + Link2_U * Link2_u/Link2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link2_X_fr_Link1, IaB);
    Link1_AI += IaB;
    Link1_p += (motionTransforms-> fr_Link2_X_fr_Link1).transpose() * pa;
    
    // + Link Link1
    Link1_u = tau(JOINT1) - Link1_p(iit::rbd::AZ);
    Link1_U = Link1_AI.col(iit::rbd::AZ);
    Link1_D = Link1_U(iit::rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    Link1_a = (motionTransforms-> fr_Link1_X_fr_DoubleInvertedPendulumBase).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JOINT1) = (Link1_u - Link1_U.dot(Link1_a)) / Link1_D;
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    Link2_a = (motionTransforms-> fr_Link2_X_fr_Link1) * Link1_a + Link2_c;
    qdd(JOINT2) = (Link2_u - Link2_U.dot(Link2_a)) / Link2_D;
    Link2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    
}
