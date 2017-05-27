

// Initialization of static-const data
template<typename TRAIT>
const typename iit::ct_quadrotor::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ct_quadrotor::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template<typename TRAIT>
iit::ct_quadrotor::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ct_quadrotor::dyn::tpl::InertiaProperties<TRAIT>& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    link1_v.setZero();
    link1_c.setZero();
    link2_v.setZero();
    link2_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& body_a,
    const Velocity& body_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    body_AI = inertiaProps->getTensor_body();
    body_p = - fext[BODY];
    link1_AI = inertiaProps->getTensor_link1();
    link1_p = - fext[LINK1];
    link2_AI = inertiaProps->getTensor_link2();
    link2_p = - fext[LINK2];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link link1
    //  - The spatial velocity:
    link1_v = (motionTransforms-> fr_link1_X_fr_body) * body_v;
    link1_v(rbd::AZ) += qd(JA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(link1_v, vcross);
    link1_c = vcross.col(rbd::AZ) * qd(JA);
    
    //  - The bias force term:
    link1_p += iit::rbd::vxIv(link1_v, link1_AI);
    
    // + Link link2
    //  - The spatial velocity:
    link2_v = (motionTransforms-> fr_link2_X_fr_link1) * link1_v;
    link2_v(rbd::AZ) += qd(JB);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    link2_c = vcross.col(rbd::AZ) * qd(JB);
    
    //  - The bias force term:
    link2_p += iit::rbd::vxIv(link2_v, link2_AI);
    
    // + The floating base body
    body_p += iit::rbd::vxIv(body_v, body_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link link2
    link2_u = tau(JB) - link2_p(rbd::AZ);
    link2_U = link2_AI.col(rbd::AZ);
    link2_D = link2_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link2_AI, link2_U, link2_D, Ia_r);  // same as: Ia_r = link2_AI - link2_U/link2_D * link2_U.transpose();
    pa = link2_p + Ia_r * link2_c + link2_U * link2_u/link2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link2_X_fr_link1, IaB);
    link1_AI += IaB;
    link1_p += (motionTransforms-> fr_link2_X_fr_link1).transpose() * pa;
    
    // + Link link1
    link1_u = tau(JA) - link1_p(rbd::AZ);
    link1_U = link1_AI.col(rbd::AZ);
    link1_D = link1_U(rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link1_AI, link1_U, link1_D, Ia_r);  // same as: Ia_r = link1_AI - link1_U/link1_D * link1_U.transpose();
    pa = link1_p + Ia_r * link1_c + link1_U * link1_u/link1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link1_X_fr_body, IaB);
    body_AI += IaB;
    body_p += (motionTransforms-> fr_link1_X_fr_body).transpose() * pa;
    
    // + The acceleration of the floating base body, without gravity
    body_a = - TRAIT::solve(body_AI, body_p);  // body_a = - IA^-1 * body_p
    
    // ---------------------- THIRD PASS ---------------------- //
    link1_a = (motionTransforms-> fr_link1_X_fr_body) * body_a + link1_c;
    qdd(JA) = (link1_u - link1_U.dot(link1_a)) / link1_D;
    link1_a(rbd::AZ) += qdd(JA);
    
    link2_a = (motionTransforms-> fr_link2_X_fr_link1) * link1_a + link2_c;
    qdd(JB) = (link2_u - link2_U.dot(link2_a)) / link2_D;
    link2_a(rbd::AZ) += qdd(JB);
    
    
    // + Add gravity to the acceleration of the floating base
    body_a += g;
}
