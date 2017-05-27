
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    link1_I(inertiaProps->getTensor_link1() ),
    link2_I(inertiaProps->getTensor_link2() )
    ,
        body_I( inertiaProps->getTensor_body() ),
        link2_Ic(link2_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ct_quadrotor, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    link1_v.setZero();
    link2_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& body_a,
    const Acceleration& g, const Velocity& body_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    body_Ic = body_I;
    link1_Ic = link1_I;

    // First pass, link 'link1'
    link1_v = ((xm->fr_link1_X_fr_body) * body_v);
    link1_v(iit::rbd::AZ) += qd(JA);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link1_v, vcross);
    
    link1_a = (vcross.col(iit::rbd::AZ) * qd(JA));
    link1_a(iit::rbd::AZ) += qdd(JA);
    
    link1_f = link1_I * link1_a + iit::rbd::vxIv(link1_v, link1_I);
    
    // First pass, link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::AZ) += qd(JB);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a + vcross.col(iit::rbd::AZ) * qd(JB);
    link2_a(iit::rbd::AZ) += qdd(JB);
    
    link2_f = link2_I * link2_a + iit::rbd::vxIv(link2_v, link2_I);
    
    // The force exerted on the floating base by the links
    body_f = iit::rbd::vxIv(body_v, body_I);
    

    // Add the external forces:
    body_f -= fext[BODY];
    link1_f -= fext[LINK1];
    link2_f -= fext[LINK2];

    link1_Ic = link1_Ic + (xm->fr_link2_X_fr_link1).transpose() * link2_Ic * (xm->fr_link2_X_fr_link1);
    link1_f = link1_f + (xm->fr_link2_X_fr_link1).transpose() * link2_f;
    
    body_Ic = body_Ic + (xm->fr_link1_X_fr_body).transpose() * link1_Ic * (xm->fr_link1_X_fr_body);
    body_f = body_f + (xm->fr_link1_X_fr_body).transpose() * link1_f;
    

    // The base acceleration due to the force due to the movement of the links
    body_a = - body_Ic.inverse() * body_f;
    
    link1_a = xm->fr_link1_X_fr_body * body_a;
    jForces(JA) = (link1_Ic.row(iit::rbd::AZ) * link1_a + link1_f(iit::rbd::AZ));
    
    link2_a = xm->fr_link2_X_fr_link1 * link1_a;
    jForces(JB) = (link2_Ic.row(iit::rbd::AZ) * link2_a + link2_f(iit::rbd::AZ));
    

    body_a += g;
}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& body_a = -g;

    // Link 'link1'
    link1_a = (xm->fr_link1_X_fr_body) * body_a;
    link1_f = link1_I * link1_a;
    // Link 'link2'
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a;
    link2_f = link2_I * link2_a;

    body_f = body_I * body_a;

    secondPass_fullyActuated(jForces);

    baseWrench = body_f;
}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& body_v, const JointState& qd)
{
    // Link 'link1'
    link1_v = ((xm->fr_link1_X_fr_body) * body_v);
    link1_v(iit::rbd::AZ) += qd(JA);
    iit::rbd::motionCrossProductMx<SCALAR>(link1_v, vcross);
    link1_a = (vcross.col(iit::rbd::AZ) * qd(JA));
    link1_f = link1_I * link1_a + iit::rbd::vxIv(link1_v, link1_I);
    
    // Link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::AZ) += qd(JB);
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a + vcross.col(iit::rbd::AZ) * qd(JB);
    link2_f = link2_I * link2_a + iit::rbd::vxIv(link2_v, link2_I);
    

    body_f = iit::rbd::vxIv(body_v, body_I);

    secondPass_fullyActuated(jForces);

    baseWrench = body_f;
}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& body_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration body_a = baseAccel -g;

    // First pass, link 'link1'
    link1_v = ((xm->fr_link1_X_fr_body) * body_v);
    link1_v(iit::rbd::AZ) += qd(JA);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link1_v, vcross);
    
    link1_a = (xm->fr_link1_X_fr_body) * body_a + vcross.col(iit::rbd::AZ) * qd(JA);
    link1_a(iit::rbd::AZ) += qdd(JA);
    
    link1_f = link1_I * link1_a + iit::rbd::vxIv(link1_v, link1_I) - fext[LINK1];
    
    // First pass, link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::AZ) += qd(JB);
    
    iit::rbd::motionCrossProductMx<SCALAR>(link2_v, vcross);
    
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a + vcross.col(iit::rbd::AZ) * qd(JB);
    link2_a(iit::rbd::AZ) += qdd(JB);
    
    link2_f = link2_I * link2_a + iit::rbd::vxIv(link2_v, link2_I) - fext[LINK2];
    

    // The base
    body_f = body_I * body_a + iit::rbd::vxIv(body_v, body_I) - fext[BODY];

    secondPass_fullyActuated(jForces);

    baseWrench = body_f;
}

template <typename TRAIT>
void iit::ct_quadrotor::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'link2'
    jForces(JB) = link2_f(iit::rbd::AZ);
    link1_f += xm->fr_link2_X_fr_link1.transpose() * link2_f;
    // Link 'link1'
    jForces(JA) = link1_f(iit::rbd::AZ);
    body_f += xm->fr_link1_X_fr_body.transpose() * link1_f;
}


