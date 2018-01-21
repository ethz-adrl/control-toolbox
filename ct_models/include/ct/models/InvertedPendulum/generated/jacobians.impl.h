
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_InvertedPendulumBase_J_fr_Link1(), 
    fr_InvertedPendulumBase_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_Link1::Type_fr_InvertedPendulumBase_J_fr_Link1()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_Link1& iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_Link1::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_ee::Type_fr_InvertedPendulumBase_J_fr_ee()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_ee& iit::ct_InvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_InvertedPendulumBase_J_fr_ee::update(const JState& jState) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( jState(JOINT1));
    c_q_Joint1_ = TRAIT::cos( jState(JOINT1));
    
    (*this)(4,0) = (- 0.435 *  c_q_Joint1_);
    (*this)(5,0) = (- 0.435 *  s_q_Joint1_);
    return *this;
}
