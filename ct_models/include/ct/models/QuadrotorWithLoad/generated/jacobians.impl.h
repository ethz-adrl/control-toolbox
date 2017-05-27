
template <typename TRAIT>
iit::ct_quadrotor::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_body_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ct_quadrotor::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ct_quadrotor::tpl::Jacobians<TRAIT>::Type_fr_body_J_fr_ee::Type_fr_body_J_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::Jacobians<TRAIT>::Type_fr_body_J_fr_ee& iit::ct_quadrotor::tpl::Jacobians<TRAIT>::Type_fr_body_J_fr_ee::update(const JState& jState) {
    SCALAR sin__q_jA__;
    SCALAR sin__q_jB__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jA__ = TRAIT::sin( jState(JA));
    sin__q_jB__ = TRAIT::sin( jState(JB));
    cos__q_jA__ = TRAIT::cos( jState(JA));
    cos__q_jB__ = TRAIT::cos( jState(JB));
    
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,1) = - cos__q_jA__;
    (*this)(3,0) = ((- 0.7 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(3,1) = (( 0.7 *  cos__q_jA__) *  cos__q_jB__);
    (*this)(4,0) = (( 0.7 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(4,1) = (( 0.7 *  sin__q_jA__) *  cos__q_jB__);
    (*this)(5,1) = ( 0.7 *  sin__q_jB__);
    return *this;
}
