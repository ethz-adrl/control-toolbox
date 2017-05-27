
template <typename TRAIT>
iit::testirb4600::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_link0_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::testirb4600::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::testirb4600::tpl::Jacobians<TRAIT>::Type_fr_link0_J_fr_ee::Type_fr_link0_J_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,5) = 0;
}

template <typename TRAIT>
const typename iit::testirb4600::tpl::Jacobians<TRAIT>::Type_fr_link0_J_fr_ee& iit::testirb4600::tpl::Jacobians<TRAIT>::Type_fr_link0_J_fr_ee::update(const JState& jState) {
    SCALAR sin__q_jA__;
    SCALAR sin__q_jB__;
    SCALAR sin__q_jC__;
    SCALAR sin__q_jD__;
    SCALAR sin__q_jE__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    SCALAR cos__q_jC__;
    SCALAR cos__q_jD__;
    SCALAR cos__q_jE__;
    
    sin__q_jA__ = TRAIT::sin( jState(JA));
    sin__q_jB__ = TRAIT::sin( jState(JB));
    sin__q_jC__ = TRAIT::sin( jState(JC));
    sin__q_jD__ = TRAIT::sin( jState(JD));
    sin__q_jE__ = TRAIT::sin( jState(JE));
    cos__q_jA__ = TRAIT::cos( jState(JA));
    cos__q_jB__ = TRAIT::cos( jState(JB));
    cos__q_jC__ = TRAIT::cos( jState(JC));
    cos__q_jD__ = TRAIT::cos( jState(JD));
    cos__q_jE__ = TRAIT::cos( jState(JE));
    
    (*this)(0,1) = - sin__q_jA__;
    (*this)(0,2) = - sin__q_jA__;
    (*this)(0,3) = ((( cos__q_jA__ *  cos__q_jB__) *  cos__q_jC__) - (( cos__q_jA__ *  sin__q_jB__) *  sin__q_jC__));
    (*this)(0,4) = ((((( cos__q_jA__ *  cos__q_jB__) *  sin__q_jC__) + (( cos__q_jA__ *  sin__q_jB__) *  cos__q_jC__)) *  sin__q_jD__) - ( sin__q_jA__ *  cos__q_jD__));
    (*this)(0,5) = (((((((- cos__q_jA__ *  cos__q_jB__) *  sin__q_jC__) - (( cos__q_jA__ *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) - ( sin__q_jA__ *  sin__q_jD__)) *  sin__q_jE__) + (((( cos__q_jA__ *  cos__q_jB__) *  cos__q_jC__) - (( cos__q_jA__ *  sin__q_jB__) *  sin__q_jC__)) *  cos__q_jE__));
    (*this)(1,1) =  cos__q_jA__;
    (*this)(1,2) =  cos__q_jA__;
    (*this)(1,3) = ((( sin__q_jA__ *  cos__q_jB__) *  cos__q_jC__) - (( sin__q_jA__ *  sin__q_jB__) *  sin__q_jC__));
    (*this)(1,4) = ((((( sin__q_jA__ *  cos__q_jB__) *  sin__q_jC__) + (( sin__q_jA__ *  sin__q_jB__) *  cos__q_jC__)) *  sin__q_jD__) + ( cos__q_jA__ *  cos__q_jD__));
    (*this)(1,5) = (((( cos__q_jA__ *  sin__q_jD__) + ((((- sin__q_jA__ *  cos__q_jB__) *  sin__q_jC__) - (( sin__q_jA__ *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__)) *  sin__q_jE__) + (((( sin__q_jA__ *  cos__q_jB__) *  cos__q_jC__) - (( sin__q_jA__ *  sin__q_jB__) *  sin__q_jC__)) *  cos__q_jE__));
    (*this)(2,3) = ((- cos__q_jB__ *  sin__q_jC__) - ( sin__q_jB__ *  cos__q_jC__));
    (*this)(2,4) = ((( cos__q_jB__ *  cos__q_jC__) - ( sin__q_jB__ *  sin__q_jC__)) *  sin__q_jD__);
    (*this)(2,5) = ((((( sin__q_jB__ *  sin__q_jC__) - ( cos__q_jB__ *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((- cos__q_jB__ *  sin__q_jC__) - ( sin__q_jB__ *  cos__q_jC__)) *  cos__q_jE__));
    (*this)(3,0) = (((((((((((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  sin__q_jC__) + ((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) - (( 0.443 *  cos__q_jA__) *  sin__q_jD__)) *  sin__q_jE__) + ((((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + (((( 1.271 *  sin__q_jA__) *  sin__q_jB__) - (( 0.175 *  sin__q_jA__) *  cos__q_jB__)) *  sin__q_jC__)) + ((((- 0.175 *  sin__q_jA__) *  sin__q_jB__) - (( 1.271 *  sin__q_jA__) *  cos__q_jB__)) *  cos__q_jC__)) - (( 1.095 *  sin__q_jA__) *  sin__q_jB__)) - ( 0.175 *  sin__q_jA__));
    (*this)(3,1) = (((((((((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((((- 0.443 *  cos__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((((- 0.175 *  cos__q_jA__) *  sin__q_jB__) - (( 1.271 *  cos__q_jA__) *  cos__q_jB__)) *  sin__q_jC__)) + (((( 0.175 *  cos__q_jA__) *  cos__q_jB__) - (( 1.271 *  cos__q_jA__) *  sin__q_jB__)) *  cos__q_jC__)) + (( 1.095 *  cos__q_jA__) *  cos__q_jB__));
    (*this)(3,2) = ((((((((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((((- 0.443 *  cos__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((((- 0.175 *  cos__q_jA__) *  sin__q_jB__) - (( 1.271 *  cos__q_jA__) *  cos__q_jB__)) *  sin__q_jC__)) + (((( 0.175 *  cos__q_jA__) *  cos__q_jB__) - (( 1.271 *  cos__q_jA__) *  sin__q_jB__)) *  cos__q_jC__));
    (*this)(3,3) = ((((((( 0.443 *  cos__q_jA__) *  cos__q_jB__) *  sin__q_jC__) + ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  sin__q_jD__) - (( 0.443 *  sin__q_jA__) *  cos__q_jD__)) *  sin__q_jE__);
    (*this)(3,4) = (((((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  sin__q_jE__) + (((((((- 0.443 *  cos__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) - (( 0.443 *  sin__q_jA__) *  sin__q_jD__)) *  cos__q_jE__));
    (*this)(4,0) = ((((((((((((- 0.443 *  cos__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) - (( 0.443 *  sin__q_jA__) *  sin__q_jD__)) *  sin__q_jE__) + ((((( 0.443 *  cos__q_jA__) *  cos__q_jB__) *  cos__q_jC__) - ((( 0.443 *  cos__q_jA__) *  sin__q_jB__) *  sin__q_jC__)) *  cos__q_jE__)) + (((( 0.175 *  cos__q_jA__) *  cos__q_jB__) - (( 1.271 *  cos__q_jA__) *  sin__q_jB__)) *  sin__q_jC__)) + (((( 0.175 *  cos__q_jA__) *  sin__q_jB__) + (( 1.271 *  cos__q_jA__) *  cos__q_jB__)) *  cos__q_jC__)) + (( 1.095 *  cos__q_jA__) *  sin__q_jB__)) + ( 0.175 *  cos__q_jA__));
    (*this)(4,1) = (((((((((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((((- 0.443 *  sin__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((((- 0.175 *  sin__q_jA__) *  sin__q_jB__) - (( 1.271 *  sin__q_jA__) *  cos__q_jB__)) *  sin__q_jC__)) + (((( 0.175 *  sin__q_jA__) *  cos__q_jB__) - (( 1.271 *  sin__q_jA__) *  sin__q_jB__)) *  cos__q_jC__)) + (( 1.095 *  sin__q_jA__) *  cos__q_jB__));
    (*this)(4,2) = ((((((((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((((- 0.443 *  sin__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((((- 0.175 *  sin__q_jA__) *  sin__q_jB__) - (( 1.271 *  sin__q_jA__) *  cos__q_jB__)) *  sin__q_jC__)) + (((( 0.175 *  sin__q_jA__) *  cos__q_jB__) - (( 1.271 *  sin__q_jA__) *  sin__q_jB__)) *  cos__q_jC__));
    (*this)(4,3) = ((((((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  sin__q_jC__) + ((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  sin__q_jD__) + (( 0.443 *  cos__q_jA__) *  cos__q_jD__)) *  sin__q_jE__);
    (*this)(4,4) = (((((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  cos__q_jB__) *  cos__q_jC__)) *  sin__q_jE__) + (((( 0.443 *  cos__q_jA__) *  sin__q_jD__) + (((((- 0.443 *  sin__q_jA__) *  cos__q_jB__) *  sin__q_jC__) - ((( 0.443 *  sin__q_jA__) *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__)) *  cos__q_jE__));
    (*this)(5,1) = ((((((((( 0.443 *  cos__q_jB__) *  sin__q_jC__) + (( 0.443 *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((( 0.443 *  sin__q_jB__) *  sin__q_jC__) - (( 0.443 *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((( 1.271 *  sin__q_jB__) - ( 0.175 *  cos__q_jB__)) *  sin__q_jC__)) + (((- 0.175 *  sin__q_jB__) - ( 1.271 *  cos__q_jB__)) *  cos__q_jC__)) - ( 1.095 *  sin__q_jB__));
    (*this)(5,2) = (((((((( 0.443 *  cos__q_jB__) *  sin__q_jC__) + (( 0.443 *  sin__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  sin__q_jE__) + (((( 0.443 *  sin__q_jB__) *  sin__q_jC__) - (( 0.443 *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jE__)) + ((( 1.271 *  sin__q_jB__) - ( 0.175 *  cos__q_jB__)) *  sin__q_jC__)) + (((- 0.175 *  sin__q_jB__) - ( 1.271 *  cos__q_jB__)) *  cos__q_jC__));
    (*this)(5,3) = ((((( 0.443 *  cos__q_jB__) *  cos__q_jC__) - (( 0.443 *  sin__q_jB__) *  sin__q_jC__)) *  sin__q_jD__) *  sin__q_jE__);
    (*this)(5,4) = ((((( 0.443 *  cos__q_jB__) *  sin__q_jC__) + (( 0.443 *  sin__q_jB__) *  cos__q_jC__)) *  sin__q_jE__) + ((((( 0.443 *  sin__q_jB__) *  sin__q_jC__) - (( 0.443 *  cos__q_jB__) *  cos__q_jC__)) *  cos__q_jD__) *  cos__q_jE__));
    return *this;
}
