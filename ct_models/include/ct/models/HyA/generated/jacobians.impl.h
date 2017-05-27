
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_HyABase_J_fr_ee(), 
    fr_HyABase_J_fr_Wrist_FE_COM(), 
    fr_HyABase_J_fr_Wrist_FE(), 
    fr_HyABase_J_fr_Wrist_R_COM(), 
    fr_HyABase_J_fr_Wrist_R(), 
    fr_HyABase_J_fr_Wrist_R_CTR(), 
    fr_HyABase_J_fr_Elbow_FE_COM(), 
    fr_HyABase_J_fr_Elbow_FE(), 
    fr_HyABase_J_fr_Elbow_FE_CTR(), 
    fr_HyABase_J_fr_Humerus_R_COM(), 
    fr_HyABase_J_fr_Humerus_R(), 
    fr_HyABase_J_fr_Humerus_R_CTR(), 
    fr_HyABase_J_fr_Shoulder_AA_CTR(), 
    fr_HyABase_J_fr_Shoulder_FE_CTR()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ct_HyA::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_ee::Type_fr_HyABase_J_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_ee& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_ee::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_WFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    sin__q_WR__ = TRAIT::sin( jState(WR));
    sin__q_WFE__ = TRAIT::sin( jState(WFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    cos__q_WR__ = TRAIT::cos( jState(WR));
    cos__q_WFE__ = TRAIT::cos( jState(WFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,5) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,5) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = (((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((((((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((((- 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = ((((((((((((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.03 *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.02075 *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((((((((((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = ((((((((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = ((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((((- 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,0) = ((((((((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((- 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((((((((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((((- 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = ((((((((((((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((- 0.03 *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.02075 *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((((((((((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = ((((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = ((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 0.03 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(5,1) = ((((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__) - (( 0.03 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) - (( 0.02075 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.03 *  sin__q_HR__) *  sin__q_SFE__) *  sin__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  sin__q_SFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  cos__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) *  cos__q_WFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__));
    (*this)(5,2) = (((((((((- 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((( 0.03 *  cos__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (( 0.02075 *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) - (((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((((((((- 0.03 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((((((- 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__));
    (*this)(5,5) = (((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + ((((- 0.03 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE_COM::Type_fr_HyABase_J_fr_Wrist_FE_COM()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE_COM& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE_COM::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_WFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    sin__q_WR__ = TRAIT::sin( jState(WR));
    sin__q_WFE__ = TRAIT::sin( jState(WFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    cos__q_WR__ = TRAIT::cos( jState(WR));
    cos__q_WFE__ = TRAIT::cos( jState(WFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,5) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,5) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((((((((((- 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.07876 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.01063 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = ((((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) *  sin__q_WFE__) + (((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) *  cos__q_WFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((((- 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WFE__)) + ((((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = (((((((((((((((- 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((- 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.07876 *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((( 0.01063 *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((- 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((((((((((((((- 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((((((((- 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((- 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = ((((((((((((- 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((- 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((- 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,0) = (((((((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = ((((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) *  sin__q_WFE__) + (((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) *  cos__q_WFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((((- 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WFE__)) + ((((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = (((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - (((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + ((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - (((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) + (((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.07876 *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.07876 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((( 0.01063 *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.01063 *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - (((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((((((((((((((- 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((((((( 0.07876 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.01063 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = ((((((((((((- 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.01063 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) + ((((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((- 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(5,1) = (((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__) + (( 0.07876 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WFE__) + ((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__) + (( 0.01063 *  sin__q_EFE__) *  cos__q_SFE__)) *  cos__q_WFE__)) + (((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SFE__)) - (( 0.02075 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((( 0.07876 *  sin__q_HR__) *  sin__q_SFE__) *  sin__q_WFE__) + ((( 0.01063 *  sin__q_HR__) *  sin__q_SFE__) *  cos__q_WFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  sin__q_SFE__)) + (( 8.4E-4 *  sin__q_EFE__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.01063 *  cos__q_EFE__) *  cos__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) *  sin__q_WFE__)) + ((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_EFE__) *  cos__q_SFE__)) *  cos__q_WFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__));
    (*this)(5,2) = ((((((((((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.07876 *  cos__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.01063 *  cos__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__)) - (((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) + (((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((((((((((( 0.07876 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) + (((( 0.01063 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + ((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__)) + (((( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = (((((((( 0.07876 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.01063 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - (( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) + (((( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__));
    (*this)(5,5) = (((((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((( 0.01063 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.07876 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.07876 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + (((( 0.01063 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE::Type_fr_HyABase_J_fr_Wrist_FE()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,5) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_FE::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    sin__q_WR__ = TRAIT::sin( jState(WR));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    cos__q_WR__ = TRAIT::cos( jState(WR));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,5) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,5) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) - (((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__) *  cos__q_WR__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = ((((((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = (((((((((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = (((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) - (((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__) *  cos__q_WR__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = ((((((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((- 0.02075 *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) - (((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,1) = ((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WR__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SFE__) *  cos__q_WR__)) - ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__));
    (*this)(5,2) = ((((((- 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + ((( 0.02075 *  cos__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((((- 0.02075 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.296 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = (((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_COM::Type_fr_HyABase_J_fr_Wrist_R_COM()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_COM& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_COM::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    sin__q_WR__ = TRAIT::sin( jState(WR));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    cos__q_WR__ = TRAIT::cos( jState(WR));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = ((((((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__) + ((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__) - ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__)) - (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = (((((((((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + (((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((((((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.08883 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.08883 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.08883 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.00261 *  cos__q_EFE__) *  sin__q_HR__) + ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = (((((((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__) + ((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__)) - (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = (((((((((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + (((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((((((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.08883 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.08883 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.08883 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  cos__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,1) = ((((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SFE__) - (( 4.0E-4 *  sin__q_EFE__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  sin__q_SFE__) + (( 0.00261 *  sin__q_EFE__) *  cos__q_SFE__)) *  cos__q_WR__)) - ((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__)) + ((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__));
    (*this)(5,2) = (((((( 0.00261 *  cos__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SFE__) *  sin__q_WR__) + ((((( 0.00261 *  cos__q_EFE__) *  sin__q_HR__) + ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SFE__) *  cos__q_WR__)) - ((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (((((((- 4.0E-4 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((( 0.00261 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) - (( 0.08883 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.08883 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SFE__) - (( 0.00261 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WR__) + ((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 4.0E-4 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R::Type_fr_HyABase_J_fr_Wrist_R()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,4) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = (((((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((- 0.112 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.112 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = ((((((- 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((- 0.112 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.112 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__));
    (*this)(5,2) = (((- 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = (((( 0.112 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_CTR::Type_fr_HyABase_J_fr_Wrist_R_CTR()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,4) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_CTR& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Wrist_R_CTR::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = (((((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = ((((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((- 0.204 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.204 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = ((((((- 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = ((((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((- 0.204 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.204 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__));
    (*this)(5,2) = (((- 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = (((( 0.204 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.204 *  sin__q_EFE__) *  sin__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_COM::Type_fr_HyABase_J_fr_Elbow_FE_COM()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_COM& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_COM::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = ((((((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__) - ( 0.00102 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + (((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) + ( 0.00102 *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(3,1) = ((((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = (((((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) + ( 0.00102 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + (((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__) - ( 0.00102 *  sin__q_HR__)) *  sin__q_SAA__));
    (*this)(3,3) = ((((((( 0.1466 *  cos__q_EFE__) - ( 0.01125 *  sin__q_EFE__)) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = (((((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + (((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) + ( 0.00102 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(4,1) = ((((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = (((((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) + ( 0.00102 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(4,3) = ((((((( 0.1466 *  cos__q_EFE__) - ( 0.01125 *  sin__q_EFE__)) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.1466 *  cos__q_EFE__) - ( 0.01125 *  sin__q_EFE__)) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  sin__q_SFE__) + (((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_SFE__));
    (*this)(5,2) = ((((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) - ( 0.00102 *  cos__q_HR__)) *  cos__q_SFE__);
    (*this)(5,3) = (((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  sin__q_SFE__) + (((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE::Type_fr_HyABase_J_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = ((- 0.416 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = ((- 0.416 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,0) = (( 0.416 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = ((- 0.416 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(5,1) = ( 0.416 *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_CTR::Type_fr_HyABase_J_fr_Elbow_FE_CTR()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_CTR& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Elbow_FE_CTR::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_EFE__ = TRAIT::sin( jState(EFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    cos__q_EFE__ = TRAIT::cos( jState(EFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = (((((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((((- 0.056 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((- 0.056 *  cos__q_EFE__) -  0.416) *  cos__q_SAA__) *  sin__q_SFE__) - (((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  cos__q_SFE__));
    (*this)(3,2) = ((((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = ((((((- 0.056 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.056 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.056 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = ((((((- 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,1) = (((((- 0.056 *  cos__q_EFE__) -  0.416) *  sin__q_SAA__) *  sin__q_SFE__) - (((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  cos__q_SFE__));
    (*this)(4,2) = ((((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ((((((- 0.056 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.056 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.056 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_SFE__) - ((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SFE__));
    (*this)(5,2) = (((- 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = (((( 0.056 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.056 *  sin__q_EFE__) *  sin__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_COM::Type_fr_HyABase_J_fr_Humerus_R_COM()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_COM& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_COM::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    sin__q_HR__ = TRAIT::sin( jState(HR));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    cos__q_HR__ = TRAIT::cos( jState(HR));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = (((((( 0.00618 *  cos__q_HR__) - ( 0.00239 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.08646 *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(3,1) = ((((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  cos__q_SAA__) *  cos__q_SFE__) - (( 0.08646 *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,2) = ((((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00618 *  cos__q_HR__) - ( 0.00239 *  sin__q_HR__)) *  sin__q_SAA__));
    (*this)(4,0) = (((((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.08646 *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(4,1) = ((((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  sin__q_SAA__) *  cos__q_SFE__) - (( 0.08646 *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,2) = ((((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(5,1) = (((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  sin__q_SFE__) + ( 0.08646 *  cos__q_SFE__));
    (*this)(5,2) = (((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R::Type_fr_HyABase_J_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = ((- 0.112 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = ((- 0.112 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,0) = (( 0.112 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = ((- 0.112 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(5,1) = ( 0.112 *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_CTR::Type_fr_HyABase_J_fr_Humerus_R_CTR()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_CTR& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Humerus_R_CTR::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = ((- 0.264 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = ((- 0.264 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,0) = (( 0.264 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = ((- 0.264 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(5,1) = ( 0.264 *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_AA_CTR::Type_fr_HyABase_J_fr_Shoulder_AA_CTR()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_AA_CTR& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_AA_CTR::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_FE_CTR::Type_fr_HyABase_J_fr_Shoulder_FE_CTR()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_FE_CTR& iit::ct_HyA::tpl::Jacobians<TRAIT>::Type_fr_HyABase_J_fr_Shoulder_FE_CTR::update(const JState& jState) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( jState(SAA));
    sin__q_SFE__ = TRAIT::sin( jState(SFE));
    cos__q_SAA__ = TRAIT::cos( jState(SAA));
    cos__q_SFE__ = TRAIT::cos( jState(SFE));
    
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(1,1) = - cos__q_SAA__;
    (*this)(3,0) = ((- 0.056 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = ((- 0.056 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,0) = (( 0.056 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = ((- 0.056 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(5,1) = ( 0.056 *  cos__q_SFE__);
    return *this;
}
