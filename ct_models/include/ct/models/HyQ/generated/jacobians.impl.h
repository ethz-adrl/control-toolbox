
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_trunk_J_LF_hipassemblyCOM(), 
    fr_trunk_J_RF_hipassemblyCOM(), 
    fr_trunk_J_LH_hipassemblyCOM(), 
    fr_trunk_J_RH_hipassemblyCOM(), 
    fr_trunk_J_LF_upperlegCOM(), 
    fr_trunk_J_RF_upperlegCOM(), 
    fr_trunk_J_LH_upperlegCOM(), 
    fr_trunk_J_RH_upperlegCOM(), 
    fr_trunk_J_LF_lowerlegCOM(), 
    fr_trunk_J_RF_lowerlegCOM(), 
    fr_trunk_J_LH_lowerlegCOM(), 
    fr_trunk_J_RH_lowerlegCOM(), 
    fr_trunk_J_fr_LF_foot(), 
    fr_trunk_J_fr_RF_foot(), 
    fr_trunk_J_fr_LH_foot(), 
    fr_trunk_J_fr_RH_foot()
{
    updateParameters();
}

template <typename TRAIT>
void iit::HyQ::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_hipassemblyCOM::Type_fr_trunk_J_LF_hipassemblyCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_hipassemblyCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_hipassemblyCOM::update(const JState& jState) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( jState(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( jState(LF_HAA));
    
    (*this)(4,0) = (- 0.043 *  cos__q_LF_HAA__);
    (*this)(5,0) = ( 0.043 *  sin__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_hipassemblyCOM::Type_fr_trunk_J_RF_hipassemblyCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_hipassemblyCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_hipassemblyCOM::update(const JState& jState) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( jState(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( jState(RF_HAA));
    
    (*this)(4,0) = ( 0.043 *  cos__q_RF_HAA__);
    (*this)(5,0) = ( 0.043 *  sin__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_hipassemblyCOM::Type_fr_trunk_J_LH_hipassemblyCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_hipassemblyCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_hipassemblyCOM::update(const JState& jState) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( jState(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( jState(LH_HAA));
    
    (*this)(4,0) = (- 0.043 *  cos__q_LH_HAA__);
    (*this)(5,0) = ( 0.043 *  sin__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_hipassemblyCOM::Type_fr_trunk_J_RH_hipassemblyCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_hipassemblyCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_hipassemblyCOM::update(const JState& jState) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( jState(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( jState(RH_HAA));
    
    (*this)(4,0) = ( 0.043 *  cos__q_RH_HAA__);
    (*this)(5,0) = ( 0.043 *  sin__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_upperlegCOM::Type_fr_trunk_J_LF_upperlegCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_upperlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_upperlegCOM::update(const JState& jState) {
    Scalar sin__q_LF_HAA__;
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HAA__ = TRAIT::sin( jState(LF_HAA));
    sin__q_LF_HFE__ = TRAIT::sin( jState(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( jState(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( jState(LF_HFE));
    
    (*this)(1,1) =  cos__q_LF_HAA__;
    (*this)(2,1) = - sin__q_LF_HAA__;
    (*this)(3,1) = ((- 0.026 *  sin__q_LF_HFE__) - ( 0.151 *  cos__q_LF_HFE__));
    (*this)(4,0) = ((((- 0.026 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.151 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    (*this)(4,1) = ((( 0.151 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.026 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(5,0) = (((( 0.026 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.151 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.08 *  sin__q_LF_HAA__));
    (*this)(5,1) = ((( 0.151 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.026 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_upperlegCOM::Type_fr_trunk_J_RF_upperlegCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_upperlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_upperlegCOM::update(const JState& jState) {
    Scalar sin__q_RF_HAA__;
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HAA__ = TRAIT::sin( jState(RF_HAA));
    sin__q_RF_HFE__ = TRAIT::sin( jState(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( jState(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( jState(RF_HFE));
    
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(3,1) = ((- 0.026 *  sin__q_RF_HFE__) - ( 0.151 *  cos__q_RF_HFE__));
    (*this)(4,0) = (((( 0.026 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.151 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  cos__q_RF_HAA__));
    (*this)(4,1) = ((( 0.026 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.151 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,0) = (((( 0.026 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.151 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__));
    (*this)(5,1) = ((( 0.151 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.026 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_upperlegCOM::Type_fr_trunk_J_LH_upperlegCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_upperlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_upperlegCOM::update(const JState& jState) {
    Scalar sin__q_LH_HAA__;
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HAA__ = TRAIT::sin( jState(LH_HAA));
    sin__q_LH_HFE__ = TRAIT::sin( jState(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( jState(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( jState(LH_HFE));
    
    (*this)(1,1) =  cos__q_LH_HAA__;
    (*this)(2,1) = - sin__q_LH_HAA__;
    (*this)(3,1) = (( 0.026 *  sin__q_LH_HFE__) - ( 0.151 *  cos__q_LH_HFE__));
    (*this)(4,0) = (((( 0.026 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.151 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    (*this)(4,1) = ((( 0.151 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.026 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(5,0) = ((((- 0.026 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.151 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.08 *  sin__q_LH_HAA__));
    (*this)(5,1) = ((( 0.151 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.026 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_upperlegCOM::Type_fr_trunk_J_RH_upperlegCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_upperlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_upperlegCOM::update(const JState& jState) {
    Scalar sin__q_RH_HAA__;
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HAA__ = TRAIT::sin( jState(RH_HAA));
    sin__q_RH_HFE__ = TRAIT::sin( jState(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( jState(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( jState(RH_HFE));
    
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(3,1) = (( 0.026 *  sin__q_RH_HFE__) - ( 0.151 *  cos__q_RH_HFE__));
    (*this)(4,0) = ((((- 0.026 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.151 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  cos__q_RH_HAA__));
    (*this)(4,1) = (((- 0.151 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.026 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(5,0) = ((((- 0.026 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.151 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__));
    (*this)(5,1) = ((( 0.151 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.026 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_lowerlegCOM::Type_fr_trunk_J_LF_lowerlegCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_lowerlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LF_lowerlegCOM::update(const JState& jState) {
    Scalar sin__q_LF_HAA__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_HAA__ = TRAIT::sin( jState(LF_HAA));
    sin__q_LF_HFE__ = TRAIT::sin( jState(LF_HFE));
    sin__q_LF_KFE__ = TRAIT::sin( jState(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( jState(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( jState(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( jState(LF_KFE));
    
    (*this)(1,1) =  cos__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,1) = - sin__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,1) = (((( 0.125 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.125 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - ( 0.35 *  cos__q_LF_HFE__));
    (*this)(3,2) = ((( 0.125 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.125 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,0) = (((((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    (*this)(4,1) = ((((( 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,2) = (((( 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,0) = ((((((- 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.08 *  sin__q_LF_HAA__));
    (*this)(5,1) = ((((( 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,2) = (((( 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_lowerlegCOM::Type_fr_trunk_J_RF_lowerlegCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_lowerlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RF_lowerlegCOM::update(const JState& jState) {
    Scalar sin__q_RF_HAA__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_HAA__ = TRAIT::sin( jState(RF_HAA));
    sin__q_RF_HFE__ = TRAIT::sin( jState(RF_HFE));
    sin__q_RF_KFE__ = TRAIT::sin( jState(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( jState(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( jState(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( jState(RF_KFE));
    
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,1) = ((((( 0.125 *  sin__q_RF_HFE__) + ( 0.001 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((( 0.001 *  sin__q_RF_HFE__) - ( 0.125 *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.35 *  cos__q_RF_HFE__));
    (*this)(3,2) = (((( 0.125 *  sin__q_RF_HFE__) + ( 0.001 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((( 0.001 *  sin__q_RF_HFE__) - ( 0.125 *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(4,0) = (((((((- 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  cos__q_RF_HAA__));
    (*this)(4,1) = (((((( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((((- 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,2) = ((((( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((((- 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(5,0) = (((((((- 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__));
    (*this)(5,1) = (((((( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,2) = ((((( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_lowerlegCOM::Type_fr_trunk_J_LH_lowerlegCOM()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_lowerlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_LH_lowerlegCOM::update(const JState& jState) {
    Scalar sin__q_LH_HAA__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_HAA__ = TRAIT::sin( jState(LH_HAA));
    sin__q_LH_HFE__ = TRAIT::sin( jState(LH_HFE));
    sin__q_LH_KFE__ = TRAIT::sin( jState(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( jState(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( jState(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( jState(LH_KFE));
    
    (*this)(1,1) =  cos__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,1) = - sin__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,1) = ((((( 0.125 *  sin__q_LH_HFE__) - ( 0.001 *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((- 0.001 *  sin__q_LH_HFE__) - ( 0.125 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - ( 0.35 *  cos__q_LH_HFE__));
    (*this)(3,2) = (((( 0.125 *  sin__q_LH_HFE__) - ( 0.001 *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((- 0.001 *  sin__q_LH_HFE__) - ( 0.125 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(4,0) = ((((((( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((- 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    (*this)(4,1) = (((((( 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(4,2) = ((((( 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(5,0) = ((((((( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + (( 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.08 *  sin__q_LH_HAA__));
    (*this)(5,1) = (((((( 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(5,2) = ((((( 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_lowerlegCOM::Type_fr_trunk_J_RH_lowerlegCOM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_lowerlegCOM& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_RH_lowerlegCOM::update(const JState& jState) {
    Scalar sin__q_RH_HAA__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_HAA__ = TRAIT::sin( jState(RH_HAA));
    sin__q_RH_HFE__ = TRAIT::sin( jState(RH_HFE));
    sin__q_RH_KFE__ = TRAIT::sin( jState(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( jState(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( jState(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( jState(RH_KFE));
    
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,1) = ((((( 0.1254 *  sin__q_RH_HFE__) - ( 5.0E-4 *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((- 5.0E-4 *  sin__q_RH_HFE__) - ( 0.1254 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) - ( 0.35 *  cos__q_RH_HFE__));
    (*this)(3,2) = (((( 0.1254 *  sin__q_RH_HFE__) - ( 5.0E-4 *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((- 5.0E-4 *  sin__q_RH_HFE__) - ( 0.1254 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((((( 5.0E-4 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.1254 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 5.0E-4 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.1254 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  cos__q_RH_HAA__));
    (*this)(4,1) = ((((((- 5.0E-4 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.1254 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 5.0E-4 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.1254 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) *  cos__q_RH_KFE__)) - (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(4,2) = (((((- 5.0E-4 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.1254 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 5.0E-4 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.1254 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(5,0) = ((((((( 5.0E-4 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.1254 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 5.0E-4 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.1254 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__));
    (*this)(5,1) = (((((( 5.0E-4 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.1254 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.1254 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 5.0E-4 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(5,2) = ((((( 5.0E-4 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.1254 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.1254 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 5.0E-4 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LF_foot::Type_fr_trunk_J_fr_LF_foot()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LF_foot& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LF_foot::update(const JState& jState) {
    Scalar sin__q_LF_HAA__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_HAA__ = TRAIT::sin( jState(LF_HAA));
    sin__q_LF_HFE__ = TRAIT::sin( jState(LF_HFE));
    sin__q_LF_KFE__ = TRAIT::sin( jState(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( jState(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( jState(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( jState(LF_KFE));
    
    (*this)(1,1) =  cos__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,1) = - sin__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,1) = (((( 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - ( 0.35 *  cos__q_LF_HFE__));
    (*this)(3,2) = ((( 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,0) = (((((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    (*this)(4,1) = ((((( 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,2) = (((( 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.08 *  sin__q_LF_HAA__));
    (*this)(5,1) = ((((( 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,2) = (((( 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RF_foot::Type_fr_trunk_J_fr_RF_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RF_foot& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RF_foot::update(const JState& jState) {
    Scalar sin__q_RF_HAA__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_HAA__ = TRAIT::sin( jState(RF_HAA));
    sin__q_RF_HFE__ = TRAIT::sin( jState(RF_HFE));
    sin__q_RF_KFE__ = TRAIT::sin( jState(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( jState(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( jState(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( jState(RF_KFE));
    
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,1) = (((( 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) - ( 0.35 *  cos__q_RF_HFE__));
    (*this)(3,2) = ((( 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((((- 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  cos__q_RF_HAA__));
    (*this)(4,1) = (((((- 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,2) = ((((- 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__));
    (*this)(5,1) = ((((( 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,2) = (((( 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LH_foot::Type_fr_trunk_J_fr_LH_foot()
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LH_foot& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_LH_foot::update(const JState& jState) {
    Scalar sin__q_LH_HAA__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_HAA__ = TRAIT::sin( jState(LH_HAA));
    sin__q_LH_HFE__ = TRAIT::sin( jState(LH_HFE));
    sin__q_LH_KFE__ = TRAIT::sin( jState(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( jState(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( jState(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( jState(LH_KFE));
    
    (*this)(1,1) =  cos__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,1) = - sin__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,1) = (((( 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - ( 0.35 *  cos__q_LH_HFE__));
    (*this)(3,2) = ((( 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,0) = (((((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    (*this)(4,1) = ((((( 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(4,2) = (((( 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.08 *  sin__q_LH_HAA__));
    (*this)(5,1) = ((((( 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(5,2) = (((( 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    return *this;
}
template <typename TRAIT>
iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RH_foot::Type_fr_trunk_J_fr_RH_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RH_foot& iit::HyQ::tpl::Jacobians<TRAIT>::Type_fr_trunk_J_fr_RH_foot::update(const JState& jState) {
    Scalar sin__q_RH_HAA__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_HAA__ = TRAIT::sin( jState(RH_HAA));
    sin__q_RH_HFE__ = TRAIT::sin( jState(RH_HFE));
    sin__q_RH_KFE__ = TRAIT::sin( jState(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( jState(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( jState(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( jState(RH_KFE));
    
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,1) = (((( 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) - ( 0.35 *  cos__q_RH_HFE__));
    (*this)(3,2) = ((( 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((((- 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  cos__q_RH_HAA__));
    (*this)(4,1) = (((((- 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(4,2) = ((((- 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__));
    (*this)(5,1) = ((((( 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(5,2) = (((( 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    return *this;
}
