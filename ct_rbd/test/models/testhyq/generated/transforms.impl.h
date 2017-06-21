
// Constructors
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_trunk_X_fr_LF_hipassembly(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_trunk_X_fr_LH_hipassembly(),
    fr_trunk_X_fr_RH_hipassembly(),
    fr_trunk_X_fr_LF_upperleg(),
    fr_trunk_X_fr_RF_upperleg(),
    fr_trunk_X_fr_LH_upperleg(),
    fr_trunk_X_fr_RH_upperleg(),
    fr_trunk_X_fr_LF_lowerleg(),
    fr_trunk_X_fr_RF_lowerleg(),
    fr_trunk_X_fr_LH_lowerleg(),
    fr_trunk_X_fr_RH_lowerleg(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_LH_hipassembly_X_fr_trunk(),
    fr_RH_hipassembly_X_fr_trunk(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_LH_upperleg_X_fr_trunk(),
    fr_RH_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_LH_lowerleg_X_fr_trunk(),
    fr_RH_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_LH_hipassemblyCOM(),
    fr_trunk_X_RH_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_LH_upperlegCOM(),
    fr_trunk_X_RH_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_LH_lowerlegCOM(),
    fr_trunk_X_RH_lowerlegCOM(),
    fr_LF_foot_X_fr_LF_lowerleg(),
    fr_RF_foot_X_fr_RF_lowerleg(),
    fr_LH_foot_X_fr_LH_lowerleg(),
    fr_RH_foot_X_fr_RH_lowerleg(),
    fr_trunk_X_fr_LF_foot(),
    fr_trunk_X_fr_RF_foot(),
    fr_trunk_X_fr_LH_foot(),
    fr_trunk_X_fr_RH_foot(),
    fr_LF_foot_X_fr_trunk(),
    fr_RF_foot_X_fr_trunk(),
    fr_LH_foot_X_fr_trunk(),
    fr_RH_foot_X_fr_trunk(),
    fr_trunk_X_fr_LF_HAA(),
    fr_trunk_X_fr_RF_HAA(),
    fr_trunk_X_fr_LH_HAA(),
    fr_trunk_X_fr_RH_HAA(),
    fr_trunk_X_fr_LF_HFE(),
    fr_trunk_X_fr_RF_HFE(),
    fr_trunk_X_fr_LH_HFE(),
    fr_trunk_X_fr_RH_HFE(),
    fr_trunk_X_fr_LF_KFE(),
    fr_trunk_X_fr_RF_KFE(),
    fr_trunk_X_fr_LH_KFE(),
    fr_trunk_X_fr_RH_KFE(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_LH_upperleg_X_fr_LH_hipassembly(),
    fr_LH_hipassembly_X_fr_LH_upperleg(),
    fr_LH_lowerleg_X_fr_LH_upperleg(),
    fr_LH_upperleg_X_fr_LH_lowerleg(),
    fr_RH_upperleg_X_fr_RH_hipassembly(),
    fr_RH_hipassembly_X_fr_RH_upperleg(),
    fr_RH_lowerleg_X_fr_RH_upperleg(),
    fr_RH_upperleg_X_fr_RH_lowerleg()
{
    updateParameters();
}
template <typename TRAIT>
void iit::TestHyQ::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_trunk_X_fr_LF_hipassembly(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_trunk_X_fr_LH_hipassembly(),
    fr_trunk_X_fr_RH_hipassembly(),
    fr_trunk_X_fr_LF_upperleg(),
    fr_trunk_X_fr_RF_upperleg(),
    fr_trunk_X_fr_LH_upperleg(),
    fr_trunk_X_fr_RH_upperleg(),
    fr_trunk_X_fr_LF_lowerleg(),
    fr_trunk_X_fr_RF_lowerleg(),
    fr_trunk_X_fr_LH_lowerleg(),
    fr_trunk_X_fr_RH_lowerleg(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_LH_hipassembly_X_fr_trunk(),
    fr_RH_hipassembly_X_fr_trunk(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_LH_upperleg_X_fr_trunk(),
    fr_RH_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_LH_lowerleg_X_fr_trunk(),
    fr_RH_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_LH_hipassemblyCOM(),
    fr_trunk_X_RH_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_LH_upperlegCOM(),
    fr_trunk_X_RH_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_LH_lowerlegCOM(),
    fr_trunk_X_RH_lowerlegCOM(),
    fr_LF_foot_X_fr_LF_lowerleg(),
    fr_RF_foot_X_fr_RF_lowerleg(),
    fr_LH_foot_X_fr_LH_lowerleg(),
    fr_RH_foot_X_fr_RH_lowerleg(),
    fr_trunk_X_fr_LF_foot(),
    fr_trunk_X_fr_RF_foot(),
    fr_trunk_X_fr_LH_foot(),
    fr_trunk_X_fr_RH_foot(),
    fr_LF_foot_X_fr_trunk(),
    fr_RF_foot_X_fr_trunk(),
    fr_LH_foot_X_fr_trunk(),
    fr_RH_foot_X_fr_trunk(),
    fr_trunk_X_fr_LF_HAA(),
    fr_trunk_X_fr_RF_HAA(),
    fr_trunk_X_fr_LH_HAA(),
    fr_trunk_X_fr_RH_HAA(),
    fr_trunk_X_fr_LF_HFE(),
    fr_trunk_X_fr_RF_HFE(),
    fr_trunk_X_fr_LH_HFE(),
    fr_trunk_X_fr_RH_HFE(),
    fr_trunk_X_fr_LF_KFE(),
    fr_trunk_X_fr_RF_KFE(),
    fr_trunk_X_fr_LH_KFE(),
    fr_trunk_X_fr_RH_KFE(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_LH_upperleg_X_fr_LH_hipassembly(),
    fr_LH_hipassembly_X_fr_LH_upperleg(),
    fr_LH_lowerleg_X_fr_LH_upperleg(),
    fr_LH_upperleg_X_fr_LH_lowerleg(),
    fr_RH_upperleg_X_fr_RH_hipassembly(),
    fr_RH_hipassembly_X_fr_RH_upperleg(),
    fr_RH_lowerleg_X_fr_RH_upperleg(),
    fr_RH_upperleg_X_fr_RH_lowerleg()
{
    updateParameters();
}
template <typename TRAIT>
void iit::TestHyQ::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_trunk_X_fr_LF_hipassembly(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_trunk_X_fr_LH_hipassembly(),
    fr_trunk_X_fr_RH_hipassembly(),
    fr_trunk_X_fr_LF_upperleg(),
    fr_trunk_X_fr_RF_upperleg(),
    fr_trunk_X_fr_LH_upperleg(),
    fr_trunk_X_fr_RH_upperleg(),
    fr_trunk_X_fr_LF_lowerleg(),
    fr_trunk_X_fr_RF_lowerleg(),
    fr_trunk_X_fr_LH_lowerleg(),
    fr_trunk_X_fr_RH_lowerleg(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_LH_hipassembly_X_fr_trunk(),
    fr_RH_hipassembly_X_fr_trunk(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_LH_upperleg_X_fr_trunk(),
    fr_RH_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_LH_lowerleg_X_fr_trunk(),
    fr_RH_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_LH_hipassemblyCOM(),
    fr_trunk_X_RH_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_LH_upperlegCOM(),
    fr_trunk_X_RH_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_LH_lowerlegCOM(),
    fr_trunk_X_RH_lowerlegCOM(),
    fr_LF_foot_X_fr_LF_lowerleg(),
    fr_RF_foot_X_fr_RF_lowerleg(),
    fr_LH_foot_X_fr_LH_lowerleg(),
    fr_RH_foot_X_fr_RH_lowerleg(),
    fr_trunk_X_fr_LF_foot(),
    fr_trunk_X_fr_RF_foot(),
    fr_trunk_X_fr_LH_foot(),
    fr_trunk_X_fr_RH_foot(),
    fr_LF_foot_X_fr_trunk(),
    fr_RF_foot_X_fr_trunk(),
    fr_LH_foot_X_fr_trunk(),
    fr_RH_foot_X_fr_trunk(),
    fr_trunk_X_fr_LF_HAA(),
    fr_trunk_X_fr_RF_HAA(),
    fr_trunk_X_fr_LH_HAA(),
    fr_trunk_X_fr_RH_HAA(),
    fr_trunk_X_fr_LF_HFE(),
    fr_trunk_X_fr_RF_HFE(),
    fr_trunk_X_fr_LH_HFE(),
    fr_trunk_X_fr_RH_HFE(),
    fr_trunk_X_fr_LF_KFE(),
    fr_trunk_X_fr_RF_KFE(),
    fr_trunk_X_fr_LH_KFE(),
    fr_trunk_X_fr_RH_KFE(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_LH_upperleg_X_fr_LH_hipassembly(),
    fr_LH_hipassembly_X_fr_LH_upperleg(),
    fr_LH_lowerleg_X_fr_LH_upperleg(),
    fr_LH_upperleg_X_fr_LH_lowerleg(),
    fr_RH_upperleg_X_fr_RH_hipassembly(),
    fr_RH_hipassembly_X_fr_RH_upperleg(),
    fr_RH_lowerleg_X_fr_RH_upperleg(),
    fr_RH_upperleg_X_fr_RH_lowerleg()
{
    updateParameters();
}
template <typename TRAIT>
void iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(3,1) = ( 0.207 *  sin__q_LF_HAA__);
    (*this)(4,0) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(4,1) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(5,0) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(5,1) = (- 0.3735 *  cos__q_LF_HAA__);
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,4) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(3,1) = (- 0.207 *  sin__q_RF_HAA__);
    (*this)(4,0) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(4,1) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(5,0) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(5,1) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,4) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::Type_fr_trunk_X_fr_LH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(3,1) = ( 0.207 *  sin__q_LH_HAA__);
    (*this)(4,0) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(4,1) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(5,0) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(5,1) = ( 0.3735 *  cos__q_LH_HAA__);
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,4) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::Type_fr_trunk_X_fr_RH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(3,1) = (- 0.207 *  sin__q_RH_HAA__);
    (*this)(4,0) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(4,1) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(5,0) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(5,1) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,4) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::Type_fr_trunk_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(4,1) = ((( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,2) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(5,1) = ((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__));
    (*this)(5,2) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::Type_fr_trunk_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(4,1) = ((( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,2) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(5,1) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,2) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::Type_fr_trunk_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(4,1) = ((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(4,2) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(5,1) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(5,2) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::Type_fr_trunk_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(4,1) = ((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(4,2) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(5,1) = ((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__));
    (*this)(5,2) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::Type_fr_trunk_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,2) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,0) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(4,1) = (((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(4,2) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(5,1) = (((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(5,2) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::Type_fr_trunk_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(3,2) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(4,1) = (((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(4,2) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(5,1) = ((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(5,2) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::Type_fr_trunk_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,2) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,0) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(4,1) = ((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(4,2) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(5,1) = (((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(5,2) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::Type_fr_trunk_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(3,2) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,0) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(4,1) = ((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(4,2) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(5,1) = ((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(5,2) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) = - sin__q_LF_HAA__;
    (*this)(0,2) = - cos__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,2) =  sin__q_LF_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(3,1) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(3,2) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(3,4) = - sin__q_LF_HAA__;
    (*this)(3,5) = - cos__q_LF_HAA__;
    (*this)(4,0) = ( 0.207 *  sin__q_LF_HAA__);
    (*this)(4,1) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(4,2) = (- 0.3735 *  cos__q_LF_HAA__);
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(4,5) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  sin__q_RF_HAA__;
    (*this)(0,2) = - cos__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(3,1) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(3,2) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(3,4) =  sin__q_RF_HAA__;
    (*this)(3,5) = - cos__q_RF_HAA__;
    (*this)(4,0) = (- 0.207 *  sin__q_RF_HAA__);
    (*this)(4,1) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(4,2) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(4,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::Type_fr_LH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) = - sin__q_LH_HAA__;
    (*this)(0,2) = - cos__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,2) =  sin__q_LH_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(3,1) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(3,2) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(3,4) = - sin__q_LH_HAA__;
    (*this)(3,5) = - cos__q_LH_HAA__;
    (*this)(4,0) = ( 0.207 *  sin__q_LH_HAA__);
    (*this)(4,1) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(4,2) = ( 0.3735 *  cos__q_LH_HAA__);
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(4,5) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::Type_fr_RH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  sin__q_RH_HAA__;
    (*this)(0,2) = - cos__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(3,1) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(3,2) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(3,4) =  sin__q_RH_HAA__;
    (*this)(3,5) = - cos__q_RH_HAA__;
    (*this)(4,0) = (- 0.207 *  sin__q_RH_HAA__);
    (*this)(4,1) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(4,2) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(4,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(0,2) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,0) = - cos__q_LF_HFE__;
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(3,1) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(3,2) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(3,5) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,0) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(4,1) = ((( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,2) = ((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__));
    (*this)(4,3) = - cos__q_LF_HFE__;
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,0) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(5,1) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(5,2) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(0,2) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,0) = - cos__q_RF_HFE__;
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(3,1) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(3,2) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(3,5) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,0) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(4,1) = ((( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,2) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,3) = - cos__q_RF_HFE__;
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,0) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(5,1) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(5,2) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::Type_fr_LH_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(0,2) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,0) = - cos__q_LH_HFE__;
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(3,1) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(3,2) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(3,5) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,0) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(4,1) = ((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(4,2) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(4,3) = - cos__q_LH_HFE__;
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,0) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(5,1) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(5,2) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::Type_fr_RH_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(0,2) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,0) = - cos__q_RH_HFE__;
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(3,1) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(3,2) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(3,5) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,0) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(4,1) = ((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(4,2) = ((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__));
    (*this)(4,3) = - cos__q_RH_HFE__;
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,0) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(5,1) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(5,2) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,1) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(3,2) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,5) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,0) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,1) = (((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(4,2) = (((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(4,3) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,0) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(5,1) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,2) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,1) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(3,2) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,5) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,1) = (((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(4,2) = ((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(4,3) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,0) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(5,1) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(5,2) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::Type_fr_LH_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,1) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(3,2) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,5) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,0) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,1) = ((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(4,2) = (((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(4,3) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,0) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(5,1) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(5,2) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::Type_fr_RH_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,1) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(3,2) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,5) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,1) = ((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(4,2) = ((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(4,3) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,0) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(5,1) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(5,2) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(3,1) = (( 0.207 *  sin__q_LF_HAA__) -  0.043);
    (*this)(4,0) = ( 0.2045 *  cos__q_LF_HAA__);
    (*this)(4,1) = (- 0.2045 *  sin__q_LF_HAA__);
    (*this)(4,2) = ( 0.043 *  cos__q_LF_HAA__);
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(5,0) = (- 0.2045 *  sin__q_LF_HAA__);
    (*this)(5,1) = (- 0.2045 *  cos__q_LF_HAA__);
    (*this)(5,2) = ( 0.207 - ( 0.043 *  sin__q_LF_HAA__));
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,4) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(3,1) = ( 0.043 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(4,0) = ( 0.2045 *  cos__q_RF_HAA__);
    (*this)(4,1) = (- 0.2045 *  sin__q_RF_HAA__);
    (*this)(4,2) = (- 0.043 *  cos__q_RF_HAA__);
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(5,0) = ( 0.2045 *  sin__q_RF_HAA__);
    (*this)(5,1) = ( 0.2045 *  cos__q_RF_HAA__);
    (*this)(5,2) = ( 0.207 - ( 0.043 *  sin__q_RF_HAA__));
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,4) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::Type_fr_trunk_X_LH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(3,1) = (( 0.207 *  sin__q_LH_HAA__) -  0.043);
    (*this)(4,0) = (- 0.2045 *  cos__q_LH_HAA__);
    (*this)(4,1) = ( 0.2045 *  sin__q_LH_HAA__);
    (*this)(4,2) = ( 0.043 *  cos__q_LH_HAA__);
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(5,0) = ( 0.2045 *  sin__q_LH_HAA__);
    (*this)(5,1) = ( 0.2045 *  cos__q_LH_HAA__);
    (*this)(5,2) = ( 0.207 - ( 0.043 *  sin__q_LH_HAA__));
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,4) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::Type_fr_trunk_X_RH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(3,1) = ( 0.043 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(4,0) = (- 0.2045 *  cos__q_RH_HAA__);
    (*this)(4,1) = ( 0.2045 *  sin__q_RH_HAA__);
    (*this)(4,2) = (- 0.043 *  cos__q_RH_HAA__);
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(5,0) = (- 0.2045 *  sin__q_RH_HAA__);
    (*this)(5,1) = (- 0.2045 *  cos__q_RH_HAA__);
    (*this)(5,2) = ( 0.207 - ( 0.043 *  sin__q_RH_HAA__));
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,4) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(3,2) = (((( 0.026 *  sin__q_LF_HFE__) + ( 0.151 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,0) = (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.026 *  cos__q_LF_HAA__));
    (*this)(4,1) = ((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.151 *  cos__q_LF_HAA__));
    (*this)(4,2) = ((((- 0.151 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.026 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.026 *  sin__q_LF_HAA__));
    (*this)(5,1) = (((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.151 *  sin__q_LF_HAA__));
    (*this)(5,2) = ((((- 0.151 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.026 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(3,2) = (((( 0.026 *  sin__q_RF_HFE__) + ( 0.151 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,0) = (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.026 *  cos__q_RF_HAA__));
    (*this)(4,1) = ((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.151 *  cos__q_RF_HAA__));
    (*this)(4,2) = (((( 0.151 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.026 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.026 *  sin__q_RF_HAA__));
    (*this)(5,1) = ((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.151 *  sin__q_RF_HAA__));
    (*this)(5,2) = ((((- 0.151 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.026 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::Type_fr_trunk_X_LH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(3,2) = ((((- 0.026 *  sin__q_LH_HFE__) + ( 0.151 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,0) = (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.026 *  cos__q_LH_HAA__));
    (*this)(4,1) = (((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.151 *  cos__q_LH_HAA__));
    (*this)(4,2) = ((((- 0.151 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.026 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.026 *  sin__q_LH_HAA__));
    (*this)(5,1) = ((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.151 *  sin__q_LH_HAA__));
    (*this)(5,2) = ((((- 0.151 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.026 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::Type_fr_trunk_X_RH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(3,2) = ((((- 0.026 *  sin__q_RH_HFE__) + ( 0.151 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,0) = (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.026 *  cos__q_RH_HAA__));
    (*this)(4,1) = (((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.151 *  cos__q_RH_HAA__));
    (*this)(4,2) = (((( 0.151 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.026 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.026 *  sin__q_RH_HAA__));
    (*this)(5,1) = (((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.151 *  sin__q_RH_HAA__));
    (*this)(5,2) = ((((- 0.151 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.026 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,2) = ((((((- 0.125 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.125 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,0) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.125 *  cos__q_LF_HAA__));
    (*this)(4,2) = ((((((- 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(5,1) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.125 *  sin__q_LF_HAA__));
    (*this)(5,2) = ((((((- 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(3,2) = (((((((- 0.125 *  sin__q_RF_HFE__) - ( 0.001 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((( 0.125 *  cos__q_RF_HFE__) - ( 0.001 *  sin__q_RF_HFE__)) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,0) = (((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.001 *  cos__q_RF_HAA__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.125 *  cos__q_RF_HAA__));
    (*this)(4,2) = ((((((( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = (((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.001 *  sin__q_RF_HAA__));
    (*this)(5,1) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.125 *  sin__q_RF_HAA__));
    (*this)(5,2) = ((((((( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((((- 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::Type_fr_trunk_X_LH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,2) = ((((((( 0.001 *  cos__q_LH_HFE__) - ( 0.125 *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((( 0.001 *  sin__q_LH_HFE__) + ( 0.125 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,0) = ((((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + ( 0.001 *  cos__q_LH_HAA__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.125 *  cos__q_LH_HAA__));
    (*this)(4,2) = (((((((- 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = (((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - ( 0.001 *  sin__q_LH_HAA__));
    (*this)(5,1) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.125 *  sin__q_LH_HAA__));
    (*this)(5,2) = (((((((- 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::Type_fr_trunk_X_RH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(3,2) = ((((((( 0.001 *  cos__q_RH_HFE__) - ( 0.125 *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((( 0.001 *  sin__q_RH_HFE__) + ( 0.125 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.001 *  cos__q_RH_HAA__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.125 *  cos__q_RH_HAA__));
    (*this)(4,2) = ((((((( 0.001 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.125 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.125 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.001 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = ((((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.001 *  sin__q_RH_HAA__));
    (*this)(5,1) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.125 *  sin__q_RH_HAA__));
    (*this)(5,2) = (((((((- 0.001 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.125 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.001 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.125 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::Type_fr_LF_foot_X_fr_LF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.33;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.33;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::Type_fr_RF_foot_X_fr_RF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.33;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.33;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::Type_fr_LH_foot_X_fr_LH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.33;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.33;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::Type_fr_RH_foot_X_fr_RH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.33;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.33;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::Type_fr_trunk_X_fr_LF_foot()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,2) = ((((((- 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,0) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.33 *  cos__q_LF_HAA__));
    (*this)(4,2) = ((((((- 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(5,1) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.33 *  sin__q_LF_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::Type_fr_trunk_X_fr_RF_foot()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(3,2) = ((((((- 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  cos__q_RF_HAA__));
    (*this)(4,2) = (((((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(5,1) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  sin__q_RF_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::Type_fr_trunk_X_fr_LH_foot()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,1) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,2) = ((((((- 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,0) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.33 *  cos__q_LH_HAA__));
    (*this)(4,2) = ((((((- 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(5,1) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.33 *  sin__q_LH_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::Type_fr_trunk_X_fr_RH_foot()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,1) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(3,2) = ((((((- 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,0) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  cos__q_RH_HAA__));
    (*this)(4,2) = (((((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(5,1) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  sin__q_RH_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::Type_fr_LF_foot_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,1) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(3,2) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,5) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,0) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.33 *  cos__q_LF_HAA__));
    (*this)(4,2) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.33 *  sin__q_LF_HAA__));
    (*this)(4,3) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(5,1) = ((((((- 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::Type_fr_RF_foot_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,1) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(3,2) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,5) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,1) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  cos__q_RF_HAA__));
    (*this)(4,2) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  sin__q_RF_HAA__));
    (*this)(4,3) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(5,1) = (((((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::Type_fr_LH_foot_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,1) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(3,2) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,5) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,0) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.33 *  cos__q_LH_HAA__));
    (*this)(4,2) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.33 *  sin__q_LH_HAA__));
    (*this)(4,3) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(5,1) = ((((((- 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::Type_fr_RH_foot_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,1) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(3,2) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,5) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,1) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  cos__q_RH_HAA__));
    (*this)(4,2) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  sin__q_RH_HAA__));
    (*this)(4,3) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,0) = ((((((- 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(5,1) = (((((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(5,2) = ((((((- 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::Type_fr_trunk_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = - 0.207;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0.3735;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.3735;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::Type_fr_trunk_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0.207;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.3735;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.3735;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::Type_fr_trunk_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = - 0.207;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = - 0.3735;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.3735;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::Type_fr_trunk_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0.207;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.3735;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.3735;
    (*this)(5,2) = 0.207;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::Type_fr_trunk_X_fr_LF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,4) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(4,0) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(4,1) = ( 0.08 *  cos__q_LF_HAA__);
    (*this)(4,2) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(5,1) = ( 0.207 - ( 0.08 *  sin__q_LF_HAA__));
    (*this)(5,2) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::Type_fr_trunk_X_fr_RF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,4) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(4,0) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(4,1) = ( 0.08 *  cos__q_RF_HAA__);
    (*this)(4,2) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(5,1) = (( 0.08 *  sin__q_RF_HAA__) -  0.207);
    (*this)(5,2) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::Type_fr_trunk_X_fr_LH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,4) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(4,0) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(4,1) = ( 0.08 *  cos__q_LH_HAA__);
    (*this)(4,2) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(5,1) = ( 0.207 - ( 0.08 *  sin__q_LH_HAA__));
    (*this)(5,2) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::Type_fr_trunk_X_fr_RH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,4) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(3,2) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(4,0) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(4,1) = ( 0.08 *  cos__q_RH_HAA__);
    (*this)(4,2) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(5,1) = (( 0.08 *  sin__q_RH_HAA__) -  0.207);
    (*this)(5,2) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::Type_fr_trunk_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(3,2) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(4,1) = ((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__));
    (*this)(4,2) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,0) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(5,1) = (((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__));
    (*this)(5,2) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::Type_fr_trunk_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(3,2) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(4,1) = ((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__));
    (*this)(4,2) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,0) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(5,1) = ((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__));
    (*this)(5,2) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::Type_fr_trunk_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,0) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(3,1) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(3,2) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(4,1) = (((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__));
    (*this)(4,2) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,0) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(5,1) = ((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__));
    (*this)(5,2) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::Type_fr_trunk_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,0) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(3,1) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(3,2) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,0) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(4,1) = (((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__));
    (*this)(4,2) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,0) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(5,1) = (((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__));
    (*this)(5,2) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.08;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,2) =  sin__q_LF_HFE__;
    (*this)(1,0) = - sin__q_LF_HFE__;
    (*this)(1,2) =  cos__q_LF_HFE__;
    (*this)(3,1) = (- 0.08 *  sin__q_LF_HFE__);
    (*this)(3,3) =  cos__q_LF_HFE__;
    (*this)(3,5) =  sin__q_LF_HFE__;
    (*this)(4,1) = (- 0.08 *  cos__q_LF_HFE__);
    (*this)(4,3) = - sin__q_LF_HFE__;
    (*this)(4,5) =  cos__q_LF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.08;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,1) = - sin__q_LF_HFE__;
    (*this)(2,0) =  sin__q_LF_HFE__;
    (*this)(2,1) =  cos__q_LF_HFE__;
    (*this)(3,3) =  cos__q_LF_HFE__;
    (*this)(3,4) = - sin__q_LF_HFE__;
    (*this)(4,0) = (- 0.08 *  sin__q_LF_HFE__);
    (*this)(4,1) = (- 0.08 *  cos__q_LF_HFE__);
    (*this)(5,3) =  sin__q_LF_HFE__;
    (*this)(5,4) =  cos__q_LF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.35;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) =  sin__q_LF_KFE__;
    (*this)(1,0) = - sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    (*this)(3,2) = ( 0.35 *  sin__q_LF_KFE__);
    (*this)(3,3) =  cos__q_LF_KFE__;
    (*this)(3,4) =  sin__q_LF_KFE__;
    (*this)(4,2) = ( 0.35 *  cos__q_LF_KFE__);
    (*this)(4,3) = - sin__q_LF_KFE__;
    (*this)(4,4) =  cos__q_LF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.35;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) = - sin__q_LF_KFE__;
    (*this)(1,0) =  sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    (*this)(3,3) =  cos__q_LF_KFE__;
    (*this)(3,4) = - sin__q_LF_KFE__;
    (*this)(4,3) =  sin__q_LF_KFE__;
    (*this)(4,4) =  cos__q_LF_KFE__;
    (*this)(5,0) = ( 0.35 *  sin__q_LF_KFE__);
    (*this)(5,1) = ( 0.35 *  cos__q_LF_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.08;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,2) = - sin__q_RF_HFE__;
    (*this)(1,0) = - sin__q_RF_HFE__;
    (*this)(1,2) = - cos__q_RF_HFE__;
    (*this)(3,1) = ( 0.08 *  sin__q_RF_HFE__);
    (*this)(3,3) =  cos__q_RF_HFE__;
    (*this)(3,5) = - sin__q_RF_HFE__;
    (*this)(4,1) = ( 0.08 *  cos__q_RF_HFE__);
    (*this)(4,3) = - sin__q_RF_HFE__;
    (*this)(4,5) = - cos__q_RF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.08;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,1) = - sin__q_RF_HFE__;
    (*this)(2,0) = - sin__q_RF_HFE__;
    (*this)(2,1) = - cos__q_RF_HFE__;
    (*this)(3,3) =  cos__q_RF_HFE__;
    (*this)(3,4) = - sin__q_RF_HFE__;
    (*this)(4,0) = ( 0.08 *  sin__q_RF_HFE__);
    (*this)(4,1) = ( 0.08 *  cos__q_RF_HFE__);
    (*this)(5,3) = - sin__q_RF_HFE__;
    (*this)(5,4) = - cos__q_RF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.35;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) =  sin__q_RF_KFE__;
    (*this)(1,0) = - sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    (*this)(3,2) = ( 0.35 *  sin__q_RF_KFE__);
    (*this)(3,3) =  cos__q_RF_KFE__;
    (*this)(3,4) =  sin__q_RF_KFE__;
    (*this)(4,2) = ( 0.35 *  cos__q_RF_KFE__);
    (*this)(4,3) = - sin__q_RF_KFE__;
    (*this)(4,4) =  cos__q_RF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.35;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) = - sin__q_RF_KFE__;
    (*this)(1,0) =  sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    (*this)(3,3) =  cos__q_RF_KFE__;
    (*this)(3,4) = - sin__q_RF_KFE__;
    (*this)(4,3) =  sin__q_RF_KFE__;
    (*this)(4,4) =  cos__q_RF_KFE__;
    (*this)(5,0) = ( 0.35 *  sin__q_RF_KFE__);
    (*this)(5,1) = ( 0.35 *  cos__q_RF_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::Type_fr_LH_upperleg_X_fr_LH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.08;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,2) =  sin__q_LH_HFE__;
    (*this)(1,0) = - sin__q_LH_HFE__;
    (*this)(1,2) =  cos__q_LH_HFE__;
    (*this)(3,1) = (- 0.08 *  sin__q_LH_HFE__);
    (*this)(3,3) =  cos__q_LH_HFE__;
    (*this)(3,5) =  sin__q_LH_HFE__;
    (*this)(4,1) = (- 0.08 *  cos__q_LH_HFE__);
    (*this)(4,3) = - sin__q_LH_HFE__;
    (*this)(4,5) =  cos__q_LH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::Type_fr_LH_hipassembly_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.08;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,1) = - sin__q_LH_HFE__;
    (*this)(2,0) =  sin__q_LH_HFE__;
    (*this)(2,1) =  cos__q_LH_HFE__;
    (*this)(3,3) =  cos__q_LH_HFE__;
    (*this)(3,4) = - sin__q_LH_HFE__;
    (*this)(4,0) = (- 0.08 *  sin__q_LH_HFE__);
    (*this)(4,1) = (- 0.08 *  cos__q_LH_HFE__);
    (*this)(5,3) =  sin__q_LH_HFE__;
    (*this)(5,4) =  cos__q_LH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.35;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) =  sin__q_LH_KFE__;
    (*this)(1,0) = - sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    (*this)(3,2) = ( 0.35 *  sin__q_LH_KFE__);
    (*this)(3,3) =  cos__q_LH_KFE__;
    (*this)(3,4) =  sin__q_LH_KFE__;
    (*this)(4,2) = ( 0.35 *  cos__q_LH_KFE__);
    (*this)(4,3) = - sin__q_LH_KFE__;
    (*this)(4,4) =  cos__q_LH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.35;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) = - sin__q_LH_KFE__;
    (*this)(1,0) =  sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    (*this)(3,3) =  cos__q_LH_KFE__;
    (*this)(3,4) = - sin__q_LH_KFE__;
    (*this)(4,3) =  sin__q_LH_KFE__;
    (*this)(4,4) =  cos__q_LH_KFE__;
    (*this)(5,0) = ( 0.35 *  sin__q_LH_KFE__);
    (*this)(5,1) = ( 0.35 *  cos__q_LH_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::Type_fr_RH_upperleg_X_fr_RH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.08;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,2) = - sin__q_RH_HFE__;
    (*this)(1,0) = - sin__q_RH_HFE__;
    (*this)(1,2) = - cos__q_RH_HFE__;
    (*this)(3,1) = ( 0.08 *  sin__q_RH_HFE__);
    (*this)(3,3) =  cos__q_RH_HFE__;
    (*this)(3,5) = - sin__q_RH_HFE__;
    (*this)(4,1) = ( 0.08 *  cos__q_RH_HFE__);
    (*this)(4,3) = - sin__q_RH_HFE__;
    (*this)(4,5) = - cos__q_RH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::Type_fr_RH_hipassembly_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.08;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,1) = - sin__q_RH_HFE__;
    (*this)(2,0) = - sin__q_RH_HFE__;
    (*this)(2,1) = - cos__q_RH_HFE__;
    (*this)(3,3) =  cos__q_RH_HFE__;
    (*this)(3,4) = - sin__q_RH_HFE__;
    (*this)(4,0) = ( 0.08 *  sin__q_RH_HFE__);
    (*this)(4,1) = ( 0.08 *  cos__q_RH_HFE__);
    (*this)(5,3) = - sin__q_RH_HFE__;
    (*this)(5,4) = - cos__q_RH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.35;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) =  sin__q_RH_KFE__;
    (*this)(1,0) = - sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    (*this)(3,2) = ( 0.35 *  sin__q_RH_KFE__);
    (*this)(3,3) =  cos__q_RH_KFE__;
    (*this)(3,4) =  sin__q_RH_KFE__;
    (*this)(4,2) = ( 0.35 *  cos__q_RH_KFE__);
    (*this)(4,3) = - sin__q_RH_KFE__;
    (*this)(4,4) =  cos__q_RH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.35;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& iit::TestHyQ::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) = - sin__q_RH_KFE__;
    (*this)(1,0) =  sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    (*this)(3,3) =  cos__q_RH_KFE__;
    (*this)(3,4) = - sin__q_RH_KFE__;
    (*this)(4,3) =  sin__q_RH_KFE__;
    (*this)(4,4) =  cos__q_RH_KFE__;
    (*this)(5,0) = ( 0.35 *  sin__q_RH_KFE__);
    (*this)(5,1) = ( 0.35 *  cos__q_RH_KFE__);
    return *this;
}

template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(0,4) = ( 0.207 *  sin__q_LF_HAA__);
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(1,4) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,4) = (- 0.3735 *  cos__q_LF_HAA__);
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,4) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(0,4) = (- 0.207 *  sin__q_RF_HAA__);
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,3) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(1,4) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,4) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,4) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::Type_fr_trunk_X_fr_LH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(0,4) = ( 0.207 *  sin__q_LH_HAA__);
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,3) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(1,4) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    (*this)(2,3) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,4) = ( 0.3735 *  cos__q_LH_HAA__);
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,4) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::Type_fr_trunk_X_fr_RH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(0,4) = (- 0.207 *  sin__q_RH_HAA__);
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,3) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(1,4) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,4) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,4) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::Type_fr_trunk_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(1,4) = ((( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(1,5) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(2,4) = ((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__));
    (*this)(2,5) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::Type_fr_trunk_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(1,4) = ((( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(1,5) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(2,4) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(2,5) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::Type_fr_trunk_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(1,4) = ((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(1,5) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(2,4) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(2,5) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::Type_fr_trunk_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(1,4) = ((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(1,5) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(2,4) = ((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__));
    (*this)(2,5) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::Type_fr_trunk_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,5) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(1,4) = (((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(1,5) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(2,4) = (((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(2,5) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::Type_fr_trunk_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,5) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(1,4) = (((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(1,5) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(2,4) = ((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(2,5) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::Type_fr_trunk_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,5) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(1,4) = ((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(1,5) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(2,4) = (((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(2,5) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::Type_fr_trunk_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,5) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(1,4) = ((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(1,5) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(2,4) = ((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(2,5) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) = - sin__q_LF_HAA__;
    (*this)(0,2) = - cos__q_LF_HAA__;
    (*this)(0,3) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(0,4) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(0,5) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,2) =  sin__q_LF_HAA__;
    (*this)(1,3) = ( 0.207 *  sin__q_LF_HAA__);
    (*this)(1,4) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(1,5) = (- 0.3735 *  cos__q_LF_HAA__);
    (*this)(3,4) = - sin__q_LF_HAA__;
    (*this)(3,5) = - cos__q_LF_HAA__;
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(4,5) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  sin__q_RF_HAA__;
    (*this)(0,2) = - cos__q_RF_HAA__;
    (*this)(0,3) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(0,4) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(0,5) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  sin__q_RF_HAA__;
    (*this)(1,3) = (- 0.207 *  sin__q_RF_HAA__);
    (*this)(1,4) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(1,5) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(3,4) =  sin__q_RF_HAA__;
    (*this)(3,5) = - cos__q_RF_HAA__;
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(4,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::Type_fr_LH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) = - sin__q_LH_HAA__;
    (*this)(0,2) = - cos__q_LH_HAA__;
    (*this)(0,3) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(0,4) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(0,5) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,2) =  sin__q_LH_HAA__;
    (*this)(1,3) = ( 0.207 *  sin__q_LH_HAA__);
    (*this)(1,4) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(1,5) = ( 0.3735 *  cos__q_LH_HAA__);
    (*this)(3,4) = - sin__q_LH_HAA__;
    (*this)(3,5) = - cos__q_LH_HAA__;
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(4,5) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::Type_fr_RH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  sin__q_RH_HAA__;
    (*this)(0,2) = - cos__q_RH_HAA__;
    (*this)(0,3) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(0,4) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(0,5) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  sin__q_RH_HAA__;
    (*this)(1,3) = (- 0.207 *  sin__q_RH_HAA__);
    (*this)(1,4) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(1,5) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(3,4) =  sin__q_RH_HAA__;
    (*this)(3,5) = - cos__q_RH_HAA__;
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(4,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(0,2) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(0,3) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(0,4) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(0,5) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(1,0) = - cos__q_LF_HFE__;
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,3) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(1,4) = ((( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(1,5) = ((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(2,4) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,5) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(3,5) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,3) = - cos__q_LF_HFE__;
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(0,2) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(0,3) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(0,4) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(0,5) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(1,0) = - cos__q_RF_HFE__;
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,3) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(1,4) = ((( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(1,5) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(2,4) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,5) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(3,5) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,3) = - cos__q_RF_HFE__;
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::Type_fr_LH_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(0,2) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(0,3) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(0,4) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(0,5) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(1,0) = - cos__q_LH_HFE__;
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,3) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(1,4) = ((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(1,5) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(2,4) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,5) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(3,5) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,3) = - cos__q_LH_HFE__;
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::Type_fr_RH_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(0,2) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(0,3) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(0,4) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(0,5) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(1,0) = - cos__q_RH_HFE__;
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,3) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(1,4) = ((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(1,5) = ((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(2,4) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,5) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(3,5) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,3) = - cos__q_RH_HFE__;
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,4) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(0,5) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,3) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,4) = (((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(1,5) = (((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(2,4) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(2,5) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,5) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,3) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,4) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(0,5) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,3) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,4) = (((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(1,5) = ((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(2,4) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,5) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,5) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,3) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::Type_fr_LH_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,4) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(0,5) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,3) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,4) = ((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(1,5) = (((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(2,4) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,5) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,5) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,3) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::Type_fr_RH_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,4) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(0,5) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,3) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,4) = ((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(1,5) = ((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(2,4) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,5) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,5) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,3) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(0,4) = (( 0.207 *  sin__q_LF_HAA__) -  0.043);
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.2045 *  cos__q_LF_HAA__);
    (*this)(1,4) = (- 0.2045 *  sin__q_LF_HAA__);
    (*this)(1,5) = ( 0.043 *  cos__q_LF_HAA__);
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.2045 *  sin__q_LF_HAA__);
    (*this)(2,4) = (- 0.2045 *  cos__q_LF_HAA__);
    (*this)(2,5) = ( 0.207 - ( 0.043 *  sin__q_LF_HAA__));
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,4) = - cos__q_LF_HAA__;
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,4) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(0,4) = ( 0.043 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,3) = ( 0.2045 *  cos__q_RF_HAA__);
    (*this)(1,4) = (- 0.2045 *  sin__q_RF_HAA__);
    (*this)(1,5) = (- 0.043 *  cos__q_RF_HAA__);
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.2045 *  sin__q_RF_HAA__);
    (*this)(2,4) = ( 0.2045 *  cos__q_RF_HAA__);
    (*this)(2,5) = ( 0.207 - ( 0.043 *  sin__q_RF_HAA__));
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,4) =  cos__q_RF_HAA__;
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,4) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::Type_fr_trunk_X_LH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(0,4) = (( 0.207 *  sin__q_LH_HAA__) -  0.043);
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,3) = (- 0.2045 *  cos__q_LH_HAA__);
    (*this)(1,4) = ( 0.2045 *  sin__q_LH_HAA__);
    (*this)(1,5) = ( 0.043 *  cos__q_LH_HAA__);
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    (*this)(2,3) = ( 0.2045 *  sin__q_LH_HAA__);
    (*this)(2,4) = ( 0.2045 *  cos__q_LH_HAA__);
    (*this)(2,5) = ( 0.207 - ( 0.043 *  sin__q_LH_HAA__));
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,4) = - cos__q_LH_HAA__;
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,4) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::Type_fr_trunk_X_RH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(0,4) = ( 0.043 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,3) = (- 0.2045 *  cos__q_RH_HAA__);
    (*this)(1,4) = ( 0.2045 *  sin__q_RH_HAA__);
    (*this)(1,5) = (- 0.043 *  cos__q_RH_HAA__);
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.2045 *  sin__q_RH_HAA__);
    (*this)(2,4) = (- 0.2045 *  cos__q_RH_HAA__);
    (*this)(2,5) = ( 0.207 - ( 0.043 *  sin__q_RH_HAA__));
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,4) =  cos__q_RH_HAA__;
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,4) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(0,5) = (((( 0.026 *  sin__q_LF_HFE__) + ( 0.151 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.026 *  cos__q_LF_HAA__));
    (*this)(1,4) = ((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.151 *  cos__q_LF_HAA__));
    (*this)(1,5) = ((((- 0.151 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.026 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.026 *  sin__q_LF_HAA__));
    (*this)(2,4) = (((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.151 *  sin__q_LF_HAA__));
    (*this)(2,5) = ((((- 0.151 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.026 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(0,5) = (((( 0.026 *  sin__q_RF_HFE__) + ( 0.151 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.026 *  cos__q_RF_HAA__));
    (*this)(1,4) = ((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.151 *  cos__q_RF_HAA__));
    (*this)(1,5) = (((( 0.151 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.026 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.026 *  sin__q_RF_HAA__));
    (*this)(2,4) = ((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.151 *  sin__q_RF_HAA__));
    (*this)(2,5) = ((((- 0.151 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.026 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::Type_fr_trunk_X_LH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(0,5) = ((((- 0.026 *  sin__q_LH_HFE__) + ( 0.151 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.026 *  cos__q_LH_HAA__));
    (*this)(1,4) = (((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.151 *  cos__q_LH_HAA__));
    (*this)(1,5) = ((((- 0.151 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.026 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.026 *  sin__q_LH_HAA__));
    (*this)(2,4) = ((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.151 *  sin__q_LH_HAA__));
    (*this)(2,5) = ((((- 0.151 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.026 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::Type_fr_trunk_X_RH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(0,5) = ((((- 0.026 *  sin__q_RH_HFE__) + ( 0.151 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.026 *  cos__q_RH_HAA__));
    (*this)(1,4) = (((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.151 *  cos__q_RH_HAA__));
    (*this)(1,5) = (((( 0.151 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.026 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.026 *  sin__q_RH_HAA__));
    (*this)(2,4) = (((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.151 *  sin__q_RH_HAA__));
    (*this)(2,5) = ((((- 0.151 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.026 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,5) = ((((((- 0.125 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.125 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.125 *  cos__q_LF_HAA__));
    (*this)(1,5) = ((((((- 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(2,4) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.125 *  sin__q_LF_HAA__));
    (*this)(2,5) = ((((((- 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,5) = (((((((- 0.125 *  sin__q_RF_HFE__) - ( 0.001 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((( 0.125 *  cos__q_RF_HFE__) - ( 0.001 *  sin__q_RF_HFE__)) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.001 *  cos__q_RF_HAA__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.125 *  cos__q_RF_HAA__));
    (*this)(1,5) = ((((((( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.001 *  sin__q_RF_HAA__));
    (*this)(2,4) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.125 *  sin__q_RF_HAA__));
    (*this)(2,5) = ((((((( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + ((((- 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::Type_fr_trunk_X_LH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,5) = ((((((( 0.001 *  cos__q_LH_HFE__) - ( 0.125 *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((( 0.001 *  sin__q_LH_HFE__) + ( 0.125 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) + ( 0.001 *  cos__q_LH_HAA__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.125 *  cos__q_LH_HAA__));
    (*this)(1,5) = (((((((- 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - ( 0.001 *  sin__q_LH_HAA__));
    (*this)(2,4) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.125 *  sin__q_LH_HAA__));
    (*this)(2,5) = (((((((- 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::Type_fr_trunk_X_RH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,5) = ((((((( 0.001 *  cos__q_RH_HFE__) - ( 0.125 *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((( 0.001 *  sin__q_RH_HFE__) + ( 0.125 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = ((((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.001 *  cos__q_RH_HAA__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.125 *  cos__q_RH_HAA__));
    (*this)(1,5) = ((((((( 0.001 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.125 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.125 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.001 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ((((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + ( 0.001 *  sin__q_RH_HAA__));
    (*this)(2,4) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.125 *  sin__q_RH_HAA__));
    (*this)(2,5) = (((((((- 0.001 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.125 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.001 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.125 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::Type_fr_LF_foot_X_fr_LF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.33;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.33;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::Type_fr_RF_foot_X_fr_RF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.33;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.33;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::Type_fr_LH_foot_X_fr_LH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.33;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.33;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::Type_fr_RH_foot_X_fr_RH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.33;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.33;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::Type_fr_trunk_X_fr_LF_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,5) = ((((((- 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.33 *  cos__q_LF_HAA__));
    (*this)(1,5) = ((((((- 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(2,4) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.33 *  sin__q_LF_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,3) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::Type_fr_trunk_X_fr_RF_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,5) = ((((((- 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  cos__q_RF_HAA__));
    (*this)(1,5) = (((((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(2,4) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  sin__q_RF_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,3) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::Type_fr_trunk_X_fr_LH_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,4) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,5) = ((((((- 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.33 *  cos__q_LH_HAA__));
    (*this)(1,5) = ((((((- 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(2,4) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.33 *  sin__q_LH_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,3) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::Type_fr_trunk_X_fr_RH_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,4) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,5) = ((((((- 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  cos__q_RH_HAA__));
    (*this)(1,5) = (((((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(2,4) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  sin__q_RH_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,3) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::Type_fr_LF_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,4) = ((((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(0,5) = (((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  sin__q_LF_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,3) = (((( 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__)) *  cos__q_LF_KFE__)) + ( 0.33 *  cos__q_LF_HAA__));
    (*this)(1,5) = ((((((( 0.08 *  sin__q_LF_HAA__) -  0.207) *  sin__q_LF_HFE__) + (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + ((((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__)) *  cos__q_LF_KFE__)) - ( 0.33 *  sin__q_LF_HAA__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ((((((- 0.33 *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + (( 0.33 *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + ( 0.35 *  cos__q_LF_HFE__)) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(2,4) = ((((((- 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  sin__q_LF_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__)) + ( 0.3735 *  cos__q_LF_HAA__));
    (*this)(3,3) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(3,4) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(3,5) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,3) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(4,4) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,5) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,4) =  cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::Type_fr_RF_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,4) = ((((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  sin__q_RF_KFE__) + (((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(0,5) = ((((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  sin__q_RF_KFE__) + ((((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,3) = ((((- 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,4) = ((((((- 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  cos__q_RF_HAA__));
    (*this)(1,5) = (((((( 0.207 - ( 0.08 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) - (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__)) *  cos__q_RF_KFE__)) + ( 0.33 *  sin__q_RF_HAA__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((((- 0.33 *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + (( 0.33 *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + ( 0.35 *  cos__q_RF_HFE__)) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(2,4) = (((((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__)) + ( 0.3735 *  cos__q_RF_HAA__));
    (*this)(3,3) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(3,4) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(3,5) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,3) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(4,4) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,5) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,4) =  cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::Type_fr_LH_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,4) = (((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(0,5) = ((((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  sin__q_LH_KFE__) + (((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,3) = (((( 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - (( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__)) *  cos__q_LH_KFE__)) + ( 0.33 *  cos__q_LH_HAA__));
    (*this)(1,5) = ((((((( 0.08 *  sin__q_LH_HAA__) -  0.207) *  sin__q_LH_HFE__) - (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + (((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__)) *  cos__q_LH_KFE__)) - ( 0.33 *  sin__q_LH_HAA__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((((((- 0.33 *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + (( 0.33 *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + ( 0.35 *  cos__q_LH_HFE__)) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(2,4) = ((((((- 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__)) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(3,4) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(3,5) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,3) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(4,4) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,5) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,4) =  cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::Type_fr_RH_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,3) = (((( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,4) = (((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  sin__q_RH_KFE__) + (((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(0,5) = (((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  sin__q_RH_KFE__) + ((((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,3) = ((((- 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,4) = (((((( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  cos__q_RH_HAA__));
    (*this)(1,5) = (((((( 0.207 - ( 0.08 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) + (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__)) *  cos__q_RH_KFE__)) + ( 0.33 *  sin__q_RH_HAA__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ((((((- 0.33 *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + (( 0.33 *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + ( 0.35 *  cos__q_RH_HFE__)) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(2,4) = (((((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,5) = ((((((- 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__)) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(3,4) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(3,5) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,3) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(4,4) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,5) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,4) =  cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::Type_fr_trunk_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = - 0.207;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.3735;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.3735;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::Type_fr_trunk_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.207;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.3735;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.3735;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::Type_fr_trunk_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = - 0.207;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.3735;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.3735;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::Type_fr_trunk_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.207;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.3735;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.3735;
    (*this)(2,5) = 0.207;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::Type_fr_trunk_X_fr_LF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LF_HAA__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_LF_HAA__));
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(1,4) = ( 0.08 *  cos__q_LF_HAA__);
    (*this)(1,5) = ( 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.3735 *  sin__q_LF_HAA__);
    (*this)(2,4) = ( 0.207 - ( 0.08 *  sin__q_LF_HAA__));
    (*this)(2,5) = ( 0.3735 *  cos__q_LF_HAA__);
    (*this)(4,3) = - sin__q_LF_HAA__;
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = - cos__q_LF_HAA__;
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::Type_fr_trunk_X_fr_RF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RF_HAA__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_RF_HAA__));
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(1,4) = ( 0.08 *  cos__q_RF_HAA__);
    (*this)(1,5) = (- 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.3735 *  sin__q_RF_HAA__);
    (*this)(2,4) = (( 0.08 *  sin__q_RF_HAA__) -  0.207);
    (*this)(2,5) = ( 0.3735 *  cos__q_RF_HAA__);
    (*this)(4,3) =  sin__q_RF_HAA__;
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = - cos__q_RF_HAA__;
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::Type_fr_trunk_X_fr_LH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.207 *  cos__q_LH_HAA__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_LH_HAA__));
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(1,4) = ( 0.08 *  cos__q_LH_HAA__);
    (*this)(1,5) = (- 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ( 0.3735 *  sin__q_LH_HAA__);
    (*this)(2,4) = ( 0.207 - ( 0.08 *  sin__q_LH_HAA__));
    (*this)(2,5) = (- 0.3735 *  cos__q_LH_HAA__);
    (*this)(4,3) = - sin__q_LH_HAA__;
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = - cos__q_LH_HAA__;
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::Type_fr_trunk_X_fr_RH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.207 *  cos__q_RH_HAA__);
    (*this)(0,5) = ( 0.08 - ( 0.207 *  sin__q_RH_HAA__));
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(1,4) = ( 0.08 *  cos__q_RH_HAA__);
    (*this)(1,5) = ( 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.3735 *  sin__q_RH_HAA__);
    (*this)(2,4) = (( 0.08 *  sin__q_RH_HAA__) -  0.207);
    (*this)(2,5) = (- 0.3735 *  cos__q_RH_HAA__);
    (*this)(4,3) =  sin__q_RH_HAA__;
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = - cos__q_RH_HAA__;
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::Type_fr_trunk_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__);
    (*this)(0,5) = ((( 0.35 *  cos__q_LF_HFE__) - ( 0.207 *  sin__q_LF_HAA__)) +  0.08);
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.3735 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(1,4) = ((((- 0.3735 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.08 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) + ( 0.35 *  cos__q_LF_HAA__));
    (*this)(1,5) = (( 0.3735 *  sin__q_LF_HAA__) - (( 0.35 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ((( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) - (( 0.3735 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__));
    (*this)(2,4) = (((( 0.3735 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) - ( 0.35 *  sin__q_LF_HAA__));
    (*this)(2,5) = (( 0.3735 *  cos__q_LF_HAA__) - (( 0.35 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(3,3) = - sin__q_LF_HFE__;
    (*this)(3,4) = - cos__q_LF_HFE__;
    (*this)(4,3) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(4,4) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(4,5) =  cos__q_LF_HAA__;
    (*this)(5,3) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(5,4) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(5,5) = - sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::Type_fr_trunk_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__);
    (*this)(0,5) = ((( 0.35 *  cos__q_RF_HFE__) - ( 0.207 *  sin__q_RF_HAA__)) +  0.08);
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.3735 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(1,4) = ((((- 0.3735 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.08 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.35 *  cos__q_RF_HAA__));
    (*this)(1,5) = ((( 0.35 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - ( 0.3735 *  sin__q_RF_HAA__));
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  sin__q_RF_HFE__) + (( 0.3735 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__));
    (*this)(2,4) = ((((- 0.3735 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + ((( 0.08 *  sin__q_RF_HAA__) -  0.207) *  cos__q_RF_HFE__)) + ( 0.35 *  sin__q_RF_HAA__));
    (*this)(2,5) = (( 0.3735 *  cos__q_RF_HAA__) - (( 0.35 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(3,3) = - sin__q_RF_HFE__;
    (*this)(3,4) = - cos__q_RF_HFE__;
    (*this)(4,3) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(4,4) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(4,5) =  cos__q_RF_HAA__;
    (*this)(5,3) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(5,4) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(5,5) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::Type_fr_trunk_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(0,3) = ((- 0.207 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__);
    (*this)(0,4) = (( 0.207 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__);
    (*this)(0,5) = ((( 0.35 *  cos__q_LH_HFE__) - ( 0.207 *  sin__q_LH_HAA__)) +  0.08);
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.3735 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(1,4) = (((( 0.3735 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.08 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) + ( 0.35 *  cos__q_LH_HAA__));
    (*this)(1,5) = (((- 0.35 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HAA__));
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) + (( 0.3735 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__));
    (*this)(2,4) = ((((- 0.3735 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) + (( 0.207 - ( 0.08 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) - ( 0.35 *  sin__q_LH_HAA__));
    (*this)(2,5) = (((- 0.35 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HAA__));
    (*this)(3,3) = - sin__q_LH_HFE__;
    (*this)(3,4) = - cos__q_LH_HFE__;
    (*this)(4,3) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(4,4) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(4,5) =  cos__q_LH_HAA__;
    (*this)(5,3) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(5,4) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(5,5) = - sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::Type_fr_trunk_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(0,3) = (( 0.207 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__);
    (*this)(0,4) = ((- 0.207 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__);
    (*this)(0,5) = ((( 0.35 *  cos__q_RH_HFE__) - ( 0.207 *  sin__q_RH_HAA__)) +  0.08);
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = ((( 0.08 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.3735 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(1,4) = (((( 0.3735 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.08 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.35 *  cos__q_RH_HAA__));
    (*this)(1,5) = ((( 0.35 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ( 0.3735 *  sin__q_RH_HAA__));
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  sin__q_RH_HFE__) - (( 0.3735 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__));
    (*this)(2,4) = (((( 0.3735 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + ((( 0.08 *  sin__q_RH_HAA__) -  0.207) *  cos__q_RH_HFE__)) + ( 0.35 *  sin__q_RH_HAA__));
    (*this)(2,5) = (((- 0.35 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HAA__));
    (*this)(3,3) = - sin__q_RH_HFE__;
    (*this)(3,4) = - cos__q_RH_HFE__;
    (*this)(4,3) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(4,4) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(4,5) =  cos__q_RH_HAA__;
    (*this)(5,3) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(5,4) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(5,5) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,2) =  sin__q_LF_HFE__;
    (*this)(0,4) = (- 0.08 *  sin__q_LF_HFE__);
    (*this)(1,0) = - sin__q_LF_HFE__;
    (*this)(1,2) =  cos__q_LF_HFE__;
    (*this)(1,4) = (- 0.08 *  cos__q_LF_HFE__);
    (*this)(3,3) =  cos__q_LF_HFE__;
    (*this)(3,5) =  sin__q_LF_HFE__;
    (*this)(4,3) = - sin__q_LF_HFE__;
    (*this)(4,5) =  cos__q_LF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,1) = - sin__q_LF_HFE__;
    (*this)(1,3) = (- 0.08 *  sin__q_LF_HFE__);
    (*this)(1,4) = (- 0.08 *  cos__q_LF_HFE__);
    (*this)(2,0) =  sin__q_LF_HFE__;
    (*this)(2,1) =  cos__q_LF_HFE__;
    (*this)(3,3) =  cos__q_LF_HFE__;
    (*this)(3,4) = - sin__q_LF_HFE__;
    (*this)(5,3) =  sin__q_LF_HFE__;
    (*this)(5,4) =  cos__q_LF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.35;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) =  sin__q_LF_KFE__;
    (*this)(0,5) = ( 0.35 *  sin__q_LF_KFE__);
    (*this)(1,0) = - sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    (*this)(1,5) = ( 0.35 *  cos__q_LF_KFE__);
    (*this)(3,3) =  cos__q_LF_KFE__;
    (*this)(3,4) =  sin__q_LF_KFE__;
    (*this)(4,3) = - sin__q_LF_KFE__;
    (*this)(4,4) =  cos__q_LF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.35;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) = - sin__q_LF_KFE__;
    (*this)(1,0) =  sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    (*this)(2,3) = ( 0.35 *  sin__q_LF_KFE__);
    (*this)(2,4) = ( 0.35 *  cos__q_LF_KFE__);
    (*this)(3,3) =  cos__q_LF_KFE__;
    (*this)(3,4) = - sin__q_LF_KFE__;
    (*this)(4,3) =  sin__q_LF_KFE__;
    (*this)(4,4) =  cos__q_LF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,2) = - sin__q_RF_HFE__;
    (*this)(0,4) = ( 0.08 *  sin__q_RF_HFE__);
    (*this)(1,0) = - sin__q_RF_HFE__;
    (*this)(1,2) = - cos__q_RF_HFE__;
    (*this)(1,4) = ( 0.08 *  cos__q_RF_HFE__);
    (*this)(3,3) =  cos__q_RF_HFE__;
    (*this)(3,5) = - sin__q_RF_HFE__;
    (*this)(4,3) = - sin__q_RF_HFE__;
    (*this)(4,5) = - cos__q_RF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,1) = - sin__q_RF_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_RF_HFE__);
    (*this)(1,4) = ( 0.08 *  cos__q_RF_HFE__);
    (*this)(2,0) = - sin__q_RF_HFE__;
    (*this)(2,1) = - cos__q_RF_HFE__;
    (*this)(3,3) =  cos__q_RF_HFE__;
    (*this)(3,4) = - sin__q_RF_HFE__;
    (*this)(5,3) = - sin__q_RF_HFE__;
    (*this)(5,4) = - cos__q_RF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.35;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) =  sin__q_RF_KFE__;
    (*this)(0,5) = ( 0.35 *  sin__q_RF_KFE__);
    (*this)(1,0) = - sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    (*this)(1,5) = ( 0.35 *  cos__q_RF_KFE__);
    (*this)(3,3) =  cos__q_RF_KFE__;
    (*this)(3,4) =  sin__q_RF_KFE__;
    (*this)(4,3) = - sin__q_RF_KFE__;
    (*this)(4,4) =  cos__q_RF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.35;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) = - sin__q_RF_KFE__;
    (*this)(1,0) =  sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    (*this)(2,3) = ( 0.35 *  sin__q_RF_KFE__);
    (*this)(2,4) = ( 0.35 *  cos__q_RF_KFE__);
    (*this)(3,3) =  cos__q_RF_KFE__;
    (*this)(3,4) = - sin__q_RF_KFE__;
    (*this)(4,3) =  sin__q_RF_KFE__;
    (*this)(4,4) =  cos__q_RF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::Type_fr_LH_upperleg_X_fr_LH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,2) =  sin__q_LH_HFE__;
    (*this)(0,4) = (- 0.08 *  sin__q_LH_HFE__);
    (*this)(1,0) = - sin__q_LH_HFE__;
    (*this)(1,2) =  cos__q_LH_HFE__;
    (*this)(1,4) = (- 0.08 *  cos__q_LH_HFE__);
    (*this)(3,3) =  cos__q_LH_HFE__;
    (*this)(3,5) =  sin__q_LH_HFE__;
    (*this)(4,3) = - sin__q_LH_HFE__;
    (*this)(4,5) =  cos__q_LH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::Type_fr_LH_hipassembly_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,1) = - sin__q_LH_HFE__;
    (*this)(1,3) = (- 0.08 *  sin__q_LH_HFE__);
    (*this)(1,4) = (- 0.08 *  cos__q_LH_HFE__);
    (*this)(2,0) =  sin__q_LH_HFE__;
    (*this)(2,1) =  cos__q_LH_HFE__;
    (*this)(3,3) =  cos__q_LH_HFE__;
    (*this)(3,4) = - sin__q_LH_HFE__;
    (*this)(5,3) =  sin__q_LH_HFE__;
    (*this)(5,4) =  cos__q_LH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.35;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) =  sin__q_LH_KFE__;
    (*this)(0,5) = ( 0.35 *  sin__q_LH_KFE__);
    (*this)(1,0) = - sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    (*this)(1,5) = ( 0.35 *  cos__q_LH_KFE__);
    (*this)(3,3) =  cos__q_LH_KFE__;
    (*this)(3,4) =  sin__q_LH_KFE__;
    (*this)(4,3) = - sin__q_LH_KFE__;
    (*this)(4,4) =  cos__q_LH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.35;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) = - sin__q_LH_KFE__;
    (*this)(1,0) =  sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    (*this)(2,3) = ( 0.35 *  sin__q_LH_KFE__);
    (*this)(2,4) = ( 0.35 *  cos__q_LH_KFE__);
    (*this)(3,3) =  cos__q_LH_KFE__;
    (*this)(3,4) = - sin__q_LH_KFE__;
    (*this)(4,3) =  sin__q_LH_KFE__;
    (*this)(4,4) =  cos__q_LH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::Type_fr_RH_upperleg_X_fr_RH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,2) = - sin__q_RH_HFE__;
    (*this)(0,4) = ( 0.08 *  sin__q_RH_HFE__);
    (*this)(1,0) = - sin__q_RH_HFE__;
    (*this)(1,2) = - cos__q_RH_HFE__;
    (*this)(1,4) = ( 0.08 *  cos__q_RH_HFE__);
    (*this)(3,3) =  cos__q_RH_HFE__;
    (*this)(3,5) = - sin__q_RH_HFE__;
    (*this)(4,3) = - sin__q_RH_HFE__;
    (*this)(4,5) = - cos__q_RH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::Type_fr_RH_hipassembly_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.08;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,1) = - sin__q_RH_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_RH_HFE__);
    (*this)(1,4) = ( 0.08 *  cos__q_RH_HFE__);
    (*this)(2,0) = - sin__q_RH_HFE__;
    (*this)(2,1) = - cos__q_RH_HFE__;
    (*this)(3,3) =  cos__q_RH_HFE__;
    (*this)(3,4) = - sin__q_RH_HFE__;
    (*this)(5,3) = - sin__q_RH_HFE__;
    (*this)(5,4) = - cos__q_RH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.35;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) =  sin__q_RH_KFE__;
    (*this)(0,5) = ( 0.35 *  sin__q_RH_KFE__);
    (*this)(1,0) = - sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    (*this)(1,5) = ( 0.35 *  cos__q_RH_KFE__);
    (*this)(3,3) =  cos__q_RH_KFE__;
    (*this)(3,4) =  sin__q_RH_KFE__;
    (*this)(4,3) = - sin__q_RH_KFE__;
    (*this)(4,4) =  cos__q_RH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.35;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& iit::TestHyQ::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) = - sin__q_RH_KFE__;
    (*this)(1,0) =  sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    (*this)(2,3) = ( 0.35 *  sin__q_RH_KFE__);
    (*this)(2,4) = ( 0.35 *  cos__q_RH_KFE__);
    (*this)(3,3) =  cos__q_RH_KFE__;
    (*this)(3,4) = - sin__q_RH_KFE__;
    (*this)(4,3) =  sin__q_RH_KFE__;
    (*this)(4,4) =  cos__q_RH_KFE__;
    return *this;
}

template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0.3735;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.207;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3735;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.207;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::Type_fr_trunk_X_fr_LH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.207;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::Type_fr_trunk_X_fr_RH_hipassembly()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.207;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::Type_fr_trunk_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.08 *  sin__q_LF_HAA__));
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::Type_fr_trunk_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (( 0.08 *  sin__q_RF_HAA__) -  0.207);
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::Type_fr_trunk_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.08 *  sin__q_LH_HAA__));
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::Type_fr_trunk_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (( 0.08 *  sin__q_RH_HAA__) -  0.207);
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::Type_fr_trunk_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = ( 0.3735 - ( 0.35 *  sin__q_LF_HFE__));
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((- 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) - ( 0.08 *  sin__q_LF_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - ( 0.08 *  cos__q_LF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::Type_fr_trunk_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = ( 0.3735 - ( 0.35 *  sin__q_RF_HFE__));
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (((( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) + ( 0.08 *  sin__q_RF_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - ( 0.08 *  cos__q_RF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::Type_fr_trunk_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = ((- 0.35 *  sin__q_LH_HFE__) -  0.3735);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((((- 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) - ( 0.08 *  sin__q_LH_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - ( 0.08 *  cos__q_LH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::Type_fr_trunk_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = ((- 0.35 *  sin__q_RH_HFE__) -  0.3735);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) + ( 0.08 *  sin__q_RH_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - ( 0.08 *  cos__q_RH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) = - sin__q_LF_HAA__;
    (*this)(0,2) = - cos__q_LF_HAA__;
    (*this)(0,3) = ( 0.207 *  sin__q_LF_HAA__);
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,2) =  sin__q_LF_HAA__;
    (*this)(1,3) = ( 0.207 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  sin__q_RF_HAA__;
    (*this)(0,2) = - cos__q_RF_HAA__;
    (*this)(0,3) = ( 0.207 *  sin__q_RF_HAA__);
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  sin__q_RF_HAA__;
    (*this)(1,3) = ( 0.207 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::Type_fr_LH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) = - sin__q_LH_HAA__;
    (*this)(0,2) = - cos__q_LH_HAA__;
    (*this)(0,3) = ( 0.207 *  sin__q_LH_HAA__);
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,2) =  sin__q_LH_HAA__;
    (*this)(1,3) = ( 0.207 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::Type_fr_RH_hipassembly_X_fr_trunk()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.3735;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  sin__q_RH_HAA__;
    (*this)(0,2) = - cos__q_RH_HAA__;
    (*this)(0,3) = ( 0.207 *  sin__q_RH_HAA__);
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  sin__q_RH_HAA__;
    (*this)(1,3) = ( 0.207 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(0,2) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(0,3) = (( 0.3735 *  sin__q_LF_HFE__) + ((( 0.207 *  sin__q_LF_HAA__) -  0.08) *  cos__q_LF_HFE__));
    (*this)(1,0) = - cos__q_LF_HFE__;
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,3) = ((( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) + ( 0.3735 *  cos__q_LF_HFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(0,2) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(0,3) = (( 0.3735 *  sin__q_RF_HFE__) + ((( 0.207 *  sin__q_RF_HAA__) -  0.08) *  cos__q_RF_HFE__));
    (*this)(1,0) = - cos__q_RF_HFE__;
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,3) = ((( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) + ( 0.3735 *  cos__q_RF_HFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::Type_fr_LH_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(0,2) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(0,3) = (((( 0.207 *  sin__q_LH_HAA__) -  0.08) *  cos__q_LH_HFE__) - ( 0.3735 *  sin__q_LH_HFE__));
    (*this)(1,0) = - cos__q_LH_HFE__;
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,3) = ((( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::Type_fr_RH_upperleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(0,2) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(0,3) = (((( 0.207 *  sin__q_RH_HAA__) -  0.08) *  cos__q_RH_HFE__) - ( 0.3735 *  sin__q_RH_HFE__));
    (*this)(1,0) = - cos__q_RH_HFE__;
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,3) = ((( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,3) = ((((( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) + ( 0.3735 *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((( 0.3735 *  sin__q_LF_HFE__) + ((( 0.207 *  sin__q_LF_HAA__) -  0.08) *  cos__q_LF_HFE__)) -  0.35) *  cos__q_LF_KFE__));
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,3) = (((((- 0.3735 *  sin__q_LF_HFE__) + (( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) +  0.35) *  sin__q_LF_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) + ( 0.3735 *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,3) = ((((( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) + ( 0.3735 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.3735 *  sin__q_RF_HFE__) + ((( 0.207 *  sin__q_RF_HAA__) -  0.08) *  cos__q_RF_HFE__)) -  0.35) *  cos__q_RF_KFE__));
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,3) = (((((- 0.3735 *  sin__q_RF_HFE__) + (( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  cos__q_RF_HFE__)) +  0.35) *  sin__q_RF_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) + ( 0.3735 *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::Type_fr_LH_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,3) = ((((( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((- 0.3735 *  sin__q_LH_HFE__) + ((( 0.207 *  sin__q_LH_HAA__) -  0.08) *  cos__q_LH_HFE__)) -  0.35) *  cos__q_LH_KFE__));
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,3) = ((((( 0.3735 *  sin__q_LH_HFE__) + (( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) +  0.35) *  sin__q_LH_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::Type_fr_RH_lowerleg_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,3) = ((((( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((- 0.3735 *  sin__q_RH_HFE__) + ((( 0.207 *  sin__q_RH_HAA__) -  0.08) *  cos__q_RH_HFE__)) -  0.35) *  cos__q_RH_KFE__));
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,3) = ((((( 0.3735 *  sin__q_RH_HFE__) + (( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  cos__q_RH_HFE__)) +  0.35) *  sin__q_RH_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0.2045;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,1) = - cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.043 *  sin__q_LF_HAA__));
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,1) =  sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.043 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.2045;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,3) = (( 0.043 *  sin__q_RF_HAA__) -  0.207);
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,3) = (- 0.043 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::Type_fr_trunk_X_LH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = - 0.2045;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,1) = - cos__q_LH_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.043 *  sin__q_LH_HAA__));
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,1) =  sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.043 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::Type_fr_trunk_X_RH_hipassemblyCOM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.2045;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_hipassemblyCOM::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,3) = (( 0.043 *  sin__q_RH_HAA__) -  0.207);
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.043 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(0,3) = (((- 0.151 *  sin__q_LF_HFE__) + ( 0.026 *  cos__q_LF_HFE__)) +  0.3735);
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = (((((- 0.026 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.151 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  sin__q_LF_HAA__)) +  0.207);
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = ((((- 0.026 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) - (( 0.151 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(0,3) = (((- 0.151 *  sin__q_RF_HFE__) + ( 0.026 *  cos__q_RF_HFE__)) +  0.3735);
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((((( 0.026 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.151 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__)) -  0.207);
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((- 0.026 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.151 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( 0.08 *  cos__q_RF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::Type_fr_trunk_X_LH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(0,3) = (((- 0.151 *  sin__q_LH_HFE__) - ( 0.026 *  cos__q_LH_HFE__)) -  0.3735);
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((((( 0.026 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.151 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  sin__q_LH_HAA__)) +  0.207);
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((( 0.026 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.151 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::Type_fr_trunk_X_RH_upperlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_upperlegCOM::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(0,3) = (((- 0.151 *  sin__q_RH_HFE__) - ( 0.026 *  cos__q_RH_HFE__)) -  0.3735);
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((((- 0.026 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.151 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__)) -  0.207);
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((( 0.026 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.151 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.08 *  cos__q_RH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = (((((- 0.125 *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.125 *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - ( 0.35 *  sin__q_LF_HFE__)) +  0.3735);
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((((( 0.125 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  sin__q_LF_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((((( 0.125 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.125 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RF_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = (((((( 0.001 *  sin__q_RF_HFE__) - ( 0.125 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((- 0.125 *  sin__q_RF_HFE__) - ( 0.001 *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - ( 0.35 *  sin__q_RF_HFE__)) +  0.3735);
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = ((((((((- 0.125 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.125 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) - (( 0.001 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__)) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ((((((( 0.125 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) + (( 0.001 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.001 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) - (( 0.125 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( 0.08 *  cos__q_RF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::Type_fr_trunk_X_LH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_LH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = ((((((- 0.001 *  sin__q_LH_HFE__) - ( 0.125 *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((( 0.001 *  cos__q_LH_HFE__) - ( 0.125 *  sin__q_LH_HFE__)) *  cos__q_LH_KFE__)) - ( 0.35 *  sin__q_LH_HFE__)) -  0.3735);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = (((((((( 0.125 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((- 0.001 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  sin__q_LH_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = ((((((( 0.125 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.001 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((- 0.001 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) - (( 0.125 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::Type_fr_trunk_X_RH_lowerlegCOM()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_RH_lowerlegCOM::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = ((((((- 0.001 *  sin__q_RH_HFE__) - ( 0.125 *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((( 0.001 *  cos__q_RH_HFE__) - ( 0.125 *  sin__q_RH_HFE__)) *  cos__q_RH_KFE__)) - ( 0.35 *  sin__q_RH_HFE__)) -  0.3735);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((((((( 0.001 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) - (( 0.125 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__)) *  sin__q_RH_KFE__) + (((( 0.001 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) + (( 0.125 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ((((((( 0.125 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.001 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((- 0.001 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) - (( 0.125 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.08 *  cos__q_RH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::Type_fr_LF_foot_X_fr_LF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.33;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_LF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::Type_fr_RF_foot_X_fr_RF_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.33;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_RF_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::Type_fr_LH_foot_X_fr_LH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.33;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_LH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::Type_fr_RH_foot_X_fr_RH_lowerleg()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.33;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_RH_lowerleg::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::Type_fr_trunk_X_fr_LF_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_foot::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,3) = (((((- 0.33 *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) - (( 0.33 *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) - ( 0.35 *  sin__q_LF_HFE__)) +  0.3735);
    (*this)(1,0) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((((( 0.33 *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  sin__q_LF_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,1) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((((( 0.33 *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( 0.33 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( 0.08 *  cos__q_LF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::Type_fr_trunk_X_fr_RF_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_foot::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,3) = (((((- 0.33 *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( 0.33 *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - ( 0.35 *  sin__q_RF_HFE__)) +  0.3735);
    (*this)(1,0) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (((((((- 0.33 *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( 0.33 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( 0.08 *  sin__q_RF_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,1) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((((( 0.33 *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( 0.33 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( 0.08 *  cos__q_RF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::Type_fr_trunk_X_fr_LH_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_foot::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,3) = (((((- 0.33 *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) - (( 0.33 *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) - ( 0.35 *  sin__q_LH_HFE__)) -  0.3735);
    (*this)(1,0) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((((((( 0.33 *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  sin__q_LH_HAA__)) +  0.207);
    (*this)(2,0) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,1) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((((( 0.33 *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( 0.33 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( 0.08 *  cos__q_LH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::Type_fr_trunk_X_fr_RH_foot()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_foot::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,3) = (((((- 0.33 *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( 0.33 *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - ( 0.35 *  sin__q_RH_HFE__)) -  0.3735);
    (*this)(1,0) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((((((- 0.33 *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( 0.33 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( 0.08 *  sin__q_RH_HAA__)) -  0.207);
    (*this)(2,0) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,1) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((((( 0.33 *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( 0.33 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( 0.08 *  cos__q_RH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::Type_fr_LF_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_KFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((- cos__q_LF_HFE__ *  sin__q_LF_KFE__) - ( sin__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(0,1) = ((( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,2) = ((( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(0,3) = (((((( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) + ( 0.3735 *  cos__q_LF_HFE__)) *  sin__q_LF_KFE__) + (((( 0.3735 *  sin__q_LF_HFE__) + ((( 0.207 *  sin__q_LF_HAA__) -  0.08) *  cos__q_LF_HFE__)) -  0.35) *  cos__q_LF_KFE__)) -  0.33);
    (*this)(1,0) = (( sin__q_LF_HFE__ *  sin__q_LF_KFE__) - ( cos__q_LF_HFE__ *  cos__q_LF_KFE__));
    (*this)(1,1) = ((( sin__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( sin__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,2) = ((( cos__q_LF_HAA__ *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + (( cos__q_LF_HAA__ *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(1,3) = (((((- 0.3735 *  sin__q_LF_HFE__) + (( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  cos__q_LF_HFE__)) +  0.35) *  sin__q_LF_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_LF_HAA__)) *  sin__q_LF_HFE__) + ( 0.3735 *  cos__q_LF_HFE__)) *  cos__q_LF_KFE__));
    (*this)(2,1) =  cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::Type_fr_RF_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_KFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((- cos__q_RF_HFE__ *  sin__q_RF_KFE__) - ( sin__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(0,1) = ((( sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__));
    (*this)(0,2) = ((( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(0,3) = (((((( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) + ( 0.3735 *  cos__q_RF_HFE__)) *  sin__q_RF_KFE__) + (((( 0.3735 *  sin__q_RF_HFE__) + ((( 0.207 *  sin__q_RF_HAA__) -  0.08) *  cos__q_RF_HFE__)) -  0.35) *  cos__q_RF_KFE__)) -  0.33);
    (*this)(1,0) = (( sin__q_RF_HFE__ *  sin__q_RF_KFE__) - ( cos__q_RF_HFE__ *  cos__q_RF_KFE__));
    (*this)(1,1) = (((- sin__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - (( sin__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,2) = ((( cos__q_RF_HAA__ *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + (( cos__q_RF_HAA__ *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(1,3) = (((((- 0.3735 *  sin__q_RF_HFE__) + (( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  cos__q_RF_HFE__)) +  0.35) *  sin__q_RF_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_RF_HAA__)) *  sin__q_RF_HFE__) + ( 0.3735 *  cos__q_RF_HFE__)) *  cos__q_RF_KFE__));
    (*this)(2,1) =  cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::Type_fr_LH_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_KFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((- cos__q_LH_HFE__ *  sin__q_LH_KFE__) - ( sin__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(0,1) = ((( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,2) = ((( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(0,3) = (((((( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HFE__)) *  sin__q_LH_KFE__) + ((((- 0.3735 *  sin__q_LH_HFE__) + ((( 0.207 *  sin__q_LH_HAA__) -  0.08) *  cos__q_LH_HFE__)) -  0.35) *  cos__q_LH_KFE__)) -  0.33);
    (*this)(1,0) = (( sin__q_LH_HFE__ *  sin__q_LH_KFE__) - ( cos__q_LH_HFE__ *  cos__q_LH_KFE__));
    (*this)(1,1) = ((( sin__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( sin__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,2) = ((( cos__q_LH_HAA__ *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + (( cos__q_LH_HAA__ *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(1,3) = ((((( 0.3735 *  sin__q_LH_HFE__) + (( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  cos__q_LH_HFE__)) +  0.35) *  sin__q_LH_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_LH_HAA__)) *  sin__q_LH_HFE__) - ( 0.3735 *  cos__q_LH_HFE__)) *  cos__q_LH_KFE__));
    (*this)(2,1) =  cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.207 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::Type_fr_RH_foot_X_fr_trunk()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_trunk::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_KFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((- cos__q_RH_HFE__ *  sin__q_RH_KFE__) - ( sin__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(0,1) = ((( sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__));
    (*this)(0,2) = ((( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(0,3) = (((((( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HFE__)) *  sin__q_RH_KFE__) + ((((- 0.3735 *  sin__q_RH_HFE__) + ((( 0.207 *  sin__q_RH_HAA__) -  0.08) *  cos__q_RH_HFE__)) -  0.35) *  cos__q_RH_KFE__)) -  0.33);
    (*this)(1,0) = (( sin__q_RH_HFE__ *  sin__q_RH_KFE__) - ( cos__q_RH_HFE__ *  cos__q_RH_KFE__));
    (*this)(1,1) = (((- sin__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - (( sin__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,2) = ((( cos__q_RH_HAA__ *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + (( cos__q_RH_HAA__ *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(1,3) = ((((( 0.3735 *  sin__q_RH_HFE__) + (( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  cos__q_RH_HFE__)) +  0.35) *  sin__q_RH_KFE__) + (((( 0.08 - ( 0.207 *  sin__q_RH_HAA__)) *  sin__q_RH_HFE__) - ( 0.3735 *  cos__q_RH_HFE__)) *  cos__q_RH_KFE__));
    (*this)(2,1) =  cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = ( 0.207 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::Type_fr_trunk_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = 0.3735;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.207;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::Type_fr_trunk_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3735;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.207;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::Type_fr_trunk_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.207;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::Type_fr_trunk_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.207;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::Type_fr_trunk_X_fr_LF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3735;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_HFE::update(const JState& q) {
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) = - sin__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.08 *  sin__q_LF_HAA__));
    (*this)(2,0) = - cos__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_LF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::Type_fr_trunk_X_fr_RF_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3735;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_HFE::update(const JState& q) {
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  sin__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (( 0.08 *  sin__q_RF_HAA__) -  0.207);
    (*this)(2,0) = - cos__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_RF_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::Type_fr_trunk_X_fr_LH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_HFE::update(const JState& q) {
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) = - sin__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ( 0.207 - ( 0.08 *  sin__q_LH_HAA__));
    (*this)(2,0) = - cos__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_LH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::Type_fr_trunk_X_fr_RH_HFE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3735;
    (*this)(1,1) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_HFE::update(const JState& q) {
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  sin__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (( 0.08 *  sin__q_RH_HAA__) -  0.207);
    (*this)(2,0) = - cos__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (- 0.08 *  cos__q_RH_HAA__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::Type_fr_trunk_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LF_KFE::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar sin__q_LF_HAA__;
    Scalar cos__q_LF_HFE__;
    Scalar cos__q_LF_HAA__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    sin__q_LF_HAA__ = TRAIT::sin( q(LF_HAA));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    cos__q_LF_HAA__ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = - sin__q_LF_HFE__;
    (*this)(0,1) = - cos__q_LF_HFE__;
    (*this)(0,3) = ( 0.3735 - ( 0.35 *  sin__q_LF_HFE__));
    (*this)(1,0) = (- sin__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(1,1) = ( sin__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(1,3) = ((((- 0.35 *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) - ( 0.08 *  sin__q_LF_HAA__)) +  0.207);
    (*this)(2,0) = (- cos__q_LF_HAA__ *  cos__q_LF_HFE__);
    (*this)(2,1) = ( cos__q_LF_HAA__ *  sin__q_LF_HFE__);
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) - ( 0.08 *  cos__q_LF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::Type_fr_trunk_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RF_KFE::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar sin__q_RF_HAA__;
    Scalar cos__q_RF_HFE__;
    Scalar cos__q_RF_HAA__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    sin__q_RF_HAA__ = TRAIT::sin( q(RF_HAA));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    cos__q_RF_HAA__ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = - sin__q_RF_HFE__;
    (*this)(0,1) = - cos__q_RF_HFE__;
    (*this)(0,3) = ( 0.3735 - ( 0.35 *  sin__q_RF_HFE__));
    (*this)(1,0) = ( sin__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(1,1) = (- sin__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(1,3) = (((( 0.35 *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) + ( 0.08 *  sin__q_RF_HAA__)) -  0.207);
    (*this)(2,0) = (- cos__q_RF_HAA__ *  cos__q_RF_HFE__);
    (*this)(2,1) = ( cos__q_RF_HAA__ *  sin__q_RF_HFE__);
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) - ( 0.08 *  cos__q_RF_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::Type_fr_trunk_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_LH_KFE::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar sin__q_LH_HAA__;
    Scalar cos__q_LH_HFE__;
    Scalar cos__q_LH_HAA__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    sin__q_LH_HAA__ = TRAIT::sin( q(LH_HAA));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    cos__q_LH_HAA__ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = - sin__q_LH_HFE__;
    (*this)(0,1) = - cos__q_LH_HFE__;
    (*this)(0,3) = ((- 0.35 *  sin__q_LH_HFE__) -  0.3735);
    (*this)(1,0) = (- sin__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(1,1) = ( sin__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(1,3) = ((((- 0.35 *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) - ( 0.08 *  sin__q_LH_HAA__)) +  0.207);
    (*this)(2,0) = (- cos__q_LH_HAA__ *  cos__q_LH_HFE__);
    (*this)(2,1) = ( cos__q_LH_HAA__ *  sin__q_LH_HFE__);
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) - ( 0.08 *  cos__q_LH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::Type_fr_trunk_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_trunk_X_fr_RH_KFE::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar sin__q_RH_HAA__;
    Scalar cos__q_RH_HFE__;
    Scalar cos__q_RH_HAA__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    sin__q_RH_HAA__ = TRAIT::sin( q(RH_HAA));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    cos__q_RH_HAA__ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = - sin__q_RH_HFE__;
    (*this)(0,1) = - cos__q_RH_HFE__;
    (*this)(0,3) = ((- 0.35 *  sin__q_RH_HFE__) -  0.3735);
    (*this)(1,0) = ( sin__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(1,1) = (- sin__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(1,3) = (((( 0.35 *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) + ( 0.08 *  sin__q_RH_HAA__)) -  0.207);
    (*this)(2,0) = (- cos__q_RH_HAA__ *  cos__q_RH_HFE__);
    (*this)(2,1) = ( cos__q_RH_HAA__ *  sin__q_RH_HFE__);
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(2,3) = (((- 0.35 *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) - ( 0.08 *  cos__q_RH_HAA__));
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,2) =  sin__q_LF_HFE__;
    (*this)(0,3) = (- 0.08 *  cos__q_LF_HFE__);
    (*this)(1,0) = - sin__q_LF_HFE__;
    (*this)(1,2) =  cos__q_LF_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_LF_HFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_HFE__;
    Scalar cos__q_LF_HFE__;
    
    sin__q_LF_HFE__ = TRAIT::sin( q(LF_HFE));
    cos__q_LF_HFE__ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  cos__q_LF_HFE__;
    (*this)(0,1) = - sin__q_LF_HFE__;
    (*this)(2,0) =  sin__q_LF_HFE__;
    (*this)(2,1) =  cos__q_LF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) =  sin__q_LF_KFE__;
    (*this)(0,3) = (- 0.35 *  cos__q_LF_KFE__);
    (*this)(1,0) = - sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    (*this)(1,3) = ( 0.35 *  sin__q_LF_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.35;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q) {
    Scalar sin__q_LF_KFE__;
    Scalar cos__q_LF_KFE__;
    
    sin__q_LF_KFE__ = TRAIT::sin( q(LF_KFE));
    cos__q_LF_KFE__ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  cos__q_LF_KFE__;
    (*this)(0,1) = - sin__q_LF_KFE__;
    (*this)(1,0) =  sin__q_LF_KFE__;
    (*this)(1,1) =  cos__q_LF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,2) = - sin__q_RF_HFE__;
    (*this)(0,3) = (- 0.08 *  cos__q_RF_HFE__);
    (*this)(1,0) = - sin__q_RF_HFE__;
    (*this)(1,2) = - cos__q_RF_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_RF_HFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_HFE__;
    Scalar cos__q_RF_HFE__;
    
    sin__q_RF_HFE__ = TRAIT::sin( q(RF_HFE));
    cos__q_RF_HFE__ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  cos__q_RF_HFE__;
    (*this)(0,1) = - sin__q_RF_HFE__;
    (*this)(2,0) = - sin__q_RF_HFE__;
    (*this)(2,1) = - cos__q_RF_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) =  sin__q_RF_KFE__;
    (*this)(0,3) = (- 0.35 *  cos__q_RF_KFE__);
    (*this)(1,0) = - sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    (*this)(1,3) = ( 0.35 *  sin__q_RF_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.35;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q) {
    Scalar sin__q_RF_KFE__;
    Scalar cos__q_RF_KFE__;
    
    sin__q_RF_KFE__ = TRAIT::sin( q(RF_KFE));
    cos__q_RF_KFE__ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  cos__q_RF_KFE__;
    (*this)(0,1) = - sin__q_RF_KFE__;
    (*this)(1,0) =  sin__q_RF_KFE__;
    (*this)(1,1) =  cos__q_RF_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::Type_fr_LH_upperleg_X_fr_LH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hipassembly::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,2) =  sin__q_LH_HFE__;
    (*this)(0,3) = (- 0.08 *  cos__q_LH_HFE__);
    (*this)(1,0) = - sin__q_LH_HFE__;
    (*this)(1,2) =  cos__q_LH_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_LH_HFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::Type_fr_LH_hipassembly_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hipassembly_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_HFE__;
    Scalar cos__q_LH_HFE__;
    
    sin__q_LH_HFE__ = TRAIT::sin( q(LH_HFE));
    cos__q_LH_HFE__ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  cos__q_LH_HFE__;
    (*this)(0,1) = - sin__q_LH_HFE__;
    (*this)(2,0) =  sin__q_LH_HFE__;
    (*this)(2,1) =  cos__q_LH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) =  sin__q_LH_KFE__;
    (*this)(0,3) = (- 0.35 *  cos__q_LH_KFE__);
    (*this)(1,0) = - sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    (*this)(1,3) = ( 0.35 *  sin__q_LH_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.35;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q) {
    Scalar sin__q_LH_KFE__;
    Scalar cos__q_LH_KFE__;
    
    sin__q_LH_KFE__ = TRAIT::sin( q(LH_KFE));
    cos__q_LH_KFE__ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  cos__q_LH_KFE__;
    (*this)(0,1) = - sin__q_LH_KFE__;
    (*this)(1,0) =  sin__q_LH_KFE__;
    (*this)(1,1) =  cos__q_LH_KFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::Type_fr_RH_upperleg_X_fr_RH_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hipassembly::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,2) = - sin__q_RH_HFE__;
    (*this)(0,3) = (- 0.08 *  cos__q_RH_HFE__);
    (*this)(1,0) = - sin__q_RH_HFE__;
    (*this)(1,2) = - cos__q_RH_HFE__;
    (*this)(1,3) = ( 0.08 *  sin__q_RH_HFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::Type_fr_RH_hipassembly_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hipassembly_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_HFE__;
    Scalar cos__q_RH_HFE__;
    
    sin__q_RH_HFE__ = TRAIT::sin( q(RH_HFE));
    cos__q_RH_HFE__ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  cos__q_RH_HFE__;
    (*this)(0,1) = - sin__q_RH_HFE__;
    (*this)(2,0) = - sin__q_RH_HFE__;
    (*this)(2,1) = - cos__q_RH_HFE__;
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) =  sin__q_RH_KFE__;
    (*this)(0,3) = (- 0.35 *  cos__q_RH_KFE__);
    (*this)(1,0) = - sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    (*this)(1,3) = ( 0.35 *  sin__q_RH_KFE__);
    return *this;
}
template <typename TRAIT>
iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.35;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& iit::TestHyQ::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q) {
    Scalar sin__q_RH_KFE__;
    Scalar cos__q_RH_KFE__;
    
    sin__q_RH_KFE__ = TRAIT::sin( q(RH_KFE));
    cos__q_RH_KFE__ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  cos__q_RH_KFE__;
    (*this)(0,1) = - sin__q_RH_KFE__;
    (*this)(1,0) =  sin__q_RH_KFE__;
    (*this)(1,1) =  cos__q_RH_KFE__;
    return *this;
}

