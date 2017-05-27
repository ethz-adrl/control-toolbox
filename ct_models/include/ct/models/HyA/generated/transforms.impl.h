
// Constructors
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_HyABase_X_fr_Shoulder_AA(),
    fr_HyABase_X_fr_Shoulder_FE(),
    fr_HyABase_X_fr_Humerus_R(),
    fr_HyABase_X_fr_Elbow_FE(),
    fr_HyABase_X_fr_Wrist_R(),
    fr_HyABase_X_fr_Wrist_FE(),
    fr_HyABase_X_fr_ee(),
    fr_HyABase_X_fr_Shoulder_AA_COM(),
    fr_HyABase_X_fr_Shoulder_FE_COM(),
    fr_HyABase_X_fr_Humerus_R_COM(),
    fr_HyABase_X_fr_Elbow_FE_COM(),
    fr_HyABase_X_fr_Wrist_R_COM(),
    fr_HyABase_X_fr_Wrist_FE_COM(),
    fr_HyABase_X_fr_Shoulder_AA_CTR(),
    fr_HyABase_X_fr_Shoulder_FE_CTR(),
    fr_HyABase_X_fr_Humerus_R_CTR(),
    fr_HyABase_X_fr_Elbow_FE_CTR(),
    fr_HyABase_X_fr_Wrist_R_CTR(),
    fr_Shoulder_FE_X_fr_Wrist_FE(),
    fr_Humerus_R_X_fr_Wrist_FE(),
    fr_Elbow_FE_X_fr_Wrist_FE(),
    fr_Wrist_R_X_fr_Wrist_FE(),
    fr_Wrist_FE_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_ee(),
    fr_Humerus_R_X_fr_ee(),
    fr_Elbow_FE_X_fr_ee(),
    fr_Wrist_R_X_fr_ee(),
    fr_Wrist_FE_X_fr_ee(),
    fr_Shoulder_AA_X_fr_HyABase(),
    fr_Shoulder_FE_X_fr_HyABase(),
    fr_Humerus_R_X_fr_HyABase(),
    fr_Elbow_FE_X_fr_HyABase(),
    fr_Wrist_R_X_fr_HyABase(),
    fr_Wrist_FE_X_fr_HyABase(),
    fr_ee_X_fr_HyABase(),
    fr_HyABase_X_fr_SAA(),
    fr_HyABase_X_fr_SFE(),
    fr_HyABase_X_fr_HR(),
    fr_HyABase_X_fr_EFE(),
    fr_HyABase_X_fr_WR(),
    fr_HyABase_X_fr_WFE(),
    fr_Shoulder_FE_X_fr_Shoulder_AA(),
    fr_Shoulder_AA_X_fr_Shoulder_FE(),
    fr_Humerus_R_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_Humerus_R(),
    fr_Elbow_FE_X_fr_Humerus_R(),
    fr_Humerus_R_X_fr_Elbow_FE(),
    fr_Wrist_R_X_fr_Elbow_FE(),
    fr_Elbow_FE_X_fr_Wrist_R(),
    fr_Wrist_FE_X_fr_Wrist_R()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_HyA::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_HyABase_X_fr_Shoulder_AA(),
    fr_HyABase_X_fr_Shoulder_FE(),
    fr_HyABase_X_fr_Humerus_R(),
    fr_HyABase_X_fr_Elbow_FE(),
    fr_HyABase_X_fr_Wrist_R(),
    fr_HyABase_X_fr_Wrist_FE(),
    fr_HyABase_X_fr_ee(),
    fr_HyABase_X_fr_Shoulder_AA_COM(),
    fr_HyABase_X_fr_Shoulder_FE_COM(),
    fr_HyABase_X_fr_Humerus_R_COM(),
    fr_HyABase_X_fr_Elbow_FE_COM(),
    fr_HyABase_X_fr_Wrist_R_COM(),
    fr_HyABase_X_fr_Wrist_FE_COM(),
    fr_HyABase_X_fr_Shoulder_AA_CTR(),
    fr_HyABase_X_fr_Shoulder_FE_CTR(),
    fr_HyABase_X_fr_Humerus_R_CTR(),
    fr_HyABase_X_fr_Elbow_FE_CTR(),
    fr_HyABase_X_fr_Wrist_R_CTR(),
    fr_Shoulder_FE_X_fr_Wrist_FE(),
    fr_Humerus_R_X_fr_Wrist_FE(),
    fr_Elbow_FE_X_fr_Wrist_FE(),
    fr_Wrist_R_X_fr_Wrist_FE(),
    fr_Wrist_FE_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_ee(),
    fr_Humerus_R_X_fr_ee(),
    fr_Elbow_FE_X_fr_ee(),
    fr_Wrist_R_X_fr_ee(),
    fr_Wrist_FE_X_fr_ee(),
    fr_Shoulder_AA_X_fr_HyABase(),
    fr_Shoulder_FE_X_fr_HyABase(),
    fr_Humerus_R_X_fr_HyABase(),
    fr_Elbow_FE_X_fr_HyABase(),
    fr_Wrist_R_X_fr_HyABase(),
    fr_Wrist_FE_X_fr_HyABase(),
    fr_ee_X_fr_HyABase(),
    fr_HyABase_X_fr_SAA(),
    fr_HyABase_X_fr_SFE(),
    fr_HyABase_X_fr_HR(),
    fr_HyABase_X_fr_EFE(),
    fr_HyABase_X_fr_WR(),
    fr_HyABase_X_fr_WFE(),
    fr_Shoulder_FE_X_fr_Shoulder_AA(),
    fr_Shoulder_AA_X_fr_Shoulder_FE(),
    fr_Humerus_R_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_Humerus_R(),
    fr_Elbow_FE_X_fr_Humerus_R(),
    fr_Humerus_R_X_fr_Elbow_FE(),
    fr_Wrist_R_X_fr_Elbow_FE(),
    fr_Elbow_FE_X_fr_Wrist_R(),
    fr_Wrist_FE_X_fr_Wrist_R()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_HyA::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_HyABase_X_fr_Shoulder_AA(),
    fr_HyABase_X_fr_Shoulder_FE(),
    fr_HyABase_X_fr_Humerus_R(),
    fr_HyABase_X_fr_Elbow_FE(),
    fr_HyABase_X_fr_Wrist_R(),
    fr_HyABase_X_fr_Wrist_FE(),
    fr_HyABase_X_fr_ee(),
    fr_HyABase_X_fr_Shoulder_AA_COM(),
    fr_HyABase_X_fr_Shoulder_FE_COM(),
    fr_HyABase_X_fr_Humerus_R_COM(),
    fr_HyABase_X_fr_Elbow_FE_COM(),
    fr_HyABase_X_fr_Wrist_R_COM(),
    fr_HyABase_X_fr_Wrist_FE_COM(),
    fr_HyABase_X_fr_Shoulder_AA_CTR(),
    fr_HyABase_X_fr_Shoulder_FE_CTR(),
    fr_HyABase_X_fr_Humerus_R_CTR(),
    fr_HyABase_X_fr_Elbow_FE_CTR(),
    fr_HyABase_X_fr_Wrist_R_CTR(),
    fr_Shoulder_FE_X_fr_Wrist_FE(),
    fr_Humerus_R_X_fr_Wrist_FE(),
    fr_Elbow_FE_X_fr_Wrist_FE(),
    fr_Wrist_R_X_fr_Wrist_FE(),
    fr_Wrist_FE_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_ee(),
    fr_Humerus_R_X_fr_ee(),
    fr_Elbow_FE_X_fr_ee(),
    fr_Wrist_R_X_fr_ee(),
    fr_Wrist_FE_X_fr_ee(),
    fr_Shoulder_AA_X_fr_HyABase(),
    fr_Shoulder_FE_X_fr_HyABase(),
    fr_Humerus_R_X_fr_HyABase(),
    fr_Elbow_FE_X_fr_HyABase(),
    fr_Wrist_R_X_fr_HyABase(),
    fr_Wrist_FE_X_fr_HyABase(),
    fr_ee_X_fr_HyABase(),
    fr_HyABase_X_fr_SAA(),
    fr_HyABase_X_fr_SFE(),
    fr_HyABase_X_fr_HR(),
    fr_HyABase_X_fr_EFE(),
    fr_HyABase_X_fr_WR(),
    fr_HyABase_X_fr_WFE(),
    fr_Shoulder_FE_X_fr_Shoulder_AA(),
    fr_Shoulder_AA_X_fr_Shoulder_FE(),
    fr_Humerus_R_X_fr_Shoulder_FE(),
    fr_Shoulder_FE_X_fr_Humerus_R(),
    fr_Elbow_FE_X_fr_Humerus_R(),
    fr_Humerus_R_X_fr_Elbow_FE(),
    fr_Wrist_R_X_fr_Elbow_FE(),
    fr_Elbow_FE_X_fr_Wrist_R(),
    fr_Wrist_FE_X_fr_Wrist_R()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::Type_fr_HyABase_X_fr_Shoulder_AA()
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
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::Type_fr_HyABase_X_fr_Shoulder_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,0) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = (( 0.178 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(3,2) = ( 0.178 *  cos__q_SAA__);
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,0) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = ((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,2) = ( 0.178 *  sin__q_SAA__);
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::Type_fr_HyABase_X_fr_Humerus_R()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.112 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,0) = ((((((- 0.112 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((( 0.112 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,0) = (( 0.112 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(5,1) = ((- 0.112 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::Type_fr_HyABase_X_fr_Elbow_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,0) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,0) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,1) = (((- 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,2) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::Type_fr_HyABase_X_fr_Wrist_R()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,1) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,2) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,1) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,2) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,0) = ((((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(5,1) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,2) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::Type_fr_HyABase_X_fr_Wrist_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,1) = ((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,2) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,1) = ((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,2) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,0) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(5,1) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(5,2) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::Type_fr_HyABase_X_fr_ee()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,1) = ((((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,2) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,1) = ((((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,2) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,0) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(5,1) = ((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - (( 0.03 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(5,2) = ((((((((((( 0.03 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) - ((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::Type_fr_HyABase_X_fr_Shoulder_AA_COM()
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
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 3.6E-4;
    (*this)(5,1) = - 2.0E-4;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,0) = (- 0.00309 *  sin__q_SAA__);
    (*this)(3,1) = (- 0.00309 *  cos__q_SAA__);
    (*this)(3,2) = ((- 2.0E-4 *  sin__q_SAA__) - ( 3.6E-4 *  cos__q_SAA__));
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,0) = ( 0.00309 *  cos__q_SAA__);
    (*this)(4,1) = (- 0.00309 *  sin__q_SAA__);
    (*this)(4,2) = (( 2.0E-4 *  cos__q_SAA__) - ( 3.6E-4 *  sin__q_SAA__));
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::Type_fr_HyABase_X_fr_Shoulder_FE_COM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,0) = ((((- 0.00117 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.178 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.02338 *  sin__q_SAA__));
    (*this)(3,1) = (((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.00117 *  cos__q_SAA__) *  cos__q_SFE__)) - ( 0.00383 *  sin__q_SAA__));
    (*this)(3,2) = ((((- 0.00383 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02338 *  cos__q_SAA__) *  cos__q_SFE__)) + ( 0.178 *  cos__q_SAA__));
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,0) = ((((- 0.00117 *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__)) - ( 0.02338 *  cos__q_SAA__));
    (*this)(4,1) = ((((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.00117 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.00383 *  cos__q_SAA__));
    (*this)(4,2) = ((((- 0.00383 *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02338 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.178 *  sin__q_SAA__));
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,0) = ( 0.00117 *  cos__q_SFE__);
    (*this)(5,1) = (- 0.00117 *  sin__q_SFE__);
    (*this)(5,2) = (( 0.00383 *  cos__q_SFE__) - ( 0.02338 *  sin__q_SFE__));
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::Type_fr_HyABase_X_fr_Humerus_R_COM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = ((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.08646 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00618 *  cos__q_SAA__) *  cos__q_SFE__)) + (( 0.08646 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = ((((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.08646 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00239 *  cos__q_SAA__) *  cos__q_SFE__)) + (( 0.08646 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = ((((((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.178 *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,0) = (((((((- 0.08646 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00618 *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.08646 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = ((((((( 0.08646 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00239 *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.08646 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = ((((((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00618 *  cos__q_HR__) - ( 0.00239 *  sin__q_HR__)) *  cos__q_SAA__));
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,0) = ((( 0.08646 *  cos__q_HR__) *  cos__q_SFE__) - ( 0.00618 *  sin__q_SFE__));
    (*this)(5,1) = ((- 0.00239 *  sin__q_SFE__) - (( 0.08646 *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,2) = ((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::Type_fr_HyABase_X_fr_Elbow_FE_COM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((((( 0.416 *  sin__q_EFE__) +  0.01125) *  sin__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.178 *  cos__q_EFE__) *  sin__q_SAA__) - (( 0.00102 *  sin__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + (((( 0.00102 *  cos__q_EFE__) *  sin__q_HR__) + ((( 0.416 *  sin__q_EFE__) +  0.01125) *  cos__q_HR__)) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((((( 0.416 *  cos__q_EFE__) -  0.1466) *  sin__q_HR__) + (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_SFE__) + (((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + ((((( 0.416 *  cos__q_EFE__) -  0.1466) *  cos__q_HR__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (((((((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  cos__q_SAA__) *  cos__q_SFE__)) + (((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,0) = (((((((((( 0.416 *  sin__q_EFE__) +  0.01125) *  sin__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_SAA__)) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + (((((- 0.416 *  sin__q_EFE__) -  0.01125) *  cos__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__));
    (*this)(4,1) = (((((((((( 0.416 *  cos__q_EFE__) -  0.1466) *  sin__q_HR__) + (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.00102 *  cos__q_EFE__) *  sin__q_SAA__) - (( 0.178 *  sin__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + (((( 0.00102 *  sin__q_EFE__) *  sin__q_HR__) + (( 0.1466 - ( 0.416 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(4,2) = (((((((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,0) = ((((((- 0.416 *  sin__q_EFE__) -  0.01125) *  sin__q_HR__) + (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_SFE__));
    (*this)(5,1) = ((((( 0.1466 - ( 0.416 *  cos__q_EFE__)) *  sin__q_HR__) - (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.00102 *  cos__q_EFE__) *  sin__q_SFE__));
    (*this)(5,2) = ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_SFE__) + (((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::Type_fr_HyABase_X_fr_Wrist_R_COM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.08883 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.00261 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,1) = (((((((((((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.08883 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 4.0E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,2) = ((((((((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  cos__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + ((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.08883) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.08883 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.00261 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((((((((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.08883 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.08883) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 4.0E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = ((((((((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.00261 *  cos__q_EFE__) *  sin__q_HR__) + ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,0) = ((((((((- 0.416 *  cos__q_EFE__) -  0.08883) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.08883 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__)) + (( 0.00261 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,1) = ((((((( 0.08883 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.08883) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 4.0E-4 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,2) = (((((( 0.00261 *  sin__q_EFE__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  sin__q_WR__) + (((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SFE__) + (((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::Type_fr_HyABase_X_fr_Wrist_FE_COM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,0) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = (((((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,1) = (((((((((((((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - ((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,2) = ((((((((((((((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((- 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = ((((((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.01063 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,1) = (((((((((((((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (( 0.07876 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,2) = ((((((((((((((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.07876 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,0) = (((((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.01063 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) + (((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(5,1) = (((((((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SFE__) - (( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + (( 0.07876 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,2) = ((((((((((((( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) + ((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.01063 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.07876 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + ((((- 0.07876 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + ((((- 0.01063 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::Type_fr_HyABase_X_fr_Shoulder_AA_CTR()
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
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
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
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,0) = (- 0.089 *  sin__q_SAA__);
    (*this)(3,1) = (- 0.089 *  cos__q_SAA__);
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,0) = ( 0.089 *  cos__q_SAA__);
    (*this)(4,1) = (- 0.089 *  sin__q_SAA__);
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::Type_fr_HyABase_X_fr_Shoulder_FE_CTR()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,0) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = ((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.056 *  sin__q_SAA__));
    (*this)(3,2) = ((( 0.056 *  cos__q_SAA__) *  sin__q_SFE__) + ( 0.178 *  cos__q_SAA__));
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,0) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = (((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.056 *  cos__q_SAA__));
    (*this)(4,2) = ((( 0.056 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.178 *  sin__q_SAA__));
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,2) = (- 0.056 *  cos__q_SFE__);
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::Type_fr_HyABase_X_fr_Humerus_R_CTR()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.264 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.264 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.264 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.264 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,0) = ((((((- 0.264 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.264 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((( 0.264 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.264 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,0) = (( 0.264 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(5,1) = ((- 0.264 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::Type_fr_HyABase_X_fr_Elbow_FE_CTR()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.056) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.056) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = ((((((((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.056 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,0) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = ((((((((( 0.416 *  cos__q_EFE__) +  0.056) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.056) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = ((((((((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.056 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,0) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,1) = ((((- 0.416 *  cos__q_EFE__) -  0.056) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,2) = ((( 0.056 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.056 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::Type_fr_HyABase_X_fr_Wrist_R_CTR()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,1) = ((((((((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,2) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.204) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,1) = ((((((((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.204) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,2) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,0) = ((((((- 0.416 *  cos__q_EFE__) -  0.204) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.204 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(5,1) = ((((( 0.204 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.204) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,2) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::Type_fr_Shoulder_FE_X_fr_Wrist_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(3,0) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(3,1) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(3,2) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(3,5) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(4,0) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(4,1) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(4,2) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_WR__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(4,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,0) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(5,1) = (((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(5,2) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_WR__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::Type_fr_Humerus_R_X_fr_Wrist_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(3,0) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  sin__q_WFE__) + ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__) - (( 0.304 *  sin__q_EFE__) *  cos__q_WFE__));
    (*this)(3,1) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WFE__) - ( 0.02075 *  sin__q_WFE__)) *  sin__q_WR__) + (( 0.304 *  sin__q_EFE__) *  sin__q_WFE__));
    (*this)(3,2) = (((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WR__);
    (*this)(3,3) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(3,4) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(3,5) = - sin__q_WR__;
    (*this)(4,0) = ((((( 0.296 *  cos__q_EFE__) +  0.304) *  sin__q_WFE__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,1) = (((( 0.02075 *  cos__q_EFE__) *  sin__q_WFE__) + ((( 0.296 *  cos__q_EFE__) +  0.304) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,2) = ((((- 0.296 *  cos__q_EFE__) -  0.304) *  sin__q_WR__) + ( 0.02075 *  sin__q_EFE__));
    (*this)(4,3) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(4,4) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(4,5) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(5,0) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(5,1) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(5,2) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(5,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(5,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(5,5) = (- sin__q_EFE__ *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::Type_fr_Elbow_FE_X_fr_Wrist_FE()
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.02075;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(3,3) =  cos__q_WFE__;
    (*this)(3,4) = - sin__q_WFE__;
    (*this)(4,0) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,1) = ((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,2) = (- 0.296 *  sin__q_WR__);
    (*this)(4,3) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(4,4) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(4,5) =  cos__q_WR__;
    (*this)(5,0) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(5,1) = ((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(5,2) = ( 0.296 *  cos__q_WR__);
    (*this)(5,3) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(5,4) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(5,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::Type_fr_Wrist_R_X_fr_Wrist_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.184;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.02075;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,4) =  cos__q_WFE__;
    (*this)(4,0) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(4,1) = (( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__));
    (*this)(5,3) =  cos__q_WFE__;
    (*this)(5,4) = - sin__q_WFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::Type_fr_Wrist_FE_X_fr_Shoulder_FE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(0,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(1,0) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,0) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(3,0) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(3,1) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(3,2) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(3,5) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(4,0) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,1) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(4,2) = (((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,3) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,0) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(5,1) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_WR__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(5,2) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_WR__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(5,3) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::Type_fr_Shoulder_FE_X_fr_ee()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(3,0) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(3,1) = (((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_EFE__)) *  cos__q_WR__);
    (*this)(3,2) = (((((( 0.03 *  sin__q_EFE__) *  cos__q_WFE__) + ( 0.296 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  cos__q_EFE__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(3,5) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(4,0) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(4,1) = (((((((- 0.02075 *  sin__q_HR__) *  sin__q_WFE__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_HR__)) *  sin__q_WR__) + (((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) + (( 0.03 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(4,2) = ((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) - (( 0.03 *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(4,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,0) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(5,1) = ((((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) + ( 0.03 *  cos__q_HR__)) *  sin__q_WR__) + (((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) + (( 0.03 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(5,2) = ((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((( 0.03 *  cos__q_HR__) *  cos__q_WFE__) + ((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::Type_fr_Humerus_R_X_fr_ee()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(3,0) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  sin__q_WFE__) + ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__) - (( 0.304 *  sin__q_EFE__) *  cos__q_WFE__));
    (*this)(3,1) = (((((- 0.02075 *  sin__q_WFE__) + (((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WFE__)) -  0.03) *  sin__q_WR__) + (( 0.304 *  sin__q_EFE__) *  sin__q_WFE__));
    (*this)(3,2) = ((((- 0.03 *  cos__q_WFE__) - ( 0.304 *  cos__q_EFE__)) -  0.296) *  cos__q_WR__);
    (*this)(3,3) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(3,4) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(3,5) = - sin__q_WR__;
    (*this)(4,0) = ((((( 0.296 *  cos__q_EFE__) +  0.304) *  sin__q_WFE__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,1) = ((((( 0.02075 *  cos__q_EFE__) *  sin__q_WFE__) + ((( 0.296 *  cos__q_EFE__) +  0.304) *  cos__q_WFE__)) + ( 0.03 *  cos__q_EFE__)) *  cos__q_WR__);
    (*this)(4,2) = (((((((- 0.03 *  cos__q_EFE__) *  cos__q_WFE__) - ( 0.296 *  cos__q_EFE__)) -  0.304) *  sin__q_WR__) + (( 0.03 *  sin__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  sin__q_EFE__));
    (*this)(4,3) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(4,4) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(4,5) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(5,0) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(5,1) = (((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_EFE__)) *  cos__q_WR__);
    (*this)(5,2) = (((((( 0.03 *  sin__q_EFE__) *  cos__q_WFE__) + ( 0.296 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  cos__q_EFE__));
    (*this)(5,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(5,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(5,5) = (- sin__q_EFE__ *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::Type_fr_Elbow_FE_X_fr_ee()
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(3,2) = (( 0.03 *  sin__q_WFE__) +  0.02075);
    (*this)(3,3) =  cos__q_WFE__;
    (*this)(3,4) = - sin__q_WFE__;
    (*this)(4,0) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(4,1) = (((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) +  0.03) *  cos__q_WR__);
    (*this)(4,2) = (((- 0.03 *  cos__q_WFE__) -  0.296) *  sin__q_WR__);
    (*this)(4,3) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(4,4) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(4,5) =  cos__q_WR__;
    (*this)(5,0) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(5,1) = (((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) +  0.03) *  sin__q_WR__);
    (*this)(5,2) = ((( 0.03 *  cos__q_WFE__) +  0.296) *  cos__q_WR__);
    (*this)(5,3) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(5,4) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(5,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::Type_fr_Wrist_R_X_fr_ee()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    (*this)(3,2) = ((- 0.03 *  cos__q_WFE__) -  0.184);
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,4) =  cos__q_WFE__;
    (*this)(4,0) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(4,1) = ((( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__)) +  0.03);
    (*this)(5,2) = (( 0.03 *  sin__q_WFE__) +  0.02075);
    (*this)(5,3) =  cos__q_WFE__;
    (*this)(5,4) = - sin__q_WFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::Type_fr_Wrist_FE_X_fr_ee()
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
    (*this)(4,2) = - 0.03;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.03;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::Type_fr_Shoulder_AA_X_fr_HyABase()
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
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(1,0) = - sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) =  sin__q_SAA__;
    (*this)(4,3) = - sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::Type_fr_Shoulder_FE_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(1,0) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(2,0) =  sin__q_SAA__;
    (*this)(2,1) = - cos__q_SAA__;
    (*this)(3,0) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(3,5) =  sin__q_SFE__;
    (*this)(4,0) = (( 0.178 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(4,1) = ((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(4,3) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) =  cos__q_SFE__;
    (*this)(5,0) = ( 0.178 *  cos__q_SAA__);
    (*this)(5,1) = ( 0.178 *  sin__q_SAA__);
    (*this)(5,3) =  sin__q_SAA__;
    (*this)(5,4) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::Type_fr_Humerus_R_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(1,0) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(2,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = ((((((- 0.112 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (( 0.112 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(4,0) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.112 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((( 0.112 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = ((- 0.112 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(4,3) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,0) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(5,1) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(5,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(5,4) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::Type_fr_Elbow_FE_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(1,0) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,0) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(2,1) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(4,0) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = (((((((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (((- 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(4,3) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,0) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,2) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(5,4) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::Type_fr_Wrist_R_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(1,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(2,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,1) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,2) = ((((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(3,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(4,0) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,1) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,2) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(4,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,0) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,1) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(5,2) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(5,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::Type_fr_Wrist_FE_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,1) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,2) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(4,0) = ((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,1) = ((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,2) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,0) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(5,1) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,2) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(5,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::Type_fr_ee_X_fr_HyABase()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,1) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,2) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(4,0) = ((((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,1) = ((((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,2) = ((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - (( 0.03 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,0) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(5,1) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,2) = ((((((((((( 0.03 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) - ((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(5,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::Type_fr_HyABase_X_fr_SAA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
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
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::Type_fr_HyABase_X_fr_SFE()
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
    (*this)(3,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,1) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(3,0) = (- 0.178 *  sin__q_SAA__);
    (*this)(3,2) = ( 0.178 *  cos__q_SAA__);
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,0) = ( 0.178 *  cos__q_SAA__);
    (*this)(4,2) = ( 0.178 *  sin__q_SAA__);
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,5) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::Type_fr_HyABase_X_fr_HR()
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
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = - sin__q_SAA__;
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(1,0) =  cos__q_SAA__;
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(3,0) = (((- 0.112 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.178 *  cos__q_SAA__));
    (*this)(3,1) = ((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.112 *  sin__q_SAA__));
    (*this)(3,2) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,3) = - sin__q_SAA__;
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,0) = (((- 0.112 *  sin__q_SAA__) *  sin__q_SFE__) - ( 0.178 *  sin__q_SAA__));
    (*this)(4,1) = (((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.112 *  cos__q_SAA__));
    (*this)(4,2) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,3) =  cos__q_SAA__;
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,0) = ( 0.112 *  cos__q_SFE__);
    (*this)(5,4) =  cos__q_SFE__;
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::Type_fr_HyABase_X_fr_EFE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(3,0) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(3,1) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.416 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,0) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(4,1) = (((((( 0.416 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.416 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,1) = ((- 0.416 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,2) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::Type_fr_HyABase_X_fr_WR()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,0) = ((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,2) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,0) = (((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = ((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,2) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,0) = ((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__));
    (*this)(5,1) = ((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,2) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::Type_fr_HyABase_X_fr_WFE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(3,0) = ((((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + ((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,1) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(3,2) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,0) = ((((((((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(4,1) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,2) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,0) = ((((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(5,1) = ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(5,2) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::Type_fr_Shoulder_FE_X_fr_Shoulder_AA()
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
    (*this)(5,0) = 0.178;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(1,0) = - sin__q_SFE__;
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(3,1) = ( 0.178 *  cos__q_SFE__);
    (*this)(3,3) =  cos__q_SFE__;
    (*this)(3,5) =  sin__q_SFE__;
    (*this)(4,1) = (- 0.178 *  sin__q_SFE__);
    (*this)(4,3) = - sin__q_SFE__;
    (*this)(4,5) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::Type_fr_Shoulder_AA_X_fr_Shoulder_FE()
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
    (*this)(3,2) = 0.178;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,1) = - sin__q_SFE__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,3) =  cos__q_SFE__;
    (*this)(3,4) = - sin__q_SFE__;
    (*this)(4,0) = ( 0.178 *  cos__q_SFE__);
    (*this)(4,1) = (- 0.178 *  sin__q_SFE__);
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::Type_fr_Humerus_R_X_fr_Shoulder_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,1) =  sin__q_HR__;
    (*this)(0,2) = - cos__q_HR__;
    (*this)(1,1) =  cos__q_HR__;
    (*this)(1,2) =  sin__q_HR__;
    (*this)(3,1) = ( 0.112 *  cos__q_HR__);
    (*this)(3,2) = ( 0.112 *  sin__q_HR__);
    (*this)(3,4) =  sin__q_HR__;
    (*this)(3,5) = - cos__q_HR__;
    (*this)(4,1) = (- 0.112 *  sin__q_HR__);
    (*this)(4,2) = ( 0.112 *  cos__q_HR__);
    (*this)(4,4) =  cos__q_HR__;
    (*this)(4,5) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::Type_fr_Shoulder_FE_X_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(1,0) =  sin__q_HR__;
    (*this)(1,1) =  cos__q_HR__;
    (*this)(2,0) = - cos__q_HR__;
    (*this)(2,1) =  sin__q_HR__;
    (*this)(4,0) = ( 0.112 *  cos__q_HR__);
    (*this)(4,1) = (- 0.112 *  sin__q_HR__);
    (*this)(4,3) =  sin__q_HR__;
    (*this)(4,4) =  cos__q_HR__;
    (*this)(5,0) = ( 0.112 *  sin__q_HR__);
    (*this)(5,1) = ( 0.112 *  cos__q_HR__);
    (*this)(5,3) = - cos__q_HR__;
    (*this)(5,4) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::Type_fr_Elbow_FE_X_fr_Humerus_R()
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.304;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,1) =  sin__q_EFE__;
    (*this)(0,2) =  cos__q_EFE__;
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(1,2) = - sin__q_EFE__;
    (*this)(3,0) = (- 0.304 *  sin__q_EFE__);
    (*this)(3,4) =  sin__q_EFE__;
    (*this)(3,5) =  cos__q_EFE__;
    (*this)(4,0) = (- 0.304 *  cos__q_EFE__);
    (*this)(4,4) =  cos__q_EFE__;
    (*this)(4,5) = - sin__q_EFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::Type_fr_Humerus_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1;
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
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.304;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(1,0) =  sin__q_EFE__;
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(2,0) =  cos__q_EFE__;
    (*this)(2,1) = - sin__q_EFE__;
    (*this)(3,0) = (- 0.304 *  sin__q_EFE__);
    (*this)(3,1) = (- 0.304 *  cos__q_EFE__);
    (*this)(4,3) =  sin__q_EFE__;
    (*this)(4,4) =  cos__q_EFE__;
    (*this)(5,3) =  cos__q_EFE__;
    (*this)(5,4) = - sin__q_EFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::Type_fr_Wrist_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,1) =  sin__q_WR__;
    (*this)(0,2) = - cos__q_WR__;
    (*this)(1,1) =  cos__q_WR__;
    (*this)(1,2) =  sin__q_WR__;
    (*this)(3,1) = ( 0.112 *  cos__q_WR__);
    (*this)(3,2) = ( 0.112 *  sin__q_WR__);
    (*this)(3,4) =  sin__q_WR__;
    (*this)(3,5) = - cos__q_WR__;
    (*this)(4,1) = (- 0.112 *  sin__q_WR__);
    (*this)(4,2) = ( 0.112 *  cos__q_WR__);
    (*this)(4,4) =  cos__q_WR__;
    (*this)(4,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::Type_fr_Elbow_FE_X_fr_Wrist_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(1,0) =  sin__q_WR__;
    (*this)(1,1) =  cos__q_WR__;
    (*this)(2,0) = - cos__q_WR__;
    (*this)(2,1) =  sin__q_WR__;
    (*this)(4,0) = ( 0.112 *  cos__q_WR__);
    (*this)(4,1) = (- 0.112 *  sin__q_WR__);
    (*this)(4,3) =  sin__q_WR__;
    (*this)(4,4) =  cos__q_WR__;
    (*this)(5,0) = ( 0.112 *  sin__q_WR__);
    (*this)(5,1) = ( 0.112 *  cos__q_WR__);
    (*this)(5,3) = - cos__q_WR__;
    (*this)(5,4) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::Type_fr_Wrist_FE_X_fr_Wrist_R()
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
    (*this)(5,0) = - 0.184;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.02075;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::MotionTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,2) =  cos__q_WFE__;
    (*this)(1,0) =  cos__q_WFE__;
    (*this)(1,2) = - sin__q_WFE__;
    (*this)(3,1) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,5) =  cos__q_WFE__;
    (*this)(4,1) = (( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__));
    (*this)(4,3) =  cos__q_WFE__;
    (*this)(4,5) = - sin__q_WFE__;
    return *this;
}

template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::Type_fr_HyABase_X_fr_Shoulder_AA()
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::Type_fr_HyABase_X_fr_Shoulder_FE()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(0,4) = (( 0.178 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(0,5) = ( 0.178 *  cos__q_SAA__);
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,4) = ((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(1,5) = ( 0.178 *  sin__q_SAA__);
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::Type_fr_HyABase_X_fr_Humerus_R()
{
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.112 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((((((- 0.112 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((( 0.112 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (( 0.112 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(2,4) = ((- 0.112 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::Type_fr_HyABase_X_fr_Elbow_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,4) = (((- 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,5) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::Type_fr_HyABase_X_fr_Wrist_R()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,4) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,5) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,4) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,5) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(2,4) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,5) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::Type_fr_HyABase_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,4) = ((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,5) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,4) = ((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,5) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(2,4) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(2,5) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,3) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::Type_fr_HyABase_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,4) = ((((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,5) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,4) = ((((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,5) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(2,4) = ((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - (( 0.03 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(2,5) = ((((((((((( 0.03 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) - ((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,3) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::Type_fr_HyABase_X_fr_Shoulder_AA_COM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 3.6E-4;
    (*this)(2,4) = - 2.0E-4;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(0,3) = (- 0.00309 *  sin__q_SAA__);
    (*this)(0,4) = (- 0.00309 *  cos__q_SAA__);
    (*this)(0,5) = ((- 2.0E-4 *  sin__q_SAA__) - ( 3.6E-4 *  cos__q_SAA__));
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(1,3) = ( 0.00309 *  cos__q_SAA__);
    (*this)(1,4) = (- 0.00309 *  sin__q_SAA__);
    (*this)(1,5) = (( 2.0E-4 *  cos__q_SAA__) - ( 3.6E-4 *  sin__q_SAA__));
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::Type_fr_HyABase_X_fr_Shoulder_FE_COM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = ((((- 0.00117 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.178 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.02338 *  sin__q_SAA__));
    (*this)(0,4) = (((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.00117 *  cos__q_SAA__) *  cos__q_SFE__)) - ( 0.00383 *  sin__q_SAA__));
    (*this)(0,5) = ((((- 0.00383 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02338 *  cos__q_SAA__) *  cos__q_SFE__)) + ( 0.178 *  cos__q_SAA__));
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = ((((- 0.00117 *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__)) - ( 0.02338 *  cos__q_SAA__));
    (*this)(1,4) = ((((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.00117 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.00383 *  cos__q_SAA__));
    (*this)(1,5) = ((((- 0.00383 *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02338 *  sin__q_SAA__) *  cos__q_SFE__)) + ( 0.178 *  sin__q_SAA__));
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,3) = ( 0.00117 *  cos__q_SFE__);
    (*this)(2,4) = (- 0.00117 *  sin__q_SFE__);
    (*this)(2,5) = (( 0.00383 *  cos__q_SFE__) - ( 0.02338 *  sin__q_SFE__));
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::Type_fr_HyABase_X_fr_Humerus_R_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = ((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.08646 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00618 *  cos__q_SAA__) *  cos__q_SFE__)) + (( 0.08646 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = ((((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.08646 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00239 *  cos__q_SAA__) *  cos__q_SFE__)) + (( 0.08646 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = ((((((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.178 *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (((((((- 0.08646 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00618 *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.08646 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = ((((((( 0.08646 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.00239 *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.08646 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00618 *  cos__q_HR__) - ( 0.00239 *  sin__q_HR__)) *  cos__q_SAA__));
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = ((( 0.08646 *  cos__q_HR__) *  cos__q_SFE__) - ( 0.00618 *  sin__q_SFE__));
    (*this)(2,4) = ((- 0.00239 *  sin__q_SFE__) - (( 0.08646 *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(2,5) = ((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::Type_fr_HyABase_X_fr_Elbow_FE_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((((( 0.416 *  sin__q_EFE__) +  0.01125) *  sin__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.178 *  cos__q_EFE__) *  sin__q_SAA__) - (( 0.00102 *  sin__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + (((( 0.00102 *  cos__q_EFE__) *  sin__q_HR__) + ((( 0.416 *  sin__q_EFE__) +  0.01125) *  cos__q_HR__)) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((((( 0.416 *  cos__q_EFE__) -  0.1466) *  sin__q_HR__) + (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_SFE__) + (((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + ((((( 0.416 *  cos__q_EFE__) -  0.1466) *  cos__q_HR__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (((((((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  cos__q_SAA__) *  cos__q_SFE__)) + (((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (((((((((( 0.416 *  sin__q_EFE__) +  0.01125) *  sin__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_SAA__)) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + (((((- 0.416 *  sin__q_EFE__) -  0.01125) *  cos__q_HR__) - (( 0.00102 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__));
    (*this)(1,4) = (((((((((( 0.416 *  cos__q_EFE__) -  0.1466) *  sin__q_HR__) + (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.00102 *  cos__q_EFE__) *  sin__q_SAA__) - (( 0.178 *  sin__q_EFE__) *  cos__q_SAA__)) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + (((( 0.00102 *  sin__q_EFE__) *  sin__q_HR__) + (( 0.1466 - ( 0.416 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(1,5) = (((((((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = ((((((- 0.416 *  sin__q_EFE__) -  0.01125) *  sin__q_HR__) + (( 0.00102 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.00102 *  sin__q_EFE__) *  sin__q_SFE__));
    (*this)(2,4) = ((((( 0.1466 - ( 0.416 *  cos__q_EFE__)) *  sin__q_HR__) - (( 0.00102 *  sin__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.00102 *  cos__q_EFE__) *  sin__q_SFE__));
    (*this)(2,5) = ((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_SFE__) + (((((- 0.01125 *  sin__q_EFE__) + ( 0.1466 *  cos__q_EFE__)) -  0.416) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::Type_fr_HyABase_X_fr_Wrist_R_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.08883 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.00261 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,4) = (((((((((((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.08883 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.08883) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 4.0E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,5) = ((((((((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  cos__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + ((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.08883) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.08883 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.00261 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.00261 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((((((((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.08883 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.08883) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.08883) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 4.0E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((((((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.00261 *  cos__q_EFE__) *  sin__q_HR__) + ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((((((- 0.416 *  cos__q_EFE__) -  0.08883) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.08883 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__)) + (( 0.00261 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,4) = ((((((( 0.08883 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.08883 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.08883) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 4.0E-4 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,5) = (((((( 0.00261 *  sin__q_EFE__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  sin__q_WR__) + (((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SFE__) + (((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,3) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::Type_fr_HyABase_X_fr_Wrist_FE_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = (((((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,4) = (((((((((((((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - ((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,5) = ((((((((((((((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((- 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = ((((((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.01063 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,4) = (((((((((((((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (( 0.07876 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + (((((((- 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,5) = ((((((((((((((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((- 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.07876 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.01063 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) + (((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(2,4) = (((((((((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SFE__) - (( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + (( 0.07876 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,5) = ((((((((((((( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) + ((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.01063 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.07876 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + ((((- 0.07876 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + ((((- 0.01063 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,3) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(5,4) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::Type_fr_HyABase_X_fr_Shoulder_AA_CTR()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(0,3) = (- 0.089 *  sin__q_SAA__);
    (*this)(0,4) = (- 0.089 *  cos__q_SAA__);
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(1,3) = ( 0.089 *  cos__q_SAA__);
    (*this)(1,4) = (- 0.089 *  sin__q_SAA__);
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) = - sin__q_SAA__;
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::Type_fr_HyABase_X_fr_Shoulder_FE_CTR()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(0,4) = ((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.056 *  sin__q_SAA__));
    (*this)(0,5) = ((( 0.056 *  cos__q_SAA__) *  sin__q_SFE__) + ( 0.178 *  cos__q_SAA__));
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,4) = (((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.056 *  cos__q_SAA__));
    (*this)(1,5) = ((( 0.056 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.178 *  sin__q_SAA__));
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,5) = (- 0.056 *  cos__q_SFE__);
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = - cos__q_SAA__;
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::Type_fr_HyABase_X_fr_Humerus_R_CTR()
{
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.264 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.264 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.264 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.264 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = ((((((- 0.264 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.264 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((( 0.264 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.264 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (( 0.264 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(2,4) = ((- 0.264 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::Type_fr_HyABase_X_fr_Elbow_FE_CTR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.056) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.056) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = ((((((((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.056 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = ((((((((( 0.416 *  cos__q_EFE__) +  0.056) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.056) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((((((((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.056 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,4) = ((((- 0.416 *  cos__q_EFE__) -  0.056) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,5) = ((( 0.056 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.056 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::Type_fr_HyABase_X_fr_Wrist_R_CTR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,4) = ((((((((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.204) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,5) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.204) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.204 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,4) = ((((((((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.204) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.204) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,5) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((((- 0.416 *  cos__q_EFE__) -  0.204) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.204 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(2,4) = ((((( 0.204 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.204 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.204) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,5) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::Type_fr_Shoulder_FE_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(0,3) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(0,4) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(0,5) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(1,3) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(1,4) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(1,5) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_WR__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(2,4) = (((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(2,5) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_WR__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(3,5) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(4,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::Type_fr_Humerus_R_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(0,3) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  sin__q_WFE__) + ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__) - (( 0.304 *  sin__q_EFE__) *  cos__q_WFE__));
    (*this)(0,4) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WFE__) - ( 0.02075 *  sin__q_WFE__)) *  sin__q_WR__) + (( 0.304 *  sin__q_EFE__) *  sin__q_WFE__));
    (*this)(0,5) = (((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WR__);
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(1,3) = ((((( 0.296 *  cos__q_EFE__) +  0.304) *  sin__q_WFE__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,4) = (((( 0.02075 *  cos__q_EFE__) *  sin__q_WFE__) + ((( 0.296 *  cos__q_EFE__) +  0.304) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,5) = ((((- 0.296 *  cos__q_EFE__) -  0.304) *  sin__q_WR__) + ( 0.02075 *  sin__q_EFE__));
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,3) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(2,4) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(2,5) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(3,3) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(3,4) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(3,5) = - sin__q_WR__;
    (*this)(4,3) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(4,4) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(4,5) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(5,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(5,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(5,5) = (- sin__q_EFE__ *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::Type_fr_Elbow_FE_X_fr_Wrist_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.02075;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(1,3) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,4) = ((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,5) = (- 0.296 *  sin__q_WR__);
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(2,3) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(2,4) = ((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(2,5) = ( 0.296 *  cos__q_WR__);
    (*this)(3,3) =  cos__q_WFE__;
    (*this)(3,4) = - sin__q_WFE__;
    (*this)(4,3) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(4,4) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(4,5) =  cos__q_WR__;
    (*this)(5,3) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(5,4) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(5,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::Type_fr_Wrist_R_X_fr_Wrist_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.184;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.02075;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(1,3) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(1,4) = (( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__));
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,4) =  cos__q_WFE__;
    (*this)(5,3) =  cos__q_WFE__;
    (*this)(5,4) = - sin__q_WFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::Type_fr_Wrist_FE_X_fr_Shoulder_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(0,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(0,3) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(0,4) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(0,5) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,0) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(1,3) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,4) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__) - (( 0.02075 *  sin__q_HR__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(1,5) = (((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(2,0) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = ((( 0.296 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.02075 *  cos__q_EFE__));
    (*this)(2,4) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_WR__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(2,5) = ((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_WR__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WR__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(3,5) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(4,3) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,3) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::Type_fr_Shoulder_FE_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(0,3) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(0,4) = (((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_EFE__)) *  cos__q_WR__);
    (*this)(0,5) = (((((( 0.03 *  sin__q_EFE__) *  cos__q_WFE__) + ( 0.296 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  cos__q_EFE__));
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(1,3) = ((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(1,4) = (((((((- 0.02075 *  sin__q_HR__) *  sin__q_WFE__) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_HR__)) *  sin__q_WR__) + (((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_WFE__)) + (( 0.03 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(1,5) = ((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) - (( 0.03 *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__)) + (( 0.02075 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = (((((((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_WFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_WFE__) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  cos__q_WR__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(2,4) = ((((((( 0.02075 *  cos__q_HR__) *  sin__q_WFE__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  cos__q_WFE__)) + ( 0.03 *  cos__q_HR__)) *  sin__q_WR__) + (((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_WFE__)) + (( 0.03 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(2,5) = ((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) *  sin__q_WR__) + (((( 0.03 *  cos__q_HR__) *  cos__q_WFE__) + ((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__)) + (( 0.02075 *  sin__q_EFE__) *  sin__q_HR__));
    (*this)(3,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(3,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(3,5) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(4,3) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(4,4) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(5,3) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(5,4) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(5,5) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::Type_fr_Humerus_R_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(0,3) = ((((((- 0.304 *  cos__q_EFE__) -  0.296) *  sin__q_WFE__) + ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__) - (( 0.304 *  sin__q_EFE__) *  cos__q_WFE__));
    (*this)(0,4) = (((((- 0.02075 *  sin__q_WFE__) + (((- 0.304 *  cos__q_EFE__) -  0.296) *  cos__q_WFE__)) -  0.03) *  sin__q_WR__) + (( 0.304 *  sin__q_EFE__) *  sin__q_WFE__));
    (*this)(0,5) = ((((- 0.03 *  cos__q_WFE__) - ( 0.304 *  cos__q_EFE__)) -  0.296) *  cos__q_WR__);
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(1,3) = ((((( 0.296 *  cos__q_EFE__) +  0.304) *  sin__q_WFE__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,4) = ((((( 0.02075 *  cos__q_EFE__) *  sin__q_WFE__) + ((( 0.296 *  cos__q_EFE__) +  0.304) *  cos__q_WFE__)) + ( 0.03 *  cos__q_EFE__)) *  cos__q_WR__);
    (*this)(1,5) = (((((((- 0.03 *  cos__q_EFE__) *  cos__q_WFE__) - ( 0.296 *  cos__q_EFE__)) -  0.304) *  sin__q_WR__) + (( 0.03 *  sin__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  sin__q_EFE__));
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,3) = (((( 0.02075 *  sin__q_EFE__) *  cos__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_WFE__)) *  cos__q_WR__);
    (*this)(2,4) = (((((- 0.02075 *  sin__q_EFE__) *  sin__q_WFE__) - (( 0.296 *  sin__q_EFE__) *  cos__q_WFE__)) - ( 0.03 *  sin__q_EFE__)) *  cos__q_WR__);
    (*this)(2,5) = (((((( 0.03 *  sin__q_EFE__) *  cos__q_WFE__) + ( 0.296 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  sin__q_WFE__)) + ( 0.02075 *  cos__q_EFE__));
    (*this)(3,3) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(3,4) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(3,5) = - sin__q_WR__;
    (*this)(4,3) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(4,4) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(4,5) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(5,3) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(5,4) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(5,5) = (- sin__q_EFE__ *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::Type_fr_Elbow_FE_X_fr_ee()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(0,5) = (( 0.03 *  sin__q_WFE__) +  0.02075);
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(1,3) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  cos__q_WR__);
    (*this)(1,4) = (((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) +  0.03) *  cos__q_WR__);
    (*this)(1,5) = (((- 0.03 *  cos__q_WFE__) -  0.296) *  sin__q_WR__);
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(2,3) = ((( 0.296 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__)) *  sin__q_WR__);
    (*this)(2,4) = (((( 0.02075 *  sin__q_WFE__) + ( 0.296 *  cos__q_WFE__)) +  0.03) *  sin__q_WR__);
    (*this)(2,5) = ((( 0.03 *  cos__q_WFE__) +  0.296) *  cos__q_WR__);
    (*this)(3,3) =  cos__q_WFE__;
    (*this)(3,4) = - sin__q_WFE__;
    (*this)(4,3) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(4,4) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(4,5) =  cos__q_WR__;
    (*this)(5,3) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(5,4) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(5,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::Type_fr_Wrist_R_X_fr_ee()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(0,5) = ((- 0.03 *  cos__q_WFE__) -  0.184);
    (*this)(1,3) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(1,4) = ((( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__)) +  0.03);
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    (*this)(2,5) = (( 0.03 *  sin__q_WFE__) +  0.02075);
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,4) =  cos__q_WFE__;
    (*this)(5,3) =  cos__q_WFE__;
    (*this)(5,4) = - sin__q_WFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::Type_fr_Wrist_FE_X_fr_ee()
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
    (*this)(1,5) = - 0.03;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.03;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::Type_fr_Shoulder_AA_X_fr_HyABase()
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(1,0) = - sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,4) =  sin__q_SAA__;
    (*this)(4,3) = - sin__q_SAA__;
    (*this)(4,4) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::Type_fr_Shoulder_FE_X_fr_HyABase()
{
    (*this)(0,5) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(0,3) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(0,4) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(1,3) = (( 0.178 *  sin__q_SAA__) *  sin__q_SFE__);
    (*this)(1,4) = ((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__);
    (*this)(2,0) =  sin__q_SAA__;
    (*this)(2,1) = - cos__q_SAA__;
    (*this)(2,3) = ( 0.178 *  cos__q_SAA__);
    (*this)(2,4) = ( 0.178 *  sin__q_SAA__);
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(3,5) =  sin__q_SFE__;
    (*this)(4,3) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) =  cos__q_SFE__;
    (*this)(5,3) =  sin__q_SAA__;
    (*this)(5,4) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::Type_fr_Humerus_R_X_fr_HyABase()
{
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(0,3) = (((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = ((((((- 0.112 *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (( 0.112 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(1,0) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(1,3) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.112 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.112 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((( 0.112 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.112 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = ((- 0.112 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(2,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,4) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(4,3) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(5,4) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::Type_fr_Elbow_FE_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(0,3) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(1,0) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(1,3) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = (((((((( 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (((- 0.416 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,0) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(2,1) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,4) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,5) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(3,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(4,3) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,3) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(5,4) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::Type_fr_Wrist_R_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(0,3) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,4) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,5) = ((((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(1,3) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,4) = ((((((((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,5) = ((((( 0.112 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(2,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,4) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,5) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(3,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(4,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(4,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(5,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(5,4) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::Type_fr_Wrist_FE_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(0,3) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,4) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,5) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(1,3) = ((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,4) = ((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,5) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(2,4) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,5) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(5,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::Type_fr_ee_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(0,3) = ((((((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,4) = (((((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__)) *  sin__q_WR__) + ((((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) *  cos__q_WR__)) + ((((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,5) = (((((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  sin__q_WR__) + (((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) *  cos__q_WR__)) - (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(1,3) = ((((((((((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((((- 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,4) = ((((((((((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,5) = ((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) - (( 0.03 *  sin__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WFE__)) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) + ((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(2,4) = ((((((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) + (((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,5) = ((((((((((( 0.03 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__) + (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) - ((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(3,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(4,3) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,4) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(4,5) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(5,3) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(5,4) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::Type_fr_HyABase_X_fr_SAA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
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
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::Type_fr_HyABase_X_fr_SFE()
{
    (*this)(0,1) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = (- 0.178 *  sin__q_SAA__);
    (*this)(0,5) = ( 0.178 *  cos__q_SAA__);
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = ( 0.178 *  cos__q_SAA__);
    (*this)(1,5) = ( 0.178 *  sin__q_SAA__);
    (*this)(3,3) =  cos__q_SAA__;
    (*this)(3,5) =  sin__q_SAA__;
    (*this)(4,3) =  sin__q_SAA__;
    (*this)(4,5) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::Type_fr_HyABase_X_fr_HR()
{
    (*this)(2,0) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = - sin__q_SAA__;
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (((- 0.112 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.178 *  cos__q_SAA__));
    (*this)(0,4) = ((( 0.178 *  sin__q_SAA__) *  sin__q_SFE__) + ( 0.112 *  sin__q_SAA__));
    (*this)(0,5) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) =  cos__q_SAA__;
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (((- 0.112 *  sin__q_SAA__) *  sin__q_SFE__) - ( 0.178 *  sin__q_SAA__));
    (*this)(1,4) = (((- 0.178 *  cos__q_SAA__) *  sin__q_SFE__) - ( 0.112 *  cos__q_SAA__));
    (*this)(1,5) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = ( 0.112 *  cos__q_SFE__);
    (*this)(3,3) = - sin__q_SAA__;
    (*this)(3,4) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(3,5) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(4,3) =  cos__q_SAA__;
    (*this)(4,4) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(4,5) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(5,4) =  cos__q_SFE__;
    (*this)(5,5) =  sin__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::Type_fr_HyABase_X_fr_EFE()
{
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = ((- 0.178 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(0,4) = (((((( 0.178 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.416 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (((((( 0.416 *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) - (( 0.416 *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (( 0.178 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,4) = (((((( 0.416 *  sin__q_HR__) *  sin__q_SAA__) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.416 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (((((( 0.416 *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (( 0.416 *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,4) = ((- 0.416 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,5) = ((- 0.416 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(3,4) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(3,5) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(4,3) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(4,4) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(4,5) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(5,5) = (- sin__q_HR__ *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::Type_fr_HyABase_X_fr_WR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = ((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.112) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,5) = (((((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = (((((((((- 0.112 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.112 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.112 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = ((((((((( 0.416 *  cos__q_EFE__) +  0.112) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.112) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,5) = (((((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.112 *  sin__q_EFE__) *  sin__q_SFE__));
    (*this)(2,4) = ((((- 0.416 *  cos__q_EFE__) -  0.112) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(2,5) = (((- 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(3,3) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(3,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(4,3) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(4,4) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,5) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(5,3) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(5,4) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(5,5) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::Type_fr_HyABase_X_fr_WFE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = ((((((((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + ((((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) - ((( 0.178 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,4) = (((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.178 *  sin__q_HR__) *  sin__q_SAA__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  sin__q_SAA__)) - (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(0,5) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SAA__) - (( 0.178 *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  sin__q_SAA__)) + (( 0.178 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.416 *  cos__q_EFE__) +  0.296) *  cos__q_HR__) *  sin__q_SAA__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = ((((((((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WR__) + ((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + ((((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) - ((( 0.416 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__));
    (*this)(1,4) = ((((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  sin__q_SAA__) - (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,5) = (((((((((((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  sin__q_SAA__) + (( 0.178 *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) + ((( 0.296 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (( 0.178 *  cos__q_HR__) *  sin__q_SAA__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_HR__) *  sin__q_SAA__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.178 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) + ((((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = ((((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) - ((( 0.416 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SFE__));
    (*this)(2,4) = ((((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + (((((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.296 *  sin__q_EFE__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(2,5) = ((((((( 0.296 *  sin__q_EFE__) *  sin__q_SFE__) + ((((- 0.296 *  cos__q_EFE__) -  0.416) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + (((((- 0.416 *  cos__q_EFE__) -  0.296) *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + (( 0.02075 *  cos__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(3,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(3,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(3,5) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(4,3) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(4,4) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(4,5) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(5,3) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(5,4) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(5,5) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::Type_fr_Shoulder_FE_X_fr_Shoulder_AA()
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
    (*this)(2,3) = 0.178;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(0,4) = ( 0.178 *  cos__q_SFE__);
    (*this)(1,0) = - sin__q_SFE__;
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(1,4) = (- 0.178 *  sin__q_SFE__);
    (*this)(3,3) =  cos__q_SFE__;
    (*this)(3,5) =  sin__q_SFE__;
    (*this)(4,3) = - sin__q_SFE__;
    (*this)(4,5) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::Type_fr_Shoulder_AA_X_fr_Shoulder_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.178;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,1) = - sin__q_SFE__;
    (*this)(1,3) = ( 0.178 *  cos__q_SFE__);
    (*this)(1,4) = (- 0.178 *  sin__q_SFE__);
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(3,3) =  cos__q_SFE__;
    (*this)(3,4) = - sin__q_SFE__;
    (*this)(5,3) =  sin__q_SFE__;
    (*this)(5,4) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::Type_fr_Humerus_R_X_fr_Shoulder_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,1) =  sin__q_HR__;
    (*this)(0,2) = - cos__q_HR__;
    (*this)(0,4) = ( 0.112 *  cos__q_HR__);
    (*this)(0,5) = ( 0.112 *  sin__q_HR__);
    (*this)(1,1) =  cos__q_HR__;
    (*this)(1,2) =  sin__q_HR__;
    (*this)(1,4) = (- 0.112 *  sin__q_HR__);
    (*this)(1,5) = ( 0.112 *  cos__q_HR__);
    (*this)(3,4) =  sin__q_HR__;
    (*this)(3,5) = - cos__q_HR__;
    (*this)(4,4) =  cos__q_HR__;
    (*this)(4,5) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::Type_fr_Shoulder_FE_X_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(1,0) =  sin__q_HR__;
    (*this)(1,1) =  cos__q_HR__;
    (*this)(1,3) = ( 0.112 *  cos__q_HR__);
    (*this)(1,4) = (- 0.112 *  sin__q_HR__);
    (*this)(2,0) = - cos__q_HR__;
    (*this)(2,1) =  sin__q_HR__;
    (*this)(2,3) = ( 0.112 *  sin__q_HR__);
    (*this)(2,4) = ( 0.112 *  cos__q_HR__);
    (*this)(4,3) =  sin__q_HR__;
    (*this)(4,4) =  cos__q_HR__;
    (*this)(5,3) = - cos__q_HR__;
    (*this)(5,4) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::Type_fr_Elbow_FE_X_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.304;
    (*this)(2,5) = 0;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,1) =  sin__q_EFE__;
    (*this)(0,2) =  cos__q_EFE__;
    (*this)(0,3) = (- 0.304 *  sin__q_EFE__);
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(1,2) = - sin__q_EFE__;
    (*this)(1,3) = (- 0.304 *  cos__q_EFE__);
    (*this)(3,4) =  sin__q_EFE__;
    (*this)(3,5) =  cos__q_EFE__;
    (*this)(4,4) =  cos__q_EFE__;
    (*this)(4,5) = - sin__q_EFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::Type_fr_Humerus_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.304;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,3) = (- 0.304 *  sin__q_EFE__);
    (*this)(0,4) = (- 0.304 *  cos__q_EFE__);
    (*this)(1,0) =  sin__q_EFE__;
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(2,0) =  cos__q_EFE__;
    (*this)(2,1) = - sin__q_EFE__;
    (*this)(4,3) =  sin__q_EFE__;
    (*this)(4,4) =  cos__q_EFE__;
    (*this)(5,3) =  cos__q_EFE__;
    (*this)(5,4) = - sin__q_EFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::Type_fr_Wrist_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,1) =  sin__q_WR__;
    (*this)(0,2) = - cos__q_WR__;
    (*this)(0,4) = ( 0.112 *  cos__q_WR__);
    (*this)(0,5) = ( 0.112 *  sin__q_WR__);
    (*this)(1,1) =  cos__q_WR__;
    (*this)(1,2) =  sin__q_WR__;
    (*this)(1,4) = (- 0.112 *  sin__q_WR__);
    (*this)(1,5) = ( 0.112 *  cos__q_WR__);
    (*this)(3,4) =  sin__q_WR__;
    (*this)(3,5) = - cos__q_WR__;
    (*this)(4,4) =  cos__q_WR__;
    (*this)(4,5) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::Type_fr_Elbow_FE_X_fr_Wrist_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(1,0) =  sin__q_WR__;
    (*this)(1,1) =  cos__q_WR__;
    (*this)(1,3) = ( 0.112 *  cos__q_WR__);
    (*this)(1,4) = (- 0.112 *  sin__q_WR__);
    (*this)(2,0) = - cos__q_WR__;
    (*this)(2,1) =  sin__q_WR__;
    (*this)(2,3) = ( 0.112 *  sin__q_WR__);
    (*this)(2,4) = ( 0.112 *  cos__q_WR__);
    (*this)(4,3) =  sin__q_WR__;
    (*this)(4,4) =  cos__q_WR__;
    (*this)(5,3) = - cos__q_WR__;
    (*this)(5,4) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::Type_fr_Wrist_FE_X_fr_Wrist_R()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.184;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.02075;
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
const typename iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::ForceTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,2) =  cos__q_WFE__;
    (*this)(0,4) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    (*this)(1,0) =  cos__q_WFE__;
    (*this)(1,2) = - sin__q_WFE__;
    (*this)(1,4) = (( 0.02075 *  sin__q_WFE__) + ( 0.184 *  cos__q_WFE__));
    (*this)(3,3) =  sin__q_WFE__;
    (*this)(3,5) =  cos__q_WFE__;
    (*this)(4,3) =  cos__q_WFE__;
    (*this)(4,5) = - sin__q_WFE__;
    return *this;
}

template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::Type_fr_HyABase_X_fr_Shoulder_AA()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
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
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::Type_fr_HyABase_X_fr_Shoulder_FE()
{
    (*this)(0,3) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.178;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::Type_fr_HyABase_X_fr_Humerus_R()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (( 0.112 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (( 0.112 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (( 0.112 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::Type_fr_HyABase_X_fr_Elbow_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (( 0.416 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (( 0.416 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (( 0.416 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::Type_fr_HyABase_X_fr_Wrist_R()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = ((((((- 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = ((((((- 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::Type_fr_HyABase_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = (((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = (((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WR__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::Type_fr_HyABase_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = ((((((((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((((((- 0.03 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.03 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = ((((((((((((((((- 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.03 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) - (((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((((((( 0.03 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.03 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (( 0.02075 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.03 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + ((( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = ((((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = ((((((((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.03 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WR__) + ((((( 0.03 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) + (( 0.02075 *  sin__q_HR__) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.03 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::Type_fr_HyABase_X_fr_Shoulder_AA_COM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.00309;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_COM::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(0,3) = (( 3.6E-4 *  sin__q_SAA__) - ( 2.0E-4 *  cos__q_SAA__));
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    (*this)(1,3) = ((- 2.0E-4 *  sin__q_SAA__) - ( 3.6E-4 *  cos__q_SAA__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::Type_fr_HyABase_X_fr_Shoulder_FE_COM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = (((( 0.02338 *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.00383 *  cos__q_SAA__) *  cos__q_SFE__)) + ( 0.00117 *  sin__q_SAA__));
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = (((( 0.02338 *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.00383 *  sin__q_SAA__) *  cos__q_SFE__)) - ( 0.00117 *  cos__q_SAA__));
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,3) = (((- 0.00383 *  sin__q_SFE__) - ( 0.02338 *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::Type_fr_HyABase_X_fr_Humerus_R_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_COM::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (((((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.08646 *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.00618 *  sin__q_HR__) + ( 0.00239 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (((((( 0.00239 *  sin__q_HR__) - ( 0.00618 *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + (( 0.08646 *  sin__q_SAA__) *  cos__q_SFE__)) + (((- 0.00618 *  sin__q_HR__) - ( 0.00239 *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = ((( 0.08646 *  sin__q_SFE__) + ((( 0.00618 *  cos__q_HR__) - ( 0.00239 *  sin__q_HR__)) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::Type_fr_HyABase_X_fr_Elbow_FE_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_COM::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (((((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + (((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) + ( 0.00102 *  cos__q_HR__)) *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (((((( 0.00102 *  sin__q_HR__) + ((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) + ((((( 0.1466 *  sin__q_EFE__) + ( 0.01125 *  cos__q_EFE__)) *  sin__q_HR__) - ( 0.00102 *  cos__q_HR__)) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (((((( 0.01125 *  sin__q_EFE__) - ( 0.1466 *  cos__q_EFE__)) +  0.416) *  sin__q_SFE__) + (((((- 0.1466 *  sin__q_EFE__) - ( 0.01125 *  cos__q_EFE__)) *  cos__q_HR__) - ( 0.00102 *  sin__q_HR__)) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::Type_fr_HyABase_X_fr_Wrist_R_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = (((((((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.00261 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 0.00261 *  cos__q_EFE__) *  sin__q_HR__) - ( 4.0E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = (((((((((((- 0.00261 *  sin__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 4.0E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.00261 *  cos__q_HR__) - (( 4.0E-4 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((( 0.00261 *  cos__q_EFE__) *  cos__q_HR__) - ( 4.0E-4 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.00261 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 0.00261 *  cos__q_EFE__) *  sin__q_HR__) + ( 4.0E-4 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) - (((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.08883 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((((((( 0.00261 *  sin__q_HR__) + (( 4.0E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__) - (( 4.0E-4 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WR__) + (((( 0.00261 *  sin__q_EFE__) *  sin__q_SFE__) + ((( 4.0E-4 *  sin__q_HR__) - (( 0.00261 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__)) + ((( 0.08883 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__)) + ((( 0.08883 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::Type_fr_HyABase_X_fr_Wrist_FE_COM()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_FE_COM::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = (((((((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + (((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) - ( 8.4E-4 *  cos__q_HR__)) *  sin__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.07876 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.07876 *  cos__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) + ((((( 0.01063 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( 0.01063 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) + ((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  cos__q_SAA__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((((- 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) - ( 0.02075 *  cos__q_HR__)) *  sin__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) - ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + ((((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + ((((((( 1.0 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = (((((((((((((((((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((((- 8.4E-4 *  sin__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 8.4E-4 *  cos__q_HR__) - (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  cos__q_SAA__)) *  sin__q_WR__) + (((((((((( 0.07876 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.07876 *  cos__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) + ((((( 0.01063 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( 0.01063 *  cos__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) + ((((( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__) - ( 0.02075 *  sin__q_HR__)) *  sin__q_SAA__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + (((( 8.4E-4 *  cos__q_EFE__) *  sin__q_HR__) + ( 0.02075 *  cos__q_HR__)) *  cos__q_SAA__)) *  cos__q_WR__)) + (((((((- 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + ((( 0.01063 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.01063 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__)) + ((((((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) + ((( 0.07876 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(2,1) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((((((((((((( 0.07876 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__) + (((( 0.01063 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.01063 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 8.4E-4 *  sin__q_HR__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  sin__q_WR__) + (((((((- 0.07876 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) - ((( 0.01063 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__)) + (( 8.4E-4 *  sin__q_EFE__) *  sin__q_SFE__)) + ((( 0.02075 *  sin__q_HR__) - (( 8.4E-4 *  cos__q_EFE__) *  cos__q_HR__)) *  cos__q_SFE__)) *  cos__q_WR__)) + (((( 0.01063 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.01063 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__)) + ((((- 0.07876 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.07876 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::Type_fr_HyABase_X_fr_Shoulder_AA_CTR()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0.089;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_AA_CTR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) = - sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::Type_fr_HyABase_X_fr_Shoulder_FE_CTR()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Shoulder_FE_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(0,3) = (( 0.056 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = - cos__q_SAA__;
    (*this)(1,3) = (( 0.056 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,3) = (( 0.056 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::Type_fr_HyABase_X_fr_Humerus_R_CTR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Humerus_R_CTR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (( 0.264 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (( 0.264 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (( 0.264 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::Type_fr_HyABase_X_fr_Elbow_FE_CTR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Elbow_FE_CTR::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = ((((((- 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.056 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = ((((((- 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.056 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.056 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = ((((( 0.056 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__) + ((( 0.056 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::Type_fr_HyABase_X_fr_Wrist_R_CTR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_Wrist_R_CTR::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = ((((((- 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = ((((((- 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.204 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((( 0.204 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__) + ((( 0.204 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::Type_fr_Shoulder_FE_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(0,3) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.296 *  cos__q_EFE__)) +  0.416);
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(1,3) = ((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WR__) + (( 0.02075 *  sin__q_HR__) *  cos__q_WR__)) + (( 0.296 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = ((((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WR__) - (( 0.02075 *  cos__q_HR__) *  cos__q_WR__)) + (( 0.296 *  sin__q_EFE__) *  sin__q_HR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::Type_fr_Humerus_R_X_fr_Wrist_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(0,3) = ( 0.02075 *  cos__q_WR__);
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(1,3) = ((( 0.02075 *  cos__q_EFE__) *  sin__q_WR__) + ( 0.296 *  sin__q_EFE__));
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,3) = ((((- 0.02075 *  sin__q_EFE__) *  sin__q_WR__) + ( 0.296 *  cos__q_EFE__)) +  0.304);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::Type_fr_Elbow_FE_X_fr_Wrist_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.296;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(1,3) = ( 0.02075 *  sin__q_WR__);
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(2,3) = (- 0.02075 *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::Type_fr_Wrist_R_X_fr_Wrist_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.02075;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.184;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Wrist_FE::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::Type_fr_Wrist_FE_X_fr_Shoulder_FE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(0,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(0,3) = ((((( 0.416 *  sin__q_EFE__) *  sin__q_WFE__) *  sin__q_WR__) - ( 0.02075 *  sin__q_WFE__)) + (((- 0.416 *  cos__q_EFE__) -  0.296) *  cos__q_WFE__));
    (*this)(1,0) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(1,3) = ((((( 0.416 *  sin__q_EFE__) *  cos__q_WFE__) *  sin__q_WR__) + ((( 0.416 *  cos__q_EFE__) +  0.296) *  sin__q_WFE__)) - ( 0.02075 *  cos__q_WFE__));
    (*this)(2,0) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = (( 0.416 *  sin__q_EFE__) *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::Type_fr_Shoulder_FE_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_HR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(0,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(0,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(0,3) = (((((((- 0.03 *  sin__q_EFE__) *  sin__q_WFE__) - ( 0.02075 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  cos__q_WFE__)) + ( 0.296 *  cos__q_EFE__)) +  0.416);
    (*this)(1,0) = ((((( cos__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__));
    (*this)(1,1) = ((((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  cos__q_HR__) *  sin__q_WFE__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_WR__) - ( sin__q_HR__ *  sin__q_WR__));
    (*this)(1,3) = (((((((( 0.03 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  cos__q_EFE__) *  cos__q_HR__)) *  sin__q_WR__) + (((( 0.03 *  sin__q_HR__) *  sin__q_WFE__) + ( 0.02075 *  sin__q_HR__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_WFE__)) + (( 0.296 *  sin__q_EFE__) *  cos__q_HR__));
    (*this)(2,0) = ((((( cos__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  sin__q_WFE__) *  cos__q_WR__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__));
    (*this)(2,1) = ((((( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WFE__) *  sin__q_WR__) - (( cos__q_HR__ *  cos__q_WFE__) *  cos__q_WR__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_WFE__));
    (*this)(2,2) = (( cos__q_HR__ *  sin__q_WR__) + (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_WR__));
    (*this)(2,3) = (((((((( 0.03 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_WFE__) + (( 0.02075 *  cos__q_EFE__) *  sin__q_HR__)) *  sin__q_WR__) + ((((- 0.03 *  cos__q_HR__) *  sin__q_WFE__) - ( 0.02075 *  cos__q_HR__)) *  cos__q_WR__)) + ((( 0.03 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_WFE__)) + (( 0.296 *  sin__q_EFE__) *  sin__q_HR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::Type_fr_Humerus_R_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,0) = ( sin__q_WFE__ *  cos__q_WR__);
    (*this)(0,1) = ( cos__q_WFE__ *  cos__q_WR__);
    (*this)(0,2) = - sin__q_WR__;
    (*this)(0,3) = ((( 0.03 *  sin__q_WFE__) +  0.02075) *  cos__q_WR__);
    (*this)(1,0) = ((( cos__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__) + ( sin__q_EFE__ *  cos__q_WFE__));
    (*this)(1,1) = ((( cos__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( sin__q_EFE__ *  sin__q_WFE__));
    (*this)(1,2) = ( cos__q_EFE__ *  cos__q_WR__);
    (*this)(1,3) = (((((( 0.03 *  cos__q_EFE__) *  sin__q_WFE__) + ( 0.02075 *  cos__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  sin__q_EFE__) *  cos__q_WFE__)) + ( 0.296 *  sin__q_EFE__));
    (*this)(2,0) = (( cos__q_EFE__ *  cos__q_WFE__) - (( sin__q_EFE__ *  sin__q_WFE__) *  sin__q_WR__));
    (*this)(2,1) = (((- sin__q_EFE__ *  cos__q_WFE__) *  sin__q_WR__) - ( cos__q_EFE__ *  sin__q_WFE__));
    (*this)(2,2) = (- sin__q_EFE__ *  cos__q_WR__);
    (*this)(2,3) = (((((((- 0.03 *  sin__q_EFE__) *  sin__q_WFE__) - ( 0.02075 *  sin__q_EFE__)) *  sin__q_WR__) + (( 0.03 *  cos__q_EFE__) *  cos__q_WFE__)) + ( 0.296 *  cos__q_EFE__)) +  0.304);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::Type_fr_Elbow_FE_X_fr_ee()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_WFE__;
    SCALAR cos__q_WR__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) =  cos__q_WFE__;
    (*this)(0,1) = - sin__q_WFE__;
    (*this)(0,3) = (( 0.03 *  cos__q_WFE__) +  0.296);
    (*this)(1,0) = ( sin__q_WFE__ *  sin__q_WR__);
    (*this)(1,1) = ( cos__q_WFE__ *  sin__q_WR__);
    (*this)(1,2) =  cos__q_WR__;
    (*this)(1,3) = ((( 0.03 *  sin__q_WFE__) +  0.02075) *  sin__q_WR__);
    (*this)(2,0) = (- sin__q_WFE__ *  cos__q_WR__);
    (*this)(2,1) = (- cos__q_WFE__ *  cos__q_WR__);
    (*this)(2,2) =  sin__q_WR__;
    (*this)(2,3) = (((- 0.03 *  sin__q_WFE__) -  0.02075) *  cos__q_WR__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::Type_fr_Wrist_R_X_fr_ee()
{
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,1) =  cos__q_WFE__;
    (*this)(0,3) = (( 0.03 *  sin__q_WFE__) +  0.02075);
    (*this)(2,0) =  cos__q_WFE__;
    (*this)(2,1) = - sin__q_WFE__;
    (*this)(2,3) = (( 0.03 *  cos__q_WFE__) +  0.184);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::Type_fr_Wrist_FE_X_fr_ee()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.03;
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
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_ee::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::Type_fr_Shoulder_AA_X_fr_HyABase()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
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
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,1) =  sin__q_SAA__;
    (*this)(1,0) = - sin__q_SAA__;
    (*this)(1,1) =  cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::Type_fr_Shoulder_FE_X_fr_HyABase()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(0,3) = (- 0.178 *  sin__q_SFE__);
    (*this)(1,0) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(1,3) = (- 0.178 *  cos__q_SFE__);
    (*this)(2,0) =  sin__q_SAA__;
    (*this)(2,1) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::Type_fr_Humerus_R_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(0,3) = ((- 0.178 *  sin__q_HR__) *  cos__q_SFE__);
    (*this)(1,0) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(1,3) = ((- 0.178 *  cos__q_HR__) *  cos__q_SFE__);
    (*this)(2,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(2,1) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = ((- 0.178 *  sin__q_SFE__) -  0.112);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::Type_fr_Elbow_FE_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(0,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(0,3) = ((((- 0.178 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) - ( 0.416 *  cos__q_EFE__));
    (*this)(1,0) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(1,3) = (((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__));
    (*this)(2,0) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(2,1) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (( 0.178 *  sin__q_HR__) *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::Type_fr_Wrist_R_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + ((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(0,3) = (((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  sin__q_WR__) - ((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(1,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(1,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(1,3) = (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + ((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  cos__q_WR__));
    (*this)(2,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(2,1) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = (((((- 0.178 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) - ( 0.416 *  cos__q_EFE__)) -  0.112);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::Type_fr_Wrist_FE_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(0,3) = ((((((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  sin__q_WFE__) *  sin__q_WR__) - (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) - ( 0.02075 *  sin__q_WFE__)) + ((((((- 0.178 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) - ( 0.416 *  cos__q_EFE__)) -  0.296) *  cos__q_WFE__));
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(1,3) = ((((((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  cos__q_WFE__) *  sin__q_WR__) - (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( 0.178 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  cos__q_EFE__)) +  0.296) *  sin__q_WFE__)) - ( 0.02075 *  cos__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + ((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::Type_fr_ee_X_fr_HyABase()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_HyABase::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR sin__q_EFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WFE__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    SCALAR cos__q_WFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  sin__q_WFE__) *  cos__q_WR__)) + ((((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__));
    (*this)(0,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) + (((( 1.0 *  cos__q_EFE__) *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  cos__q_WFE__));
    (*this)(0,3) = (((((((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  sin__q_WFE__) *  sin__q_WR__) - (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WFE__) *  cos__q_WR__)) - ( 0.02075 *  sin__q_WFE__)) + ((((((- 0.178 *  cos__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) - ( 0.416 *  cos__q_EFE__)) -  0.296) *  cos__q_WFE__)) -  0.03);
    (*this)(1,0) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,1) = (((((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WFE__) *  sin__q_WR__) + (((( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WFE__));
    (*this)(1,2) = (((((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WFE__) *  sin__q_WR__) + ((( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((- cos__q_EFE__ *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__)) *  sin__q_WFE__));
    (*this)(1,3) = ((((((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  cos__q_WFE__) *  sin__q_WR__) - (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WFE__) *  cos__q_WR__)) + (((((( 0.178 *  cos__q_EFE__) *  sin__q_SFE__) + ((( 0.178 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  cos__q_EFE__)) +  0.296) *  sin__q_WFE__)) - ( 0.02075 *  cos__q_WFE__));
    (*this)(2,0) = ((((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(2,1) = ((((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = (((( 0.178 *  sin__q_HR__) *  cos__q_SFE__) *  sin__q_WR__) + ((((( 0.178 *  sin__q_EFE__) *  sin__q_SFE__) - ((( 0.178 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) + ( 0.416 *  sin__q_EFE__)) *  cos__q_WR__));
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::Type_fr_HyABase_X_fr_SAA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::Type_fr_HyABase_X_fr_SFE()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.178;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_SFE::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR cos__q_SAA__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    
    (*this)(0,0) =  cos__q_SAA__;
    (*this)(0,2) =  sin__q_SAA__;
    (*this)(1,0) =  sin__q_SAA__;
    (*this)(1,2) = - cos__q_SAA__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::Type_fr_HyABase_X_fr_HR()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_HR::update(const JState& q) {
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = - sin__q_SAA__;
    (*this)(0,1) = (- cos__q_SAA__ *  sin__q_SFE__);
    (*this)(0,2) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,3) = (( 0.112 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) =  cos__q_SAA__;
    (*this)(1,1) = (- sin__q_SAA__ *  sin__q_SFE__);
    (*this)(1,2) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,3) = (( 0.112 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,1) =  cos__q_SFE__;
    (*this)(2,2) =  sin__q_SFE__;
    (*this)(2,3) = (( 0.112 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::Type_fr_HyABase_X_fr_EFE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_EFE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,0) = ( cos__q_SAA__ *  cos__q_SFE__);
    (*this)(0,1) = (( sin__q_HR__ *  sin__q_SAA__) - (( cos__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__));
    (*this)(0,2) = ((( sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,3) = (( 0.416 *  cos__q_SAA__) *  cos__q_SFE__);
    (*this)(1,0) = ( sin__q_SAA__ *  cos__q_SFE__);
    (*this)(1,1) = (((- cos__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( sin__q_HR__ *  cos__q_SAA__));
    (*this)(1,2) = ((( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__));
    (*this)(1,3) = (( 0.416 *  sin__q_SAA__) *  cos__q_SFE__);
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) = ( cos__q_HR__ *  cos__q_SFE__);
    (*this)(2,2) = (- sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,3) = (( 0.416 *  sin__q_SFE__) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::Type_fr_HyABase_X_fr_WR()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WR::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_EFE__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) = (((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__));
    (*this)(0,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,3) = ((((((- 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (( cos__q_HR__ *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__));
    (*this)(1,1) = (((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,2) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,3) = ((((((- 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.112 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = ( sin__q_HR__ *  cos__q_SFE__);
    (*this)(2,1) = ((( cos__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__));
    (*this)(2,2) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,3) = ((((( 0.112 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__) + ((( 0.112 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::Type_fr_HyABase_X_fr_WFE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_HyABase_X_fr_WFE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR sin__q_SFE__;
    SCALAR sin__q_HR__;
    SCALAR sin__q_SAA__;
    SCALAR sin__q_WR__;
    SCALAR cos__q_HR__;
    SCALAR cos__q_SAA__;
    SCALAR cos__q_EFE__;
    SCALAR cos__q_SFE__;
    SCALAR cos__q_WR__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    sin__q_HR__ = TRAIT::sin( q(HR));
    sin__q_SAA__ = TRAIT::sin( q(SAA));
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    cos__q_SAA__ = TRAIT::cos( q(SAA));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( sin__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(0,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + ((((- sin__q_HR__ *  cos__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,2) = (((((( 1.0 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) + ( cos__q_HR__ *  sin__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  cos__q_SAA__) *  cos__q_SFE__)) + (( cos__q_EFE__ *  sin__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__));
    (*this)(0,3) = (((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__)) *  sin__q_WR__) + (((((- 0.02075 *  sin__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__) - (( 0.02075 *  cos__q_HR__) *  sin__q_SAA__)) *  cos__q_WR__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  cos__q_SAA__) *  cos__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  sin__q_SAA__));
    (*this)(1,0) = (((((- sin__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) + (( cos__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( sin__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(1,1) = (((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 1.0 *  cos__q_HR__) *  cos__q_SAA__) - (( sin__q_HR__ *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__));
    (*this)(1,2) = (((((( 1.0 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ( cos__q_HR__ *  cos__q_SAA__)) *  sin__q_WR__) + ((((((- cos__q_EFE__ *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - (( sin__q_EFE__ *  sin__q_SAA__) *  cos__q_SFE__)) - (( cos__q_EFE__ *  sin__q_HR__) *  cos__q_SAA__)) *  cos__q_WR__));
    (*this)(1,3) = (((((((((((- 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__) - ((( 0.02075 *  sin__q_EFE__) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.02075 *  cos__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__)) *  sin__q_WR__) + (((( 0.02075 *  cos__q_HR__) *  cos__q_SAA__) - ((( 0.02075 *  sin__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) *  cos__q_WR__)) - (((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  sin__q_SAA__) *  sin__q_SFE__)) + (((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SAA__) *  cos__q_SFE__)) - ((( 0.296 *  sin__q_EFE__) *  sin__q_HR__) *  cos__q_SAA__));
    (*this)(2,0) = (( cos__q_EFE__ *  sin__q_SFE__) + (( sin__q_EFE__ *  cos__q_HR__) *  cos__q_SFE__));
    (*this)(2,1) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  sin__q_WR__) + (( sin__q_HR__ *  cos__q_SFE__) *  cos__q_WR__));
    (*this)(2,2) = (((((( 1.0 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - ( sin__q_EFE__ *  sin__q_SFE__)) *  cos__q_WR__) - (( sin__q_HR__ *  cos__q_SFE__) *  sin__q_WR__));
    (*this)(2,3) = ((((((((( 0.02075 *  cos__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__) - (( 0.02075 *  sin__q_EFE__) *  sin__q_SFE__)) *  sin__q_WR__) + ((( 0.02075 *  sin__q_HR__) *  cos__q_SFE__) *  cos__q_WR__)) + ((( 0.296 *  cos__q_EFE__) +  0.416) *  sin__q_SFE__)) + ((( 0.296 *  sin__q_EFE__) *  cos__q_HR__) *  cos__q_SFE__)) +  0.178);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::Type_fr_Shoulder_FE_X_fr_Shoulder_AA()
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
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Shoulder_AA::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,2) =  sin__q_SFE__;
    (*this)(0,3) = (- 0.178 *  sin__q_SFE__);
    (*this)(1,0) = - sin__q_SFE__;
    (*this)(1,2) =  cos__q_SFE__;
    (*this)(1,3) = (- 0.178 *  cos__q_SFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::Type_fr_Shoulder_AA_X_fr_Shoulder_FE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.178;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_AA_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_SFE__;
    SCALAR cos__q_SFE__;
    
    sin__q_SFE__ = TRAIT::sin( q(SFE));
    cos__q_SFE__ = TRAIT::cos( q(SFE));
    
    (*this)(0,0) =  cos__q_SFE__;
    (*this)(0,1) = - sin__q_SFE__;
    (*this)(2,0) =  sin__q_SFE__;
    (*this)(2,1) =  cos__q_SFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::Type_fr_Humerus_R_X_fr_Shoulder_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.112;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Shoulder_FE::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(0,1) =  sin__q_HR__;
    (*this)(0,2) = - cos__q_HR__;
    (*this)(1,1) =  cos__q_HR__;
    (*this)(1,2) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::Type_fr_Shoulder_FE_X_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.112;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Shoulder_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_HR__;
    SCALAR cos__q_HR__;
    
    sin__q_HR__ = TRAIT::sin( q(HR));
    cos__q_HR__ = TRAIT::cos( q(HR));
    
    (*this)(1,0) =  sin__q_HR__;
    (*this)(1,1) =  cos__q_HR__;
    (*this)(2,0) = - cos__q_HR__;
    (*this)(2,1) =  sin__q_HR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::Type_fr_Elbow_FE_X_fr_Humerus_R()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = - 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Humerus_R::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(0,1) =  sin__q_EFE__;
    (*this)(0,2) =  cos__q_EFE__;
    (*this)(0,3) = (- 0.304 *  cos__q_EFE__);
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(1,2) = - sin__q_EFE__;
    (*this)(1,3) = ( 0.304 *  sin__q_EFE__);
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::Type_fr_Humerus_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.304;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Humerus_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_EFE__;
    SCALAR cos__q_EFE__;
    
    sin__q_EFE__ = TRAIT::sin( q(EFE));
    cos__q_EFE__ = TRAIT::cos( q(EFE));
    
    (*this)(1,0) =  sin__q_EFE__;
    (*this)(1,1) =  cos__q_EFE__;
    (*this)(2,0) =  cos__q_EFE__;
    (*this)(2,1) = - sin__q_EFE__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::Type_fr_Wrist_R_X_fr_Elbow_FE()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.112;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_R_X_fr_Elbow_FE::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(0,1) =  sin__q_WR__;
    (*this)(0,2) = - cos__q_WR__;
    (*this)(1,1) =  cos__q_WR__;
    (*this)(1,2) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::Type_fr_Elbow_FE_X_fr_Wrist_R()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.112;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Elbow_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WR__;
    SCALAR cos__q_WR__;
    
    sin__q_WR__ = TRAIT::sin( q(WR));
    cos__q_WR__ = TRAIT::cos( q(WR));
    
    (*this)(1,0) =  sin__q_WR__;
    (*this)(1,1) =  cos__q_WR__;
    (*this)(2,0) = - cos__q_WR__;
    (*this)(2,1) =  sin__q_WR__;
    return *this;
}
template <typename TRAIT>
iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::Type_fr_Wrist_FE_X_fr_Wrist_R()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R& iit::ct_HyA::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Wrist_FE_X_fr_Wrist_R::update(const JState& q) {
    SCALAR sin__q_WFE__;
    SCALAR cos__q_WFE__;
    
    sin__q_WFE__ = TRAIT::sin( q(WFE));
    cos__q_WFE__ = TRAIT::cos( q(WFE));
    
    (*this)(0,0) =  sin__q_WFE__;
    (*this)(0,2) =  cos__q_WFE__;
    (*this)(0,3) = ((- 0.02075 *  sin__q_WFE__) - ( 0.184 *  cos__q_WFE__));
    (*this)(1,0) =  cos__q_WFE__;
    (*this)(1,2) = - sin__q_WFE__;
    (*this)(1,3) = (( 0.184 *  sin__q_WFE__) - ( 0.02075 *  cos__q_WFE__));
    return *this;
}

