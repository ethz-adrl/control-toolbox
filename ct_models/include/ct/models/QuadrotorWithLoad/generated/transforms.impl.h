
// Constructors
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_body_X_fr_link1(),
    fr_body_X_fr_link2(),
    fr_body_X_fr_ee(),
    fr_body_X_fr_com0(),
    fr_body_X_fr_com1(),
    fr_link1_X_fr_body(),
    fr_link2_X_fr_body(),
    fr_ee_X_fr_body(),
    fr_com0_X_fr_body(),
    fr_com1_X_fr_body(),
    fr_body_X_fr_jA(),
    fr_body_X_fr_jB(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_body_X_fr_link1(),
    fr_body_X_fr_link2(),
    fr_body_X_fr_ee(),
    fr_body_X_fr_com0(),
    fr_body_X_fr_com1(),
    fr_link1_X_fr_body(),
    fr_link2_X_fr_body(),
    fr_ee_X_fr_body(),
    fr_com0_X_fr_body(),
    fr_com1_X_fr_body(),
    fr_body_X_fr_jA(),
    fr_body_X_fr_jB(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_body_X_fr_link1(),
    fr_body_X_fr_link2(),
    fr_body_X_fr_ee(),
    fr_body_X_fr_com0(),
    fr_body_X_fr_com1(),
    fr_link1_X_fr_body(),
    fr_link2_X_fr_body(),
    fr_ee_X_fr_body(),
    fr_com0_X_fr_body(),
    fr_com1_X_fr_body(),
    fr_body_X_fr_jA(),
    fr_body_X_fr_jB(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link1::Type_fr_body_X_fr_link1()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link1& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) = - sin__q_jA__;
    (*this)(4,3) =  sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link2::Type_fr_body_X_fr_link2()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link2& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(3,0) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(3,1) = (( 0.1 *  sin__q_jA__) *  cos__q_jB__);
    (*this)(3,2) = (- 0.1 *  cos__q_jA__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,0) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(4,1) = ((- 0.1 *  cos__q_jA__) *  cos__q_jB__);
    (*this)(4,2) = (- 0.1 *  sin__q_jA__);
    (*this)(4,3) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) = - cos__q_jA__;
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_ee::Type_fr_body_X_fr_ee()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_ee& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(3,0) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(3,1) = ((( 0.1 *  sin__q_jA__) *  cos__q_jB__) + ( 0.7 *  sin__q_jA__));
    (*this)(3,2) = (((- 0.7 *  cos__q_jA__) *  cos__q_jB__) - ( 0.1 *  cos__q_jA__));
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,0) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(4,1) = (((- 0.1 *  cos__q_jA__) *  cos__q_jB__) - ( 0.7 *  cos__q_jA__));
    (*this)(4,2) = (((- 0.7 *  sin__q_jA__) *  cos__q_jB__) - ( 0.1 *  sin__q_jA__));
    (*this)(4,3) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) = - cos__q_jA__;
    (*this)(5,2) = (- 0.7 *  sin__q_jB__);
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com0::Type_fr_body_X_fr_com0()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com0& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com0::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com1::Type_fr_body_X_fr_com1()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com1& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_com1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,0) = ( 0.05 *  sin__q_jA__);
    (*this)(3,1) = ( 0.05 *  cos__q_jA__);
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) = - sin__q_jA__;
    (*this)(4,0) = (- 0.05 *  cos__q_jA__);
    (*this)(4,1) = ( 0.05 *  sin__q_jA__);
    (*this)(4,3) =  sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_body::Type_fr_link1_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_body& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) =  sin__q_jA__;
    (*this)(4,3) = - sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_body::Type_fr_link2_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_body& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    (*this)(3,0) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(3,1) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,0) = (( 0.1 *  sin__q_jA__) *  cos__q_jB__);
    (*this)(4,1) = ((- 0.1 *  cos__q_jA__) *  cos__q_jB__);
    (*this)(4,3) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) =  sin__q_jB__;
    (*this)(5,0) = (- 0.1 *  cos__q_jA__);
    (*this)(5,1) = (- 0.1 *  sin__q_jA__);
    (*this)(5,3) =  sin__q_jA__;
    (*this)(5,4) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_body::Type_fr_ee_X_fr_body()
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
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_body& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    (*this)(3,0) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(3,1) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,0) = ((( 0.1 *  sin__q_jA__) *  cos__q_jB__) + ( 0.7 *  sin__q_jA__));
    (*this)(4,1) = (((- 0.1 *  cos__q_jA__) *  cos__q_jB__) - ( 0.7 *  cos__q_jA__));
    (*this)(4,3) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) =  sin__q_jB__;
    (*this)(5,0) = (((- 0.7 *  cos__q_jA__) *  cos__q_jB__) - ( 0.1 *  cos__q_jA__));
    (*this)(5,1) = (((- 0.7 *  sin__q_jA__) *  cos__q_jB__) - ( 0.1 *  sin__q_jA__));
    (*this)(5,2) = (- 0.7 *  sin__q_jB__);
    (*this)(5,3) =  sin__q_jA__;
    (*this)(5,4) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com0_X_fr_body::Type_fr_com0_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com0_X_fr_body& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com0_X_fr_body::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com1_X_fr_body::Type_fr_com1_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com1_X_fr_body& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_com1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,0) = ( 0.05 *  sin__q_jA__);
    (*this)(3,1) = (- 0.05 *  cos__q_jA__);
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) =  sin__q_jA__;
    (*this)(4,0) = ( 0.05 *  cos__q_jA__);
    (*this)(4,1) = ( 0.05 *  sin__q_jA__);
    (*this)(4,3) = - sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jA::Type_fr_body_X_fr_jA()
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jA& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jB::Type_fr_body_X_fr_jB()
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
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jB& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_body_X_fr_jB::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,1) =  cos__q_jA__;
    (*this)(0,2) =  sin__q_jA__;
    (*this)(1,1) =  sin__q_jA__;
    (*this)(1,2) = - cos__q_jA__;
    (*this)(3,1) = ( 0.1 *  sin__q_jA__);
    (*this)(3,2) = (- 0.1 *  cos__q_jA__);
    (*this)(3,4) =  cos__q_jA__;
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,1) = (- 0.1 *  cos__q_jA__);
    (*this)(4,2) = (- 0.1 *  sin__q_jA__);
    (*this)(4,4) =  sin__q_jA__;
    (*this)(4,5) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
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
    (*this)(5,0) = - 0.1;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,2) = - cos__q_jB__;
    (*this)(1,0) =  cos__q_jB__;
    (*this)(1,2) =  sin__q_jB__;
    (*this)(3,1) = (- 0.1 *  sin__q_jB__);
    (*this)(3,3) =  sin__q_jB__;
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,1) = (- 0.1 *  cos__q_jB__);
    (*this)(4,3) =  cos__q_jB__;
    (*this)(4,5) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
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
    (*this)(3,2) = - 0.1;
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
const typename iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::ct_quadrotor::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,1) =  cos__q_jB__;
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(3,3) =  sin__q_jB__;
    (*this)(3,4) =  cos__q_jB__;
    (*this)(4,0) = (- 0.1 *  sin__q_jB__);
    (*this)(4,1) = (- 0.1 *  cos__q_jB__);
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}

template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link1::Type_fr_body_X_fr_link1()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link1& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) = - sin__q_jA__;
    (*this)(4,3) =  sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link2::Type_fr_body_X_fr_link2()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link2& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(0,3) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(0,4) = (( 0.1 *  sin__q_jA__) *  cos__q_jB__);
    (*this)(0,5) = (- 0.1 *  cos__q_jA__);
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(1,3) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(1,4) = ((- 0.1 *  cos__q_jA__) *  cos__q_jB__);
    (*this)(1,5) = (- 0.1 *  sin__q_jA__);
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,3) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) = - cos__q_jA__;
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_ee::Type_fr_body_X_fr_ee()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_ee& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(0,3) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(0,4) = ((( 0.1 *  sin__q_jA__) *  cos__q_jB__) + ( 0.7 *  sin__q_jA__));
    (*this)(0,5) = (((- 0.7 *  cos__q_jA__) *  cos__q_jB__) - ( 0.1 *  cos__q_jA__));
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(1,3) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(1,4) = (((- 0.1 *  cos__q_jA__) *  cos__q_jB__) - ( 0.7 *  cos__q_jA__));
    (*this)(1,5) = (((- 0.7 *  sin__q_jA__) *  cos__q_jB__) - ( 0.1 *  sin__q_jA__));
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(2,5) = (- 0.7 *  sin__q_jB__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,3) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) = - cos__q_jA__;
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com0::Type_fr_body_X_fr_com0()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com0& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com0::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com1::Type_fr_body_X_fr_com1()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com1& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_com1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(0,3) = ( 0.05 *  sin__q_jA__);
    (*this)(0,4) = ( 0.05 *  cos__q_jA__);
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(1,3) = (- 0.05 *  cos__q_jA__);
    (*this)(1,4) = ( 0.05 *  sin__q_jA__);
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) = - sin__q_jA__;
    (*this)(4,3) =  sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_body::Type_fr_link1_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_body& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) =  sin__q_jA__;
    (*this)(4,3) = - sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_body::Type_fr_link2_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_body& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,3) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(0,4) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,3) = (( 0.1 *  sin__q_jA__) *  cos__q_jB__);
    (*this)(1,4) = ((- 0.1 *  cos__q_jA__) *  cos__q_jB__);
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    (*this)(2,3) = (- 0.1 *  cos__q_jA__);
    (*this)(2,4) = (- 0.1 *  sin__q_jA__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,3) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) =  sin__q_jB__;
    (*this)(5,3) =  sin__q_jA__;
    (*this)(5,4) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_body::Type_fr_ee_X_fr_body()
{
    (*this)(0,5) = 0;
    (*this)(1,5) = 0;
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_body& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,3) = (( 0.1 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(0,4) = ((- 0.1 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,3) = ((( 0.1 *  sin__q_jA__) *  cos__q_jB__) + ( 0.7 *  sin__q_jA__));
    (*this)(1,4) = (((- 0.1 *  cos__q_jA__) *  cos__q_jB__) - ( 0.7 *  cos__q_jA__));
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    (*this)(2,3) = (((- 0.7 *  cos__q_jA__) *  cos__q_jB__) - ( 0.1 *  cos__q_jA__));
    (*this)(2,4) = (((- 0.7 *  sin__q_jA__) *  cos__q_jB__) - ( 0.1 *  sin__q_jA__));
    (*this)(2,5) = (- 0.7 *  sin__q_jB__);
    (*this)(3,3) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(3,4) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,3) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(4,4) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(4,5) =  sin__q_jB__;
    (*this)(5,3) =  sin__q_jA__;
    (*this)(5,4) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com0_X_fr_body::Type_fr_com0_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com0_X_fr_body& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com0_X_fr_body::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com1_X_fr_body::Type_fr_com1_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com1_X_fr_body& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_com1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(0,3) = ( 0.05 *  sin__q_jA__);
    (*this)(0,4) = (- 0.05 *  cos__q_jA__);
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    (*this)(1,3) = ( 0.05 *  cos__q_jA__);
    (*this)(1,4) = ( 0.05 *  sin__q_jA__);
    (*this)(3,3) =  cos__q_jA__;
    (*this)(3,4) =  sin__q_jA__;
    (*this)(4,3) = - sin__q_jA__;
    (*this)(4,4) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jA::Type_fr_body_X_fr_jA()
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jA& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jB::Type_fr_body_X_fr_jB()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = - 1.0;
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
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jB& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_body_X_fr_jB::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,1) =  cos__q_jA__;
    (*this)(0,2) =  sin__q_jA__;
    (*this)(0,4) = ( 0.1 *  sin__q_jA__);
    (*this)(0,5) = (- 0.1 *  cos__q_jA__);
    (*this)(1,1) =  sin__q_jA__;
    (*this)(1,2) = - cos__q_jA__;
    (*this)(1,4) = (- 0.1 *  cos__q_jA__);
    (*this)(1,5) = (- 0.1 *  sin__q_jA__);
    (*this)(3,4) =  cos__q_jA__;
    (*this)(3,5) =  sin__q_jA__;
    (*this)(4,4) =  sin__q_jA__;
    (*this)(4,5) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
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
    (*this)(2,3) = - 0.1;
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,4) = (- 0.1 *  sin__q_jB__);
    (*this)(1,0) =  cos__q_jB__;
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,4) = (- 0.1 *  cos__q_jB__);
    (*this)(3,3) =  sin__q_jB__;
    (*this)(3,5) = - cos__q_jB__;
    (*this)(4,3) =  cos__q_jB__;
    (*this)(4,5) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.1;
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
const typename iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::ct_quadrotor::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,1) =  cos__q_jB__;
    (*this)(1,3) = (- 0.1 *  sin__q_jB__);
    (*this)(1,4) = (- 0.1 *  cos__q_jB__);
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(3,3) =  sin__q_jB__;
    (*this)(3,4) =  cos__q_jB__;
    (*this)(5,3) = - cos__q_jB__;
    (*this)(5,4) =  sin__q_jB__;
    return *this;
}

template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link1::Type_fr_body_X_fr_link1()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link1& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link2::Type_fr_body_X_fr_link2()
{
    (*this)(0,3) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.1;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link2& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_ee::Type_fr_body_X_fr_ee()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_ee& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_ee::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(0,2) =  sin__q_jA__;
    (*this)(0,3) = (( 0.7 *  cos__q_jA__) *  sin__q_jB__);
    (*this)(1,0) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) = - cos__q_jA__;
    (*this)(1,3) = (( 0.7 *  sin__q_jA__) *  sin__q_jB__);
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    (*this)(2,3) = ((- 0.7 *  cos__q_jB__) -  0.1);
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com0::Type_fr_body_X_fr_com0()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com0& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com0::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com1::Type_fr_body_X_fr_com1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = - 0.05;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com1& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_com1::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) = - sin__q_jA__;
    (*this)(1,0) =  sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_body::Type_fr_link1_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_body& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_body::Type_fr_link2_X_fr_body()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_body& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,3) = (- 0.1 *  cos__q_jB__);
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,3) = ( 0.1 *  sin__q_jB__);
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_body::Type_fr_ee_X_fr_body()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_body& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( cos__q_jA__ *  sin__q_jB__);
    (*this)(0,1) = ( sin__q_jA__ *  sin__q_jB__);
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,3) = ((- 0.1 *  cos__q_jB__) -  0.7);
    (*this)(1,0) = ( cos__q_jA__ *  cos__q_jB__);
    (*this)(1,1) = ( sin__q_jA__ *  cos__q_jB__);
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,3) = ( 0.1 *  sin__q_jB__);
    (*this)(2,0) =  sin__q_jA__;
    (*this)(2,1) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com0_X_fr_body::Type_fr_com0_X_fr_body()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com0_X_fr_body& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com0_X_fr_body::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com1_X_fr_body::Type_fr_com1_X_fr_body()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.05;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com1_X_fr_body& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_com1_X_fr_body::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  cos__q_jA__;
    (*this)(0,1) =  sin__q_jA__;
    (*this)(1,0) = - sin__q_jA__;
    (*this)(1,1) =  cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jA::Type_fr_body_X_fr_jA()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jA& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jB::Type_fr_body_X_fr_jB()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.1;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jB& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_body_X_fr_jB::update(const JState& q) {
    SCALAR sin__q_jA__;
    SCALAR cos__q_jA__;
    
    sin__q_jA__ = TRAIT::sin( q(JA));
    cos__q_jA__ = TRAIT::cos( q(JA));
    
    (*this)(0,1) =  cos__q_jA__;
    (*this)(0,2) =  sin__q_jA__;
    (*this)(1,1) =  sin__q_jA__;
    (*this)(1,2) = - cos__q_jA__;
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
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
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,2) = - cos__q_jB__;
    (*this)(0,3) = (- 0.1 *  cos__q_jB__);
    (*this)(1,0) =  cos__q_jB__;
    (*this)(1,2) =  sin__q_jB__;
    (*this)(1,3) = ( 0.1 *  sin__q_jB__);
    return *this;
}
template <typename TRAIT>
iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.1;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::ct_quadrotor::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    SCALAR sin__q_jB__;
    SCALAR cos__q_jB__;
    
    sin__q_jB__ = TRAIT::sin( q(JB));
    cos__q_jB__ = TRAIT::cos( q(JB));
    
    (*this)(0,0) =  sin__q_jB__;
    (*this)(0,1) =  cos__q_jB__;
    (*this)(2,0) = - cos__q_jB__;
    (*this)(2,1) =  sin__q_jB__;
    return *this;
}

