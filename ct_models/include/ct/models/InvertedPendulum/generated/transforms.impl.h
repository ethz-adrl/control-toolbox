
// Constructors
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_InvertedPendulumBase_X_fr_Link1(),
    fr_InvertedPendulumBase_X_fr_ee(),
    fr_Link1_X_fr_InvertedPendulumBase(),
    fr_ee_X_fr_InvertedPendulumBase(),
    fr_InvertedPendulumBase_X_fr_Joint1()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_InvertedPendulumBase_X_fr_Link1(),
    fr_InvertedPendulumBase_X_fr_ee(),
    fr_Link1_X_fr_InvertedPendulumBase(),
    fr_ee_X_fr_InvertedPendulumBase(),
    fr_InvertedPendulumBase_X_fr_Joint1()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_InvertedPendulumBase_X_fr_Link1(),
    fr_InvertedPendulumBase_X_fr_ee(),
    fr_Link1_X_fr_InvertedPendulumBase(),
    fr_ee_X_fr_InvertedPendulumBase(),
    fr_InvertedPendulumBase_X_fr_Joint1()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::Type_fr_InvertedPendulumBase_X_fr_Link1()
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
const typename iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1& iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::Type_fr_InvertedPendulumBase_X_fr_ee()
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0.435;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee& iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(4,2) = ( 0.435 *  c_q_Joint1_);
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,2) = ( 0.435 *  s_q_Joint1_);
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::Type_fr_Link1_X_fr_InvertedPendulumBase()
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
const typename iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    (*this)(3,4) = - s_q_Joint1_;
    (*this)(3,5) =  c_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(4,5) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::Type_fr_ee_X_fr_InvertedPendulumBase()
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0.435;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    (*this)(3,4) = - s_q_Joint1_;
    (*this)(3,5) =  c_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(4,5) = - s_q_Joint1_;
    (*this)(5,1) = ( 0.435 *  c_q_Joint1_);
    (*this)(5,2) = ( 0.435 *  s_q_Joint1_);
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::Type_fr_InvertedPendulumBase_X_fr_Joint1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1;
    (*this)(1,2) = 0;
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1& iit::ct_InvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}

template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::Type_fr_InvertedPendulumBase_X_fr_Link1()
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
const typename iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1& iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::Type_fr_InvertedPendulumBase_X_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0.435;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
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
const typename iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee& iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,5) = ( 0.435 *  c_q_Joint1_);
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(2,5) = ( 0.435 *  s_q_Joint1_);
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::Type_fr_Link1_X_fr_InvertedPendulumBase()
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
const typename iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    (*this)(3,4) = - s_q_Joint1_;
    (*this)(3,5) =  c_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(4,5) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::Type_fr_ee_X_fr_InvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0.435;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
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
const typename iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    (*this)(2,4) = ( 0.435 *  c_q_Joint1_);
    (*this)(2,5) = ( 0.435 *  s_q_Joint1_);
    (*this)(3,4) = - s_q_Joint1_;
    (*this)(3,5) =  c_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(4,5) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::Type_fr_InvertedPendulumBase_X_fr_Joint1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1;
    (*this)(1,2) = 0;
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1& iit::ct_InvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}

template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::Type_fr_InvertedPendulumBase_X_fr_Link1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
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
const typename iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1& iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::Type_fr_InvertedPendulumBase_X_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee& iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,3) = (- 0.435 *  s_q_Joint1_);
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(2,3) = ( 0.435 *  c_q_Joint1_);
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::Type_fr_Link1_X_fr_InvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::Type_fr_ee_X_fr_InvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = - 0.435;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase& iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_InvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,1) = - s_q_Joint1_;
    (*this)(0,2) =  c_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,2) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::Type_fr_InvertedPendulumBase_X_fr_Joint1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1& iit::ct_InvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_InvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}

