
// Constructors
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_DoubleInvertedPendulumBase_X_fr_Link1(),
    fr_DoubleInvertedPendulumBase_X_fr_Link2(),
    fr_DoubleInvertedPendulumBase_X_fr_ee(),
    fr_Link2_X_fr_DoubleInvertedPendulumBase(),
    fr_Link1_X_fr_DoubleInvertedPendulumBase(),
    fr_ee_X_fr_DoubleInvertedPendulumBase(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint1(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint2(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_DoubleInvertedPendulumBase_X_fr_Link1(),
    fr_DoubleInvertedPendulumBase_X_fr_Link2(),
    fr_DoubleInvertedPendulumBase_X_fr_ee(),
    fr_Link2_X_fr_DoubleInvertedPendulumBase(),
    fr_Link1_X_fr_DoubleInvertedPendulumBase(),
    fr_ee_X_fr_DoubleInvertedPendulumBase(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint1(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint2(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_DoubleInvertedPendulumBase_X_fr_Link1(),
    fr_DoubleInvertedPendulumBase_X_fr_Link2(),
    fr_DoubleInvertedPendulumBase_X_fr_ee(),
    fr_Link2_X_fr_DoubleInvertedPendulumBase(),
    fr_Link1_X_fr_DoubleInvertedPendulumBase(),
    fr_ee_X_fr_DoubleInvertedPendulumBase(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint1(),
    fr_DoubleInvertedPendulumBase_X_fr_Joint2(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2()
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
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,0) =  s_q_Joint2_;
    (*this)(3,1) =  c_q_Joint2_;
    (*this)(4,2) =  c_q_Joint1_;
    (*this)(4,3) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,2) =  s_q_Joint1_;
    (*this)(5,3) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(5,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::Type_fr_DoubleInvertedPendulumBase_X_fr_ee()
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
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,0) =  s_q_Joint2_;
    (*this)(3,1) = ( c_q_Joint2_ +  1.0);
    (*this)(4,2) = (((- s_q_Joint1_ *  s_q_Joint2_) + ( c_q_Joint1_ *  c_q_Joint2_)) +  c_q_Joint1_);
    (*this)(4,3) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,2) = ((( c_q_Joint1_ *  s_q_Joint2_) + ( s_q_Joint1_ *  c_q_Joint2_)) +  s_q_Joint1_);
    (*this)(5,3) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(5,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase()
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,0) =  s_q_Joint2_;
    (*this)(3,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,5) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(4,0) =  c_q_Joint2_;
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,5) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,1) =  c_q_Joint1_;
    (*this)(5,2) =  s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::Type_fr_ee_X_fr_DoubleInvertedPendulumBase()
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
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,0) =  s_q_Joint2_;
    (*this)(3,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,5) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(4,0) = ( c_q_Joint2_ +  1.0);
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,5) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,1) = (((- s_q_Joint1_ *  s_q_Joint2_) + ( c_q_Joint1_ *  c_q_Joint2_)) +  c_q_Joint1_);
    (*this)(5,2) = ((( c_q_Joint1_ *  s_q_Joint2_) + ( s_q_Joint1_ *  c_q_Joint2_)) +  s_q_Joint1_);
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2()
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
    (*this)(3,1) = 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(4,2) =  c_q_Joint1_;
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,2) =  s_q_Joint1_;
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
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
    (*this)(5,1) = - 1.0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) =  s_q_Joint2_;
    (*this)(1,0) = - s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    (*this)(3,2) =  s_q_Joint2_;
    (*this)(3,3) =  c_q_Joint2_;
    (*this)(3,4) =  s_q_Joint2_;
    (*this)(4,2) =  c_q_Joint2_;
    (*this)(4,3) = - s_q_Joint2_;
    (*this)(4,4) =  c_q_Joint2_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
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
    (*this)(4,2) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::MotionTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) = - s_q_Joint2_;
    (*this)(1,0) =  s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    (*this)(3,3) =  c_q_Joint2_;
    (*this)(3,4) = - s_q_Joint2_;
    (*this)(4,3) =  s_q_Joint2_;
    (*this)(4,4) =  c_q_Joint2_;
    (*this)(5,0) =  s_q_Joint2_;
    (*this)(5,1) =  c_q_Joint2_;
    return *this;
}

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint2_;
    Scalar c_q_Joint1_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,3) =  s_q_Joint2_;
    (*this)(0,4) =  c_q_Joint2_;
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,5) =  c_q_Joint1_;
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,5) =  s_q_Joint1_;
    (*this)(4,3) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,3) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(5,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::Type_fr_DoubleInvertedPendulumBase_X_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint2_;
    Scalar c_q_Joint1_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(0,3) =  s_q_Joint2_;
    (*this)(0,4) = ( c_q_Joint2_ +  1.0);
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,5) = (((- s_q_Joint1_ *  s_q_Joint2_) + ( c_q_Joint1_ *  c_q_Joint2_)) +  c_q_Joint1_);
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,5) = ((( c_q_Joint1_ *  s_q_Joint2_) + ( s_q_Joint1_ *  c_q_Joint2_)) +  s_q_Joint1_);
    (*this)(4,3) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,3) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(5,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
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
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(0,3) =  s_q_Joint2_;
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) =  c_q_Joint2_;
    (*this)(2,4) =  c_q_Joint1_;
    (*this)(2,5) =  s_q_Joint1_;
    (*this)(3,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,5) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,5) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::Type_fr_ee_X_fr_DoubleInvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
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
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(0,3) =  s_q_Joint2_;
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) = ( c_q_Joint2_ +  1.0);
    (*this)(2,4) = (((- s_q_Joint1_ *  s_q_Joint2_) + ( c_q_Joint1_ *  c_q_Joint2_)) +  c_q_Joint1_);
    (*this)(2,5) = ((( c_q_Joint1_ *  s_q_Joint2_) + ( s_q_Joint1_ *  c_q_Joint2_)) +  s_q_Joint1_);
    (*this)(3,4) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(3,5) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(4,4) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(4,5) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,5) =  c_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(2,5) =  s_q_Joint1_;
    (*this)(4,3) = - s_q_Joint1_;
    (*this)(4,4) = - c_q_Joint1_;
    (*this)(5,3) =  c_q_Joint1_;
    (*this)(5,4) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
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
    (*this)(2,4) = - 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) =  s_q_Joint2_;
    (*this)(0,5) =  s_q_Joint2_;
    (*this)(1,0) = - s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    (*this)(1,5) =  c_q_Joint2_;
    (*this)(3,3) =  c_q_Joint2_;
    (*this)(3,4) =  s_q_Joint2_;
    (*this)(4,3) = - s_q_Joint2_;
    (*this)(4,4) =  c_q_Joint2_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) = - s_q_Joint2_;
    (*this)(1,0) =  s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    (*this)(2,3) =  s_q_Joint2_;
    (*this)(2,4) =  c_q_Joint2_;
    (*this)(3,3) =  c_q_Joint2_;
    (*this)(3,4) = - s_q_Joint2_;
    (*this)(4,3) =  s_q_Joint2_;
    (*this)(4,4) =  c_q_Joint2_;
    return *this;
}

template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link1::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) = - s_q_Joint1_;
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,3) =  c_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::Type_fr_DoubleInvertedPendulumBase_X_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_ee::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(1,0) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) = (((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_)) -  s_q_Joint1_);
    (*this)(2,0) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(2,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(2,3) = (((- s_q_Joint1_ *  s_q_Joint2_) + ( c_q_Joint1_ *  c_q_Joint2_)) +  c_q_Joint1_);
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(0,3) = - c_q_Joint2_;
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) =  s_q_Joint2_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
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
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::Type_fr_ee_X_fr_DoubleInvertedPendulumBase()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_DoubleInvertedPendulumBase::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(0,2) = (( c_q_Joint1_ *  c_q_Joint2_) - ( s_q_Joint1_ *  s_q_Joint2_));
    (*this)(0,3) = (- c_q_Joint2_ -  1.0);
    (*this)(1,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,2) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    (*this)(1,3) =  s_q_Joint2_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint1::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_DoubleInvertedPendulumBase_X_fr_Joint2::update(const JState& q) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( q(JOINT1));
    c_q_Joint1_ = TRAIT::cos( q(JOINT1));
    
    (*this)(1,0) = - s_q_Joint1_;
    (*this)(1,1) = - c_q_Joint1_;
    (*this)(1,3) = - s_q_Joint1_;
    (*this)(2,0) =  c_q_Joint1_;
    (*this)(2,1) = - s_q_Joint1_;
    (*this)(2,3) =  c_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link2_X_fr_Link1::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) =  s_q_Joint2_;
    (*this)(0,3) = - c_q_Joint2_;
    (*this)(1,0) = - s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    (*this)(1,3) =  s_q_Joint2_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 1.0;
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
const typename iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::HomogeneousTransforms<TRAIT>::Type_fr_Link1_X_fr_Link2::update(const JState& q) {
    Scalar s_q_Joint2_;
    Scalar c_q_Joint2_;
    
    s_q_Joint2_ = TRAIT::sin( q(JOINT2));
    c_q_Joint2_ = TRAIT::cos( q(JOINT2));
    
    (*this)(0,0) =  c_q_Joint2_;
    (*this)(0,1) = - s_q_Joint2_;
    (*this)(1,0) =  s_q_Joint2_;
    (*this)(1,1) =  c_q_Joint2_;
    return *this;
}

