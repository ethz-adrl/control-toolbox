
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_DoubleInvertedPendulumBase_J_fr_Link2(), 
    fr_DoubleInvertedPendulumBase_J_fr_Link1(), 
    fr_DoubleInvertedPendulumBase_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link2::Type_fr_DoubleInvertedPendulumBase_J_fr_Link2()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 1.0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link2& iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link2::update(const JState& jState) {
    Scalar s_q_Joint1_;
    Scalar c_q_Joint1_;
    
    s_q_Joint1_ = TRAIT::sin( jState(JOINT1));
    c_q_Joint1_ = TRAIT::cos( jState(JOINT1));
    
    (*this)(4,0) = - c_q_Joint1_;
    (*this)(5,0) = - s_q_Joint1_;
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link1::Type_fr_DoubleInvertedPendulumBase_J_fr_Link1()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link1& iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_Link1::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_ee::Type_fr_DoubleInvertedPendulumBase_J_fr_ee()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 1.0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
}

template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_ee& iit::ct_DoubleInvertedPendulum::tpl::Jacobians<TRAIT>::Type_fr_DoubleInvertedPendulumBase_J_fr_ee::update(const JState& jState) {
    Scalar s_q_Joint1_;
    Scalar s_q_Joint2_;
    Scalar c_q_Joint1_;
    Scalar c_q_Joint2_;
    
    s_q_Joint1_ = TRAIT::sin( jState(JOINT1));
    s_q_Joint2_ = TRAIT::sin( jState(JOINT2));
    c_q_Joint1_ = TRAIT::cos( jState(JOINT1));
    c_q_Joint2_ = TRAIT::cos( jState(JOINT2));
    
    (*this)(4,0) = ((( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_)) -  c_q_Joint1_);
    (*this)(4,1) = (( s_q_Joint1_ *  s_q_Joint2_) - ( c_q_Joint1_ *  c_q_Joint2_));
    (*this)(5,0) = (((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_)) -  s_q_Joint1_);
    (*this)(5,1) = ((- c_q_Joint1_ *  s_q_Joint2_) - ( s_q_Joint1_ *  c_q_Joint2_));
    return *this;
}
