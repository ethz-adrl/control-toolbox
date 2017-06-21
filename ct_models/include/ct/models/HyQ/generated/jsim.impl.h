

//Implementation of default constructor
template <typename TRAIT>
iit::HyQ::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    LF_lowerleg_Ic(linkInertias.getTensor_LF_lowerleg()),
    RF_lowerleg_Ic(linkInertias.getTensor_RF_lowerleg()),
    LH_lowerleg_Ic(linkInertias.getTensor_LH_lowerleg()),
    RH_lowerleg_Ic(linkInertias.getTensor_RH_lowerleg())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::HyQ::dyn::tpl::JSIM<TRAIT>& iit::HyQ::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg(state);
    frcTransf -> fr_RH_hipassembly_X_fr_RH_upperleg(state);
    frcTransf -> fr_trunk_X_fr_RH_hipassembly(state);
    frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg(state);
    frcTransf -> fr_LH_hipassembly_X_fr_LH_upperleg(state);
    frcTransf -> fr_trunk_X_fr_LH_hipassembly(state);
    frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg(state);
    frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg(state);
    frcTransf -> fr_trunk_X_fr_RF_hipassembly(state);
    frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg(state);
    frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg(state);
    frcTransf -> fr_trunk_X_fr_LF_hipassembly(state);

    // Initializes the composite inertia tensors
    trunk_Ic = linkInertias.getTensor_trunk();
    LF_hipassembly_Ic = linkInertias.getTensor_LF_hipassembly();
    LF_upperleg_Ic = linkInertias.getTensor_LF_upperleg();
    RF_hipassembly_Ic = linkInertias.getTensor_RF_hipassembly();
    RF_upperleg_Ic = linkInertias.getTensor_RF_upperleg();
    LH_hipassembly_Ic = linkInertias.getTensor_LH_hipassembly();
    LH_upperleg_Ic = linkInertias.getTensor_LH_upperleg();
    RH_hipassembly_Ic = linkInertias.getTensor_RH_hipassembly();
    RH_upperleg_Ic = linkInertias.getTensor_RH_upperleg();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link RH_lowerleg:
    iit::rbd::transformInertia<Scalar>(RH_lowerleg_Ic, frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg, Ic_spare);
    RH_upperleg_Ic += Ic_spare;

    Fcol(RH_KFE) = RH_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(RH_KFE+6, RH_KFE+6) = Fcol(RH_KFE)(iit::rbd::AZ);

    Fcol(RH_KFE) = frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg * Fcol(RH_KFE);
    DATA(RH_KFE+6, RH_HFE+6) = F(iit::rbd::AZ,RH_KFE);
    DATA(RH_HFE+6, RH_KFE+6) = DATA(RH_KFE+6, RH_HFE+6);
    Fcol(RH_KFE) = frcTransf -> fr_RH_hipassembly_X_fr_RH_upperleg * Fcol(RH_KFE);
    DATA(RH_KFE+6, RH_HAA+6) = F(iit::rbd::AZ,RH_KFE);
    DATA(RH_HAA+6, RH_KFE+6) = DATA(RH_KFE+6, RH_HAA+6);
    Fcol(RH_KFE) = frcTransf -> fr_trunk_X_fr_RH_hipassembly * Fcol(RH_KFE);

    // Link RH_upperleg:
    iit::rbd::transformInertia<Scalar>(RH_upperleg_Ic, frcTransf -> fr_RH_hipassembly_X_fr_RH_upperleg, Ic_spare);
    RH_hipassembly_Ic += Ic_spare;

    Fcol(RH_HFE) = RH_upperleg_Ic.col(iit::rbd::AZ);
    DATA(RH_HFE+6, RH_HFE+6) = Fcol(RH_HFE)(iit::rbd::AZ);

    Fcol(RH_HFE) = frcTransf -> fr_RH_hipassembly_X_fr_RH_upperleg * Fcol(RH_HFE);
    DATA(RH_HFE+6, RH_HAA+6) = F(iit::rbd::AZ,RH_HFE);
    DATA(RH_HAA+6, RH_HFE+6) = DATA(RH_HFE+6, RH_HAA+6);
    Fcol(RH_HFE) = frcTransf -> fr_trunk_X_fr_RH_hipassembly * Fcol(RH_HFE);

    // Link RH_hipassembly:
    iit::rbd::transformInertia<Scalar>(RH_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_RH_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(RH_HAA) = RH_hipassembly_Ic.col(iit::rbd::AZ);
    DATA(RH_HAA+6, RH_HAA+6) = Fcol(RH_HAA)(iit::rbd::AZ);

    Fcol(RH_HAA) = frcTransf -> fr_trunk_X_fr_RH_hipassembly * Fcol(RH_HAA);

    // Link LH_lowerleg:
    iit::rbd::transformInertia<Scalar>(LH_lowerleg_Ic, frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg, Ic_spare);
    LH_upperleg_Ic += Ic_spare;

    Fcol(LH_KFE) = LH_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(LH_KFE+6, LH_KFE+6) = Fcol(LH_KFE)(iit::rbd::AZ);

    Fcol(LH_KFE) = frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg * Fcol(LH_KFE);
    DATA(LH_KFE+6, LH_HFE+6) = F(iit::rbd::AZ,LH_KFE);
    DATA(LH_HFE+6, LH_KFE+6) = DATA(LH_KFE+6, LH_HFE+6);
    Fcol(LH_KFE) = frcTransf -> fr_LH_hipassembly_X_fr_LH_upperleg * Fcol(LH_KFE);
    DATA(LH_KFE+6, LH_HAA+6) = F(iit::rbd::AZ,LH_KFE);
    DATA(LH_HAA+6, LH_KFE+6) = DATA(LH_KFE+6, LH_HAA+6);
    Fcol(LH_KFE) = frcTransf -> fr_trunk_X_fr_LH_hipassembly * Fcol(LH_KFE);

    // Link LH_upperleg:
    iit::rbd::transformInertia<Scalar>(LH_upperleg_Ic, frcTransf -> fr_LH_hipassembly_X_fr_LH_upperleg, Ic_spare);
    LH_hipassembly_Ic += Ic_spare;

    Fcol(LH_HFE) = LH_upperleg_Ic.col(iit::rbd::AZ);
    DATA(LH_HFE+6, LH_HFE+6) = Fcol(LH_HFE)(iit::rbd::AZ);

    Fcol(LH_HFE) = frcTransf -> fr_LH_hipassembly_X_fr_LH_upperleg * Fcol(LH_HFE);
    DATA(LH_HFE+6, LH_HAA+6) = F(iit::rbd::AZ,LH_HFE);
    DATA(LH_HAA+6, LH_HFE+6) = DATA(LH_HFE+6, LH_HAA+6);
    Fcol(LH_HFE) = frcTransf -> fr_trunk_X_fr_LH_hipassembly * Fcol(LH_HFE);

    // Link LH_hipassembly:
    iit::rbd::transformInertia<Scalar>(LH_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_LH_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(LH_HAA) = LH_hipassembly_Ic.col(iit::rbd::AZ);
    DATA(LH_HAA+6, LH_HAA+6) = Fcol(LH_HAA)(iit::rbd::AZ);

    Fcol(LH_HAA) = frcTransf -> fr_trunk_X_fr_LH_hipassembly * Fcol(LH_HAA);

    // Link RF_lowerleg:
    iit::rbd::transformInertia<Scalar>(RF_lowerleg_Ic, frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg, Ic_spare);
    RF_upperleg_Ic += Ic_spare;

    Fcol(RF_KFE) = RF_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(RF_KFE+6, RF_KFE+6) = Fcol(RF_KFE)(iit::rbd::AZ);

    Fcol(RF_KFE) = frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg * Fcol(RF_KFE);
    DATA(RF_KFE+6, RF_HFE+6) = F(iit::rbd::AZ,RF_KFE);
    DATA(RF_HFE+6, RF_KFE+6) = DATA(RF_KFE+6, RF_HFE+6);
    Fcol(RF_KFE) = frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg * Fcol(RF_KFE);
    DATA(RF_KFE+6, RF_HAA+6) = F(iit::rbd::AZ,RF_KFE);
    DATA(RF_HAA+6, RF_KFE+6) = DATA(RF_KFE+6, RF_HAA+6);
    Fcol(RF_KFE) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_KFE);

    // Link RF_upperleg:
    iit::rbd::transformInertia<Scalar>(RF_upperleg_Ic, frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg, Ic_spare);
    RF_hipassembly_Ic += Ic_spare;

    Fcol(RF_HFE) = RF_upperleg_Ic.col(iit::rbd::AZ);
    DATA(RF_HFE+6, RF_HFE+6) = Fcol(RF_HFE)(iit::rbd::AZ);

    Fcol(RF_HFE) = frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg * Fcol(RF_HFE);
    DATA(RF_HFE+6, RF_HAA+6) = F(iit::rbd::AZ,RF_HFE);
    DATA(RF_HAA+6, RF_HFE+6) = DATA(RF_HFE+6, RF_HAA+6);
    Fcol(RF_HFE) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_HFE);

    // Link RF_hipassembly:
    iit::rbd::transformInertia<Scalar>(RF_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_RF_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(RF_HAA) = RF_hipassembly_Ic.col(iit::rbd::AZ);
    DATA(RF_HAA+6, RF_HAA+6) = Fcol(RF_HAA)(iit::rbd::AZ);

    Fcol(RF_HAA) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_HAA);

    // Link LF_lowerleg:
    iit::rbd::transformInertia<Scalar>(LF_lowerleg_Ic, frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg, Ic_spare);
    LF_upperleg_Ic += Ic_spare;

    Fcol(LF_KFE) = LF_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(LF_KFE+6, LF_KFE+6) = Fcol(LF_KFE)(iit::rbd::AZ);

    Fcol(LF_KFE) = frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg * Fcol(LF_KFE);
    DATA(LF_KFE+6, LF_HFE+6) = F(iit::rbd::AZ,LF_KFE);
    DATA(LF_HFE+6, LF_KFE+6) = DATA(LF_KFE+6, LF_HFE+6);
    Fcol(LF_KFE) = frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg * Fcol(LF_KFE);
    DATA(LF_KFE+6, LF_HAA+6) = F(iit::rbd::AZ,LF_KFE);
    DATA(LF_HAA+6, LF_KFE+6) = DATA(LF_KFE+6, LF_HAA+6);
    Fcol(LF_KFE) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_KFE);

    // Link LF_upperleg:
    iit::rbd::transformInertia<Scalar>(LF_upperleg_Ic, frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg, Ic_spare);
    LF_hipassembly_Ic += Ic_spare;

    Fcol(LF_HFE) = LF_upperleg_Ic.col(iit::rbd::AZ);
    DATA(LF_HFE+6, LF_HFE+6) = Fcol(LF_HFE)(iit::rbd::AZ);

    Fcol(LF_HFE) = frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg * Fcol(LF_HFE);
    DATA(LF_HFE+6, LF_HAA+6) = F(iit::rbd::AZ,LF_HFE);
    DATA(LF_HAA+6, LF_HFE+6) = DATA(LF_HFE+6, LF_HAA+6);
    Fcol(LF_HFE) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_HFE);

    // Link LF_hipassembly:
    iit::rbd::transformInertia<Scalar>(LF_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_LF_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(LF_HAA) = LF_hipassembly_Ic.col(iit::rbd::AZ);
    DATA(LF_HAA+6, LF_HAA+6) = Fcol(LF_HAA)(iit::rbd::AZ);

    Fcol(LF_HAA) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_HAA);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<12, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = trunk_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::HyQ::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint RH_KFE, index 11 :
    L(11, 11) = std::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    
    // Joint RH_HFE, index 10 :
    L(10, 10) = std::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    
    // Joint RH_HAA, index 9 :
    L(9, 9) = std::sqrt(L(9, 9));
    
    // Joint LH_KFE, index 8 :
    L(8, 8) = std::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint LH_HFE, index 7 :
    L(7, 7) = std::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint LH_HAA, index 6 :
    L(6, 6) = std::sqrt(L(6, 6));
    
    // Joint RF_KFE, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint RF_HFE, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint RF_HAA, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint LF_KFE, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint LF_HFE, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint LF_HAA, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::HyQ::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 9) * Linv(9, 9));
    inverse(10, 10) =  + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(11, 11) =  + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
}

template <typename TRAIT>
void iit::HyQ::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
}

