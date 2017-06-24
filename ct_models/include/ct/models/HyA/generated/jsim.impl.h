

//Implementation of default constructor
template<typename TRAIT>
iit::ct_HyA::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    Wrist_FE_Ic(linkInertias.getTensor_Wrist_FE())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::ct_HyA::dyn::tpl::JSIM<TRAIT>& iit::ct_HyA::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_Wrist_R_X_fr_Wrist_FE(state);
    frcTransf -> fr_Elbow_FE_X_fr_Wrist_R(state);
    frcTransf -> fr_Humerus_R_X_fr_Elbow_FE(state);
    frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R(state);
    frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE(state);

    // Initializes the composite inertia tensors
    Shoulder_AA_Ic = linkInertias.getTensor_Shoulder_AA();
    Shoulder_FE_Ic = linkInertias.getTensor_Shoulder_FE();
    Humerus_R_Ic = linkInertias.getTensor_Humerus_R();
    Elbow_FE_Ic = linkInertias.getTensor_Elbow_FE();
    Wrist_R_Ic = linkInertias.getTensor_Wrist_R();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link Wrist_FE:
    iit::rbd::transformInertia(Wrist_FE_Ic, frcTransf -> fr_Wrist_R_X_fr_Wrist_FE, Ic_spare);
    Wrist_R_Ic += Ic_spare;

    F = Wrist_FE_Ic.col(iit::rbd::AZ);
    DATA(WFE, WFE) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Wrist_R_X_fr_Wrist_FE * F;
    DATA(WFE, WR) = F(iit::rbd::AZ);
    DATA(WR, WFE) = DATA(WFE, WR);
    F = frcTransf -> fr_Elbow_FE_X_fr_Wrist_R * F;
    DATA(WFE, EFE) = F(iit::rbd::AZ);
    DATA(EFE, WFE) = DATA(WFE, EFE);
    F = frcTransf -> fr_Humerus_R_X_fr_Elbow_FE * F;
    DATA(WFE, HR) = F(iit::rbd::AZ);
    DATA(HR, WFE) = DATA(WFE, HR);
    F = frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R * F;
    DATA(WFE, SFE) = F(iit::rbd::AZ);
    DATA(SFE, WFE) = DATA(WFE, SFE);
    F = frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE * F;
    DATA(WFE, SAA) = F(iit::rbd::AZ);
    DATA(SAA, WFE) = DATA(WFE, SAA);

    // Link Wrist_R:
    iit::rbd::transformInertia(Wrist_R_Ic, frcTransf -> fr_Elbow_FE_X_fr_Wrist_R, Ic_spare);
    Elbow_FE_Ic += Ic_spare;

    F = Wrist_R_Ic.col(iit::rbd::AZ);
    DATA(WR, WR) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Elbow_FE_X_fr_Wrist_R * F;
    DATA(WR, EFE) = F(iit::rbd::AZ);
    DATA(EFE, WR) = DATA(WR, EFE);
    F = frcTransf -> fr_Humerus_R_X_fr_Elbow_FE * F;
    DATA(WR, HR) = F(iit::rbd::AZ);
    DATA(HR, WR) = DATA(WR, HR);
    F = frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R * F;
    DATA(WR, SFE) = F(iit::rbd::AZ);
    DATA(SFE, WR) = DATA(WR, SFE);
    F = frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE * F;
    DATA(WR, SAA) = F(iit::rbd::AZ);
    DATA(SAA, WR) = DATA(WR, SAA);

    // Link Elbow_FE:
    iit::rbd::transformInertia(Elbow_FE_Ic, frcTransf -> fr_Humerus_R_X_fr_Elbow_FE, Ic_spare);
    Humerus_R_Ic += Ic_spare;

    F = Elbow_FE_Ic.col(iit::rbd::AZ);
    DATA(EFE, EFE) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Humerus_R_X_fr_Elbow_FE * F;
    DATA(EFE, HR) = F(iit::rbd::AZ);
    DATA(HR, EFE) = DATA(EFE, HR);
    F = frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R * F;
    DATA(EFE, SFE) = F(iit::rbd::AZ);
    DATA(SFE, EFE) = DATA(EFE, SFE);
    F = frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE * F;
    DATA(EFE, SAA) = F(iit::rbd::AZ);
    DATA(SAA, EFE) = DATA(EFE, SAA);

    // Link Humerus_R:
    iit::rbd::transformInertia(Humerus_R_Ic, frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R, Ic_spare);
    Shoulder_FE_Ic += Ic_spare;

    F = Humerus_R_Ic.col(iit::rbd::AZ);
    DATA(HR, HR) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Shoulder_FE_X_fr_Humerus_R * F;
    DATA(HR, SFE) = F(iit::rbd::AZ);
    DATA(SFE, HR) = DATA(HR, SFE);
    F = frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE * F;
    DATA(HR, SAA) = F(iit::rbd::AZ);
    DATA(SAA, HR) = DATA(HR, SAA);

    // Link Shoulder_FE:
    iit::rbd::transformInertia(Shoulder_FE_Ic, frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE, Ic_spare);
    Shoulder_AA_Ic += Ic_spare;

    F = Shoulder_FE_Ic.col(iit::rbd::AZ);
    DATA(SFE, SFE) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Shoulder_AA_X_fr_Shoulder_FE * F;
    DATA(SFE, SAA) = F(iit::rbd::AZ);
    DATA(SAA, SFE) = DATA(SFE, SAA);

    // Link Shoulder_AA:

    F = Shoulder_AA_Ic.col(iit::rbd::AZ);
    DATA(SAA, SAA) = F(iit::rbd::AZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint WFE, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(2, 0) = L(2, 0) - L(5, 2) * L(5, 0);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    L(1, 0) = L(1, 0) - L(5, 1) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint WR, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint EFE, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint HR, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint SFE, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint SAA, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::JSIM<TRAIT>::computeInverse() {
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
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 0) * Linv(2, 0)) + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 0) * Linv(1, 0)) + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
}

template <typename TRAIT>
void iit::ct_HyA::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 1) * L(1, 0)) + (Linv(5, 2) * L(2, 0)) + (Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
}

