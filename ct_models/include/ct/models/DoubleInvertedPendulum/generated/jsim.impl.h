

//Implementation of default constructor
template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    Link2_Ic(linkInertias.getTensor_Link2())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>& iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_Link1_X_fr_Link2(state);

    // Initializes the composite inertia tensors
    Link1_Ic = linkInertias.getTensor_Link1();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link Link2:
    iit::rbd::transformInertia<Scalar>(Link2_Ic, frcTransf -> fr_Link1_X_fr_Link2, Ic_spare);
    Link1_Ic += Ic_spare;

    F = Link2_Ic.col(iit::rbd::AZ);
    DATA(JOINT2, JOINT2) = F(iit::rbd::AZ);

    F = frcTransf -> fr_Link1_X_fr_Link2 * F;
    DATA(JOINT2, JOINT1) = F(iit::rbd::AZ);
    DATA(JOINT1, JOINT2) = DATA(JOINT2, JOINT1);

    // Link Link1:

    F = Link1_Ic.col(iit::rbd::AZ);
    DATA(JOINT1, JOINT1) = F(iit::rbd::AZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint Joint2, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint Joint1, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
}

template <typename TRAIT>
void iit::ct_DoubleInvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
}

