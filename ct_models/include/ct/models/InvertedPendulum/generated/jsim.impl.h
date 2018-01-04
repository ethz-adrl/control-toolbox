

//Implementation of default constructor
template <typename TRAIT>
iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    Link1_Ic(linkInertias.getTensor_Link1())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>& iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:

    // Initializes the composite inertia tensors

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link Link1:

    F = Link1_Ic.col(iit::rbd::AZ);
    DATA(JOINT1, JOINT1) = F(iit::rbd::AZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint Joint1, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
}

template <typename TRAIT>
void iit::ct_InvertedPendulum::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
}

