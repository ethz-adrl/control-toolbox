
 //Implementation of default constructor
 template<typename TRAIT>
 iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
     linkInertias(inertiaProperties),
     frcTransf( &forceTransforms ),
     link2_Ic(linkInertias.getTensor_link2())
 {
     //Initialize the matrix itself
     this->setZero();
 }

 #define DATA tpl::JSIM<TRAIT>::operator()
 #define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
 #define F(i,j) DATA((i),(j)+6)

 template <typename TRAIT>
 const typename iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>& iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

     // Precomputes only once the coordinate transforms:
     frcTransf -> fr_link1_X_fr_link2(state);
     frcTransf -> fr_body_X_fr_link1(state);

     // Initializes the composite inertia tensors
     body_Ic = linkInertias.getTensor_body();
     link1_Ic = linkInertias.getTensor_link1();

     // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

     // Link link2:
     iit::rbd::transformInertia(link2_Ic, frcTransf -> fr_link1_X_fr_link2, Ic_spare);
     link1_Ic += Ic_spare;

     Fcol(JB) = link2_Ic.col(iit::rbd::AZ);
     DATA(JB+6, JB+6) = Fcol(JB)(iit::rbd::AZ);

     Fcol(JB) = frcTransf -> fr_link1_X_fr_link2 * Fcol(JB);
     DATA(JB+6, JA+6) = F(iit::rbd::AZ,JB);
     DATA(JA+6, JB+6) = DATA(JB+6, JA+6);
     Fcol(JB) = frcTransf -> fr_body_X_fr_link1 * Fcol(JB);

     // Link link1:
     iit::rbd::transformInertia(link1_Ic, frcTransf -> fr_body_X_fr_link1, Ic_spare);
     body_Ic += Ic_spare;

     Fcol(JA) = link1_Ic.col(iit::rbd::AZ);
     DATA(JA+6, JA+6) = Fcol(JA)(iit::rbd::AZ);

     Fcol(JA) = frcTransf -> fr_body_X_fr_link1 * Fcol(JA);

     // Copies the upper-right block into the lower-left block, after transposing
     JSIM<TRAIT>:: template block<2, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 2>(0,6)).transpose();
     // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
     JSIM<TRAIT>:: template block<6,6>(0,0) = body_Ic;
     return *this;
 }

 #undef DATA
 #undef F

 template <typename TRAIT>
 void iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>::computeL() {
     L = this -> template triangularView<Eigen::Lower>();
     // Joint jB, index 1 :
     L(1, 1) = std::sqrt(L(1, 1));
     L(1, 0) = L(1, 0) / L(1, 1);
     L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
     
     // Joint jA, index 0 :
     L(0, 0) = std::sqrt(L(0, 0));
     
 }

 template <typename TRAIT>
 void iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>::computeInverse() {
     computeLInverse();

     inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
     inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
     inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
     inverse(0, 1) = inverse(1, 0);
 }

 template <typename TRAIT>
 void iit::ct_quadrotor::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
     //assumes L has been computed already
     Linv(0, 0) = 1 / L(0, 0);
     Linv(1, 1) = 1 / L(1, 1);
     Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
 }


