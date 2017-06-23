#ifndef IIT_CT_HYA_JSIM_H_
#define IIT_CT_HYA_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>


namespace iit {
namespace ct_HyA {
namespace dyn {

namespace tpl{

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot ct_HyA.
 */
template <class TRAIT>
class JSIM : public iit::rbd::StateDependentMatrix<iit::ct_HyA::tpl::JointState<typename TRAIT::Scalar>, 6, 6, JSIM<TRAIT>>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        typedef iit::rbd::StateDependentMatrix<iit::ct_HyA::JointState, 6, 6, JSIM<TRAIT>> Base;
    public:
        typedef typename TRAIT::Scalar SCALAR;
        typedef typename Base::Index Index;
        typedef Eigen::Matrix<SCALAR,6,6> MatrixType;
        typedef InertiaProperties<TRAIT> IProperties;
        typedef iit::ct_HyA::tpl::ForceTransforms<TRAIT> FTransforms;
        typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;
        typedef iit::ct_HyA::tpl::JointState<SCALAR> JointState;
        typedef typename iit::rbd::Core<SCALAR>::ForceVector ForceVector;

    public:
        JSIM(IProperties&, FTransforms&);
        ~JSIM() {}

        const JSIM& update(const JointState&);


        /**
         * Computes and saves the matrix L of the L^T L factorization of this JSIM.
         */
        void computeL();
        /**
         * Computes and saves the inverse of this JSIM.
         * This function assumes that computeL() has been called already, since it
         * uses L to compute the inverse. The algorithm takes advantage of the branch
         * induced sparsity of the robot, if any.
         */
        void computeInverse();
        /**
         * Returns an unmodifiable reference to the matrix L. See also computeL()
         */
        const MatrixType& getL() const;
        /**
         * Returns an unmodifiable reference to the inverse of this JSIM
         */
        const MatrixType& getInverse() const;

    protected:
        /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
        void computeLInverse();
    private:
        IProperties& linkInertias;
        FTransforms* frcTransf;

        // The composite-inertia tensor for each link
        InertiaMatrix Shoulder_AA_Ic;
        InertiaMatrix Shoulder_FE_Ic;
        InertiaMatrix Humerus_R_Ic;
        InertiaMatrix Elbow_FE_Ic;
        InertiaMatrix Wrist_R_Ic;
        const InertiaMatrix& Wrist_FE_Ic;
        InertiaMatrix Ic_spare;

        MatrixType L;
        MatrixType Linv;
        MatrixType inverse;
};

template <class TRAIT>
inline const typename JSIM<TRAIT>::MatrixType& JSIM<TRAIT>::getL() const {
    return L;
}

template <class TRAIT>
inline const typename JSIM<TRAIT>::MatrixType& JSIM<TRAIT>::getInverse() const {
    return inverse;
}


} // namespace tpl

typedef tpl::JSIM<rbd::DoubleTrait> JSIM;

}
}
}

#include "jsim.impl.h"

#endif
