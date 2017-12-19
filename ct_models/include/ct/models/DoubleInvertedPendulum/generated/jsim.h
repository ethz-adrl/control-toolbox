#ifndef IIT_CT_DOUBLEINVERTEDPENDULUM_JSIM_H_
#define IIT_CT_DOUBLEINVERTEDPENDULUM_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>


namespace iit {
namespace ct_DoubleInvertedPendulum {
namespace dyn {

namespace tpl {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot ct_DoubleInvertedPendulum.
 */
template <typename TRAIT>
class JSIM : public iit::rbd::StateDependentMatrix<iit::ct_DoubleInvertedPendulum::tpl::JointState<typename TRAIT::Scalar>, 2, 2, JSIM<TRAIT>>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        typedef iit::rbd::StateDependentMatrix<iit::ct_DoubleInvertedPendulum::tpl::JointState<typename TRAIT::Scalar>, 2, 2, JSIM<TRAIT>> Base;
    public:
        typedef typename TRAIT::Scalar Scalar;
        typedef typename iit::ct_DoubleInvertedPendulum::tpl::JointState<Scalar> JointState;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef typename Base::Index Index;
        typedef typename iit::rbd::PlainMatrix<Scalar, 2, 2> MatrixType;
        typedef InertiaProperties<TRAIT> IProperties;
        typedef iit::ct_DoubleInvertedPendulum::tpl::ForceTransforms<TRAIT> FTransforms;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
        typedef typename CoreS::ForceVector ForceVector;

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
        InertiaMatrix Link1_Ic;
        const InertiaMatrix& Link2_Ic;
        InertiaMatrix Ic_spare;

        MatrixType L;
        MatrixType Linv;
        MatrixType inverse;
};

template <typename TRAIT>
inline const typename JSIM<TRAIT>::MatrixType& JSIM<TRAIT>::getL() const {
    return L;
}

template <typename TRAIT>
inline const typename JSIM<TRAIT>::MatrixType& JSIM<TRAIT>::getInverse() const {
    return inverse;
}


}

typedef tpl::JSIM<rbd::DoubleTrait> JSIM;

}
}
}

#include "jsim.impl.h"

#endif
