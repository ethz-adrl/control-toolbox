#ifndef IIT_TESTIRB4600_JSIM_H_
#define IIT_TESTIRB4600_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>


namespace iit {
namespace testirb4600 {
namespace dyn {

namespace tpl{

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot testirb4600.
 */
template <class TRAIT>
class JSIM : public iit::rbd::StateDependentMatrix<iit::testirb4600::JointState, 6, 6, JSIM<TRAIT>>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        typedef iit::rbd::StateDependentMatrix<iit::testirb4600::JointState, 6, 6, JSIM<TRAIT>> Base;
    public:
        typedef typename TRAIT::Scalar SCALAR;
        typedef typename Base::Index Index;
        typedef Eigen::Matrix<SCALAR,6,6> MatrixType;
        typedef InertiaProperties<TRAIT> IProperties;
        typedef iit::testirb4600::tpl::ForceTransforms<TRAIT> FTransforms;
        typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;

    public:
        JSIM(IProperties&, FTransforms&);
        ~JSIM() {}

        const JSIM& update(const iit::testirb4600::JointState&);


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
        InertiaMatrix link1_Ic;
        InertiaMatrix link2_Ic;
        InertiaMatrix link3_Ic;
        InertiaMatrix link4_Ic;
        InertiaMatrix link5_Ic;
        const InertiaMatrix& link6_Ic;
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
