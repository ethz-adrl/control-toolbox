#ifndef IIT_CT_QUADROTOR_JSIM_H_
#define IIT_CT_QUADROTOR_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include <iit/rbd/traits/DoubleTrait.h>


namespace iit {
namespace ct_quadrotor {
namespace dyn {

namespace tpl{

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot ct_quadrotor.
 */
template <class TRAIT>
class JSIM : public iit::rbd::StateDependentMatrix<iit::ct_quadrotor::JointState, 8, 8, JSIM<TRAIT>>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        typedef iit::rbd::StateDependentMatrix<iit::ct_quadrotor::JointState, 8, 8, JSIM<TRAIT>> Base;
    public:
        typedef typename TRAIT::Scalar SCALAR;
        typedef typename Base::Index Index;
        typedef Eigen::Matrix<SCALAR,8,8> MatrixType;
        /** The type of the F sub-block of the floating-base JSIM */
        typedef const Eigen::Block<const MatrixType,6,2> BlockF_t;
        /** The type of the fixed-base sub-block of the JSIM */
        typedef const Eigen::Block<const MatrixType,2,2> BlockFixedBase_t;
        typedef InertiaProperties<TRAIT> IProperties;
        typedef iit::ct_quadrotor::tpl::ForceTransforms<TRAIT> FTransforms;
        typedef iit::rbd::tpl::InertiaMatrixDense<SCALAR> InertiaMatrix;

    public:
        JSIM(IProperties&, FTransforms&);
        ~JSIM() {}

        const JSIM& update(const iit::ct_quadrotor::JointState&);


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

        /**
         * The spatial composite-inertia tensor of the robot base,
         * ie the inertia of the whole robot for the current configuration.
         * According to the convention of this class about the layout of the
         * floating-base JSIM, this tensor is the 6x6 upper left corner of
         * the JSIM itself.
         * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
         *   tensor of the whole robot, according to the last joints configuration
         *   used to update this JSIM
         */
        const InertiaMatrix& getWholeBodyInertia() const;
        /**
         * The matrix that maps accelerations in the actual joints of the robot
         * to the spatial force acting on the floating-base of the robot.
         * This matrix is the F sub-block of the JSIM in Featherstone's notation.
         * \return the 6x2 upper right block of this JSIM
         */
        const BlockF_t getF() const;
        /**
         * The submatrix of this JSIM related only to the actual joints of the
         * robot (as for a fixed-base robot).
         * This matrix is the H sub-block of the JSIM in Featherstone's notation.
         * \return the 2x2 lower right block of this JSIM,
         *   which correspond to the fixed-base JSIM
         */
        const BlockFixedBase_t getFixedBaseBlock() const;
    protected:
        /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
        void computeLInverse();
    private:
        IProperties& linkInertias;
        FTransforms* frcTransf;

        // The composite-inertia tensor for each link
        InertiaMatrix body_Ic;
        InertiaMatrix link1_Ic;
        const InertiaMatrix& link2_Ic;
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

template <class TRAIT>
inline const typename JSIM<TRAIT>::InertiaMatrix& JSIM<TRAIT>::getWholeBodyInertia() const {
    return body_Ic;
}

template <class TRAIT>
inline const typename JSIM<TRAIT>::BlockF_t JSIM<TRAIT>::getF() const {
    return JSIM<TRAIT>:: template block<6,2>(0,6);
}

template <class TRAIT>
inline const typename JSIM<TRAIT>::BlockFixedBase_t JSIM<TRAIT>::getFixedBaseBlock() const{
    return JSIM<TRAIT>:: template block<2,2>(6,6);
}

} // namespace tpl

typedef tpl::JSIM<rbd::DoubleTrait> JSIM;

}
}
}

#include "jsim.impl.h"

#endif
