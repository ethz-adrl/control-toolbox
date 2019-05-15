/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef _IIT_RBD_INERTIAMATRIX_H_
#define _IIT_RBD_INERTIAMATRIX_H_

#include "rbd.h"


namespace iit {
namespace rbd {

namespace tpl {
/**
 * Dense 6x6 matrix that represents the 6D spatial inertia tensor.
 * See chapther 2 of Featherstone's "Rigid body dynamics algorithms".
 */
template<typename SCALAR>
class InertiaMatrixDense : public Core<SCALAR>::Matrix66
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR Scalar;
private:
    typedef Core<Scalar> Cores;
    typedef typename Cores::Matrix66 Base;
    typedef typename Cores::Matrix33 Mat33;
    typedef typename Cores::Vector3  Vec3;
    typedef MatrixBlock<const Base,3,3> Block33_const;
    typedef MatrixBlock<Base,3,3> Block33_t;

public:
    template<typename OtherDerived>
    InertiaMatrixDense<SCALAR>& operator= (const MatrixBase<OtherDerived>& other);

    template<typename OtherDerived>
    InertiaMatrixDense<SCALAR>& operator+= (const MatrixBase<OtherDerived>& other);

    InertiaMatrixDense();
    /**
     * See fill()
     */
    InertiaMatrixDense(Scalar m, const Vec3& com, const Mat33& I);

public:
    /**
     * Sets this 6x6 inertia tensor according to the given inertia properties.
     * All the values (ie the COM and the 3x3 tensor) must be expressed in the
     * same reference frame.
     *
     * No consistency checks are performed (Note: possibly changing in future).
     *
     * \param mass the total mass of the body
     * \param com the 3D vector with the position of the center of mass
     * \param tensor the classical 3x3 inertia tensor; this parameter should be
     *    expressed in the same coordinate frame as the center-of-mass vector.
     *    In other words, it is NOT treated as the inertia tensor with respect
     *    to a frame with origin in the center-of-mass, and the parallel axis
     *    theorem is NOT applied. The given tensor is copied as it is, in the
     *    appropriate 3x3 sub-block of this spatial tensor.
     */
    void fill(Scalar m, const Vec3& com, const Mat33& tensor);

    /** \name Components getters **/
    ///@{
    /**
     * @return the current value of the mass of the rigid body modeled by this
     * tensor
     */
    Scalar getMass() const;
    /**
     * @return the position of the center-of-mass of the rigid body modeled by
     *   this spatial tensor
     */
    Vec3 getCOM() const;
    /**
     * @return the 3x3 block of this spatial tensor which corresponds to the
     *   sole rotational inertia, that is, the classical inertia tensor
     */
    const Block33_const get3x3Tensor() const;
    ///@}

    /** \name Modifiers **/
    ///@{
    /**
     * Scales the whole tensor according to the new value of the mass.
     *
     * The changes guarantee that the matrix remains positive definite (assuming
     * it was so before).
     *
     * This method does NOT change the center-of-mass property, while it does
     * change the moments of inertia. Intuitively, calling this method corresponds
     * to changing the mass-density of the body leaving its size and geometry
     * untouched.
     *
     * @param newMass the new value of the mass (always expressed in Kilograms);
     *    it MUST be positive, no checks are performed
     */
    void changeMass(Scalar m);
    /**
     * Changes the position of the Center-Of-Mass of the rigid body modeled by
     * this tensor.
     *
     * In addition to the two off-diagonal 3x3 blocks, this method also modifies
     * the 3x3 block that corresponds to the classical inertia tensor, to keep
     * it consistent with the position of the center of mass. It does not change
     * the mass property. Cfr. chapter 2 of Featherstone's book on rigid body
     * dynamics algorithms.
     * TODO show some equations
     *
     * @param newcom a 3D vector specifying the position of the center of mass,
     *   expressed in meters
     */
    void changeCOM(const Vec3& newcom);
    /**
     * Simply sets the 3x3 block that corresponds to the classical rotational
     * inertia
     * @param tensor the new 3x3 rotational inertia tensor
     */
    void changeRotationalInertia(const Mat33& tensor);
    ///@}
protected:
    void setTheFixedZeros();

    template<typename Vector>
    void setSkewSymmetricBlock(
            const MatrixBase<Vector>& v,
            Block33_t block);
private:
    void set(Scalar m, const Vec3& com, const Mat33& I);

};

template<typename SCALAR>
class InertiaMatrixSparse : public SparseMatrix<SCALAR>
{
    typedef SparseMatrix<SCALAR> Base;
};


#define block33 this->template block<3,3>
#define data    (this->operator())
#define TPL     template<typename S>
#define CLASS   InertiaMatrixDense<S>


/* I do not care here about the creation of temporaries, thus I use const
 * refs to actual Eigen matrices, rather than the common base of the
 * matrix expression. If you haven't read Eigen docs, this comment is
 * completely obscure!
 */
TPL
inline CLASS::InertiaMatrixDense() : Base() {
    setTheFixedZeros();
}

/**
 * Initializes this 6x6 tensor according to the given inertia parameters.
 * \see fill()
 */
TPL
inline CLASS::InertiaMatrixDense(
        Scalar mass, const Vec3& cogPosition, const Mat33& tensor)
: Base()
{
    setTheFixedZeros();
    set(mass, cogPosition, tensor);
}

TPL
inline void CLASS::fill(Scalar mass, const Vec3& comPosition,
        const Mat33& tensor)
{
    set(mass, comPosition, tensor);
}

TPL
inline typename CLASS::Scalar CLASS::getMass() const
{
    return data(LX,LX);
}

TPL
inline typename CLASS::Vec3 CLASS::getCOM() const
{
    return Vec3(
            data(AZ,LY)/data(LX,LX), // X coordinate of the COM
            data(AX,LZ)/data(LX,LX), // Y
            data(AY,LX)/data(LX,LX));// Z
}

TPL
inline const typename CLASS::Block33_const CLASS::get3x3Tensor() const
{
    return block33(AX,AX);
}

TPL
inline void CLASS::changeMass(Scalar newMass) {
    // Note the use of indices AX and hard-coded 0, to make it independent from
    //  the convention angular/linear
    this->block<3,6>(AX,0) *= newMass/getMass();
    block33(LX,AX) = block33(AX,LX).transpose();
    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = newMass;
}

TPL
inline void CLASS::changeCOM(const Vec3& newcom)
{
    // Update the angular-linear block according to the new COM position
    setSkewSymmetricBlock(  getMass() * newcom, block33(AX,LX));

    // Correct the 3x3 tensor. Use the block(AX,LX) which reflects the new COM
    //  and the block(LX,AX) which instead is still based on the previous value.
    //     I = I - m(cx)(cx)^T + m(c'x)(c'x)^T
    //  where cx is the skew symmetric matrix for the old COM vector, while
    //  c'x is the one for the new COM vector.
    block33(AX,AX) += (
            ( block33(AX,LX) * block33(AX,LX).transpose() ) -
            ( block33(LX,AX).transpose() * block33(LX,AX) )
                ) / getMass();
    // Update the linear-angular block
    block33(LX,AX) = block33(AX,LX).transpose();
    // alternatively:
    // setSkewSymmetricBlock( - getMass() * newcom, block33(LX,AX));
}

TPL
inline void CLASS::changeRotationalInertia(const Mat33& tensor)
{
    block33(AX,AX) = tensor;
}

TPL
inline void CLASS::set(
        Scalar mass,
        const Vec3& com,
        const Mat33& tensor)
{
    block33(AX,AX) = tensor;// + mass * comCrossMx * comCrossMx.transpose();

    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = data(LX,AY) = mass*com(Z) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = data(LZ,AX) = mass*com(Y) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = data(LY,AZ) = mass*com(X) );
    // Equivalent, just slightly less efficient (probably because of Eigen's blocks and repeated product):
    //setSkewSymmetricBlock(  mass * com, block33(AX,LX));
    //setSkewSymmetricBlock(- mass * com, block33(LX,AX));

    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = mass;
}

TPL
template<typename OtherDerived>
inline CLASS& CLASS::operator=
        (const MatrixBase<OtherDerived>& other)
{
    // Here we silently assume that 'other' is also an inertia...
    //   Type safety would suggest to prevent the assignment of anything but
    // another inertia, but that would prevent, for example, the assignment
    // of matrix expressions. We also do not want to perform any check, for
    // performance reasons (remember this library is meant primarily to support
    // code generation, not to be a robust API for user applications).
    block33(AX,AX) = other.template block<3,3>(AX,AX);
    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = data(LX,AY) = other(LX,AY) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = data(LZ,AX) = other(LZ,AX) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = data(LY,AZ) = other(LY,AZ) );
    data(LX,LX) = data(LY,LY) = data(LZ,LZ) = other(LX,LX);
    return *this;
}

TPL
template<typename OtherDerived>
inline CLASS& CLASS::operator+=
        (const MatrixBase<OtherDerived>& other)
{
    block33(AX,AX) += other.template block<3,3>(AX,AX);
    data(AX,LY) = data(LY,AX) = - ( data(AY,LX) = (data(LX,AY) += other(LX,AY)) );
    data(AZ,LX) = data(LX,AZ) = - ( data(AX,LZ) = (data(LZ,AX) += other(LZ,AX)) );
    data(AY,LZ) = data(LZ,AY) = - ( data(AZ,LY) = (data(LY,AZ) += other(LY,AZ)) );
    data(LX,LX) = data(LY,LY) = (data(LZ,LZ) += other(LX,LX));
    return *this;
}

TPL
inline void CLASS::setTheFixedZeros()
{
    block33(LX,LX).setZero(); // the diagonal won't be zero, but who cares
    data(AX,LX) = data(AY,LY) = data(AZ,LZ) =
            data(LX,AX) = data(LY,AY) = data(LZ,AZ) = 0;
}

TPL
template<typename Vector>
inline void CLASS::setSkewSymmetricBlock(
        const MatrixBase<Vector>& v, Block33_t block)
{
    block(X,Y) = - ( block(Y,X) = v(Z) );
    block(Z,X) = - ( block(X,Z) = v(Y) );
    block(Y,Z) = - ( block(Z,Y) = v(X) );
}


#undef block33
#undef data
#undef TPL
#undef CLASS
}

using InertiaMatrixDense = tpl::InertiaMatrixDense<double>;

}
}


#endif /* INERTIAMATRIX_H_ */
