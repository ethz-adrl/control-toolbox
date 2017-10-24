/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Lincensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "internal/ADLinearizerBase.h"

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear ControlledSystem using Automatic Differentiation (without code generation)
/*!
 * This class takes a non-linear ControlledSystem \f$ \dot{x} = f(x,u,t) \f$ and computes the linearization
 * around a certain point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \f[
 *   \dot{x} = A x + B u
 * \f]
 *
 * where
 *
 * \f[
 * \begin{aligned}
 * A &= \frac{df}{dx} |_{x=x_s, u=u_s} \\
 * B &= \frac{df}{du} |_{x=x_s, u=u_s}
 * \end{aligned}
 * \f]
 *
 * \note This is generally the most accurate way to generate the linearization of system dynamics together with ADCodegenLinearizer.
 * However, the latter is much faster. Consider using the latter for production code.
 *
 * Unit test \ref AutoDiffLinearizerTest.cpp illustrates the use of the AutoDiffLinearizer.
 *
 *
 * \warning You should ensure that your ControlledSystem is templated on the scalar type and does not contain branching
 * (if/else statements, switch cases etc.)
 *
 *
 * \warning This function still has some issues with pure time dependency
 * \todo Make time an Auto-Diff parameter
 *
 * @tparam dimension of state vector
 * @tparam dimension of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class AutoDiffLinearizer : public internal::ADLinearizerBase<STATE_DIM, CONTROL_DIM, CppAD::AD<double>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    typedef CppAD::AD<double> AD_double;                                         //!< Auto-Diff double type
    typedef internal::ADLinearizerBase<STATE_DIM, CONTROL_DIM, AD_double> Base;  //!< Base class type

    typedef typename Base::state_vector_t state_vector_t;                  //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;              //!< input vector type
    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //!< input Jacobian type

    //! default constructor
    /*!
	 * @param nonlinearSystem non-linear system instance to linearize
	 */
    AutoDiffLinearizer(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, AD_double>> nonlinearSystem)
        : Base(nonlinearSystem)
    {
    }

    //! copy constructor
    AutoDiffLinearizer(const AutoDiffLinearizer& arg) : Base(arg), dFdx_(arg.dFdx_), dFdu_(arg.dFdu_) {}
    //! destructor
    virtual ~AutoDiffLinearizer() {}
    //! deep cloning
    AutoDiffLinearizer<STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new AutoDiffLinearizer<STATE_DIM, CONTROL_DIM>(*this);
    }

    //! get the Jacobian with respect to the state
    /*!
	 * This computes the linearization of the system with respect to the state at a given point \f$ x=x_s \f$, \f$ u=u_s \f$,
	 * i.e. it computes
	 *
	 * \f[
	 * A = \frac{df}{dx} |_{x=x_s, u=u_s}
	 * \f]
	 *
	 * @param x state to linearize at
	 * @param u control to linearize at
	 * @param t time
	 * @return Jacobian wrt state
	 */
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const double t = 0.0) override
    {
        computeA(x, u);
        return dFdx_;
    }


    //! get the Jacobian with respect to the input
    /*!
	 * This computes the linearization of the system with respect to the input at a given point \f$ x=x_s \f$, \f$ u=u_s \f$,
	 * i.e. it computes
	 *
	 * \f[
	 * B = \frac{df}{du} |_{x=x_s, u=u_s}
	 * \f]
	 *
	 * @param x state to linearize at
	 * @param u control to linearize at
	 * @param t time
	 * @return Jacobian wrt input
	 */
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const double t = 0.0) override
    {
        computeB(x, u);
        return dFdu_;
    }


protected:
    //! compute the state Jacobian
    /*!
	 * @param x state to linearize around
	 * @param u input to linearize around
	 */
    void computeA(const state_vector_t& x, const control_vector_t& u)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<double, Eigen::Dynamic, 1> jac(this->A_entries);

        this->f_.SparseJacobianForward(input, this->sparsityA_.sparsity(), this->sparsityA_.row(),
            this->sparsityA_.col(), jac, this->sparsityA_.workJacobian());

        Eigen::Map<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> out(jac.data());

        dFdx_ = out;
    }

    //! compute the input Jacobian
    /*!
	 * @param x state to linearize around
	 * @param u input to linearize around
	 */
    void computeB(const state_vector_t& x, const control_vector_t& u)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<double, Eigen::Dynamic, 1> jac(this->B_entries);

        this->f_.SparseJacobianForward(input, this->sparsityB_.sparsity(), this->sparsityB_.row(),
            this->sparsityB_.col(), jac, this->sparsityB_.workJacobian());

        Eigen::Map<Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>> out(jac.data());

        dFdu_ = out;
    }


    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
};
}
}
