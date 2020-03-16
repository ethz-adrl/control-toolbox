/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include <ct/core/internal/autodiff/SparsityPattern.h>

namespace ct {
namespace core {
namespace internal {

//! Base class for Auto-Diff and Auto-Diff Codegen linearization for system dynamics
/*!
 * This class contains shared code between Auto-Diff and Auto-Diff Codegen linearization.
 *
 * \tparam STATE_DIM dimension of state vector
 * \tparam CONTROL_DIM dimension of control vector
 * \tparam SCALAR scalar type
 * \tparam TIME type of time variable of dynamics
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME>
class DynamicsLinearizerADBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //TODO this exclusive list, the following typedef, and getOutScalarType() should be generalized
    static_assert((std::is_same<SCALAR, CppAD::AD<double>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<double>>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<float>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<float>>>::value),
        "SCALAR template parameter in ADLinearizerBase should either be of CppAD::AD<XX> or "
        "CppAD::AD<CppAD::cg::XX> type with XX being float or double");

    typedef typename std::conditional<(std::is_same<SCALAR, CppAD::AD<double>>::value) ||
                                          (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<double>>>::value),
        double,
        float>::type OUT_SCALAR;  //!< scalar type of resulting linear system

    typedef StateVector<STATE_DIM, OUT_SCALAR> state_vector_t;
    typedef ControlVector<CONTROL_DIM, OUT_SCALAR> control_vector_t;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_ad_t;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_ad_t;

    typedef StateMatrix<STATE_DIM, OUT_SCALAR> state_matrix_t;
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, OUT_SCALAR> state_control_matrix_t;

    typedef std::function<void(const state_vector_ad_t&, const TIME&, const control_vector_ad_t&, state_vector_ad_t&)>
        dynamics_fct_t;  //!< dynamics function signature

    //! default constructor
    /*!
	 * @param dyn non-linear system to linearize
	 */
    DynamicsLinearizerADBase(dynamics_fct_t dyn) : dynamics_fct_(dyn) { initialize(); }
    //! copy constructor
    DynamicsLinearizerADBase(const DynamicsLinearizerADBase& arg) : dynamics_fct_(arg.dynamics_fct_)
    {
        setupSparsityA();
        setupSparsityB();
        f_ = arg.f_;
    }

    template <typename T = std::string>  // do not use this template argument
    typename std::enable_if<std::is_same<OUT_SCALAR, double>::value, T>::type getOutScalarType() const
    {
        return "double";
    }

    template <typename T = std::string>  // do not use this template argument
    typename std::enable_if<std::is_same<OUT_SCALAR, float>::value, T>::type getOutScalarType() const
    {
        return "float";
    }

    virtual ~DynamicsLinearizerADBase() = default;

protected:
    const size_t A_entries = STATE_DIM * STATE_DIM;    //!< number of entries in the state Jacobian
    const size_t B_entries = STATE_DIM * CONTROL_DIM;  //!< number of entries in the input Jacobian
    const size_t FullJac_entries =
        (STATE_DIM + CONTROL_DIM) * STATE_DIM;  //!< number of entries in the stacked Jacobian

    //! initialize all utilities
    /*!
	 * Records the model function and sets up the sparsity patterns for the Jacobians.
	 */
    void initialize()
    {
        recordTerms();
        setupSparsityA();
        setupSparsityB();
    }

    //! record the model
    void recordTerms()
    {
        // input vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> x(STATE_DIM + CONTROL_DIM);
        // init to rand to avoid floating point problems in user's code
        x.setRandom();

        // declare x as independent
        CppAD::Independent(x);

        // create fixed size types since CT uses fixed size types
        state_vector_ad_t xFixed = x.template head<STATE_DIM>();
        control_vector_ad_t uFixed = x.template tail<CONTROL_DIM>();

        state_vector_ad_t dxFixed;

        dynamics_fct_(xFixed, TIME(0.0), uFixed, dxFixed);

        // output vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> dx(STATE_DIM);
        dx = dxFixed;

        // store operation sequence in f: x -> dx and stop recording
        CppAD::ADFun<typename SCALAR::value_type> f(x, dx);

        f.optimize();

        f_ = f;
    }

    //! setup the sparsity of the state Jacobian
    void setupSparsityA()
    {
        // the derivative is a STATE_DIM*STATE_DIM Matrix:
        // dF/dx = [ A, B ]^T
        Eigen::Matrix<bool, STATE_DIM + CONTROL_DIM, STATE_DIM> sparsity;
        sparsity.setZero();
        sparsity.template topRows<STATE_DIM>().setOnes();

        sparsityA_.initPattern(sparsity);
        sparsityA_.clearWork();
    }

    //! setup the sparsity of the input Jacobian
    void setupSparsityB()
    {
        // the derivative is a STATE_DIM*CONTROL_DIM Matrix:
        // dF/dx = [ A, B ]^T
        Eigen::Matrix<bool, STATE_DIM + CONTROL_DIM, STATE_DIM> sparsity;
        sparsity.setZero();
        sparsity.template bottomRows<CONTROL_DIM>().setOnes();

        sparsityB_.initPattern(sparsity);
        sparsityB_.clearWork();
    }

    dynamics_fct_t dynamics_fct_;                  //!< function handle to system dynamics
    CppAD::ADFun<typename SCALAR::value_type> f_;  //!< Auto-Diff function

    SparsityPattern sparsityA_;  //!< sparsity pattern of the state Jacobian
    SparsityPattern sparsityB_;  //!< sparsity pattern of the input Jacobian
};

}  // namespace internal
}  // namespace core
}  // namespace ct

#endif