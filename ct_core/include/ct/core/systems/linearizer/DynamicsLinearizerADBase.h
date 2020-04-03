/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include <ct/core/internal/autodiff/SparsityPattern.h>
#include <ct/core/internal/traits/EvaluatorOutTypeTraits.h>

namespace ct {
namespace core {
namespace internal {

//! Base class for Auto-Diff and Auto-Diff Codegen linearization for system dynamics, CppAD-specific
/*!
 * This class contains shared code between Auto-Diff and Auto-Diff Codegen linearization.
 *
 * \tparam STATE_DIM dimension of state vector
 * \tparam CONTROL_DIM dimension of control vector
 * \tparam SCALAR scalar type
 * \tparam TIME type of time variable of dynamics
 */
template <typename MANIFOLD_AD, size_t CONTROL_DIM, bool CONT_T>
class DynamicsLinearizerADBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD_AD::TangentDim;

    using SCALAR = typename MANIFOLD_AD::Scalar;

    //TODO this exclusive list, the following typedef, and getOutScalarType() should be generalized
    static_assert((std::is_same<SCALAR, CppAD::AD<double>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<double>>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<float>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<float>>>::value),
        "SCALAR template parameter in ADLinearizerBase should either be of CppAD::AD<XX> or "
        "CppAD::AD<CppAD::cg::XX> type with XX being float or double");

    using OUT_SCALAR = typename ct::core::get_out_type<SCALAR>::type;

    typedef ControlVector<CONTROL_DIM, OUT_SCALAR> control_vector_t;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_ad_t;
    using Time_ad_t = typename ControlledSystem<MANIFOLD_AD, CONTROL_DIM, CONT_T>::Time_t;

    typedef StateMatrix<STATE_DIM, OUT_SCALAR> state_matrix_t;
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, OUT_SCALAR> state_control_matrix_t;

    using dynamics_fct_t = std::function<
        void(const MANIFOLD_AD&, const Time_ad_t&, const control_vector_ad_t&, typename MANIFOLD_AD::Tangent&)>;
    using lift_fct_t = std::function<typename MANIFOLD_AD::Tangent(const MANIFOLD_AD&)>;
    using retract_fct_t = std::function<MANIFOLD_AD(const typename MANIFOLD_AD::Tangent&)>;

    //! default constructor
    /*!
	 * @param dyn non-linear system to linearize
	 */
    DynamicsLinearizerADBase(dynamics_fct_t dyn, lift_fct_t lift, retract_fct_t retract)
        : dFdx_(state_matrix_t::Zero()),
          dFdu_(state_control_matrix_t::Zero()),
          dynamics_fct_(dyn),
          lift_fct_(lift),
          retract_fct_(retract)
    {
        initialize();
    }
    //! copy constructor
    DynamicsLinearizerADBase(const DynamicsLinearizerADBase& arg)
        : dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          dynamics_fct_(arg.dynamics_fct_),
          lift_fct_(arg.lift_fct_),
          retract_fct_(arg.retract_fct_)
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

    virtual ~DynamicsLinearizerADBase() {}
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
        const size_t tangent_dim = MANIFOLD_AD::TangentDim;

        // input vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> var(tangent_dim + CONTROL_DIM);
        // init to rand to avoid floating point problems in user's code
        var.setRandom();

        // declare var as independent
        CppAD::Independent(var);

        // create fixed size types since CT uses fixed size types
        typename MANIFOLD_AD::Tangent xFixed = var.template head<tangent_dim>();
        control_vector_ad_t uFixed = var.template tail<CONTROL_DIM>();

        typename MANIFOLD_AD::Tangent dxFixed;

        dynamics_fct_(retract_fct_(xFixed), Time_ad_t(0), uFixed, dxFixed);

        // output vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> dx(tangent_dim);
        dx = dxFixed;

        // store operation sequence in f: var -> dx and stop recording
        CppAD::ADFun<typename SCALAR::value_type> f(var, dx);

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

    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input

    dynamics_fct_t dynamics_fct_;  //!< function handle to system dynamics
    lift_fct_t lift_fct_;
    retract_fct_t retract_fct_;

    CppAD::ADFun<typename SCALAR::value_type> f_;  //!< Auto-Diff function

    SparsityPattern sparsityA_;  //!< sparsity pattern of the state Jacobian
    SparsityPattern sparsityB_;  //!< sparsity pattern of the input Jacobian
};

}  // namespace internal
}  // namespace core
}  // namespace ct

#endif