/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#pragma once

#ifdef CPPADCG
#ifdef CPPAD

#include "DynamicsLinearizerADBase.h"
#include <ct/core/internal/autodiff/CGHelpers.h>

namespace ct {
namespace core {

//! Computes the linearization of a system dynamics function through autodiff with code generation
/*!
 * This class takes a function handle representing system dynamics of the form
 * \f$ f(x(t),t,u(t),\dot{x(t)}) \f$ or \f$ f(x[n],n,u[n],x[n+1]) \f$ where the
 * last argument is the result of the evaluation in each case. It then computes
 * the linearization around a given point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \tparam STATE_DIM dimension of state vector
 * \tparam CONTROL_DIM dimension of control vector
 * \tparam SCALAR scalar type
 * \tparam TIME type of time variable of dynamics
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR, typename TIME>
class DynamicsLinearizerADCG : public internal::DynamicsLinearizerADBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef internal::DynamicsLinearizerADBase<STATE_DIM, CONTROL_DIM, SCALAR, TIME> Base;

    typedef typename Base::OUT_SCALAR OUT_SCALAR;  //!< scalar type of resulting linear system

    typedef typename Base::state_vector_t state_vector_t;      //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;  //!< control vector type

    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type (A)
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //!< control Jacobian type (B)

    typedef typename Base::dynamics_fct_t dynamics_fct_t;  //!< dynamics function signature

    //! default constructor
    /*!
     * Initializes the DynamicsLinearizerADCG with the system dynamics
     * @param dyn function handle to system dynamics
     * @param cacheJac if true, caches the Jacobians to prevent recomputation for same state/control
     */
    DynamicsLinearizerADCG(dynamics_fct_t dyn, bool cacheJac = true)
        : Base(dyn),
          dynamics_fct_(dyn),
          dFdx_(state_matrix_t::Zero()),
          dFdu_(state_control_matrix_t::Zero()),
          x_at_cache_(state_vector_t::Random()),
          u_at_cache_(control_vector_t::Random()),
          jitLibName_(""),
          compiled_(false),
          cacheJac_(cacheJac),
          maxTempVarCountState_(0),
          maxTempVarCountControl_(0)
    {
    }

    //! copy constructor
    DynamicsLinearizerADCG(const DynamicsLinearizerADCG& rhs)
        : Base(rhs.dynamics_fct_),
          dynamics_fct_(rhs.dynamics_fct_),
          dFdx_(rhs.dFdx_),
          dFdu_(rhs.dFdu_),
          x_at_cache_(rhs.x_at_cache_),
          u_at_cache_(rhs.u_at_cache_),
          jitLibName_(rhs.jitLibName_),
          compiled_(rhs.compiled_),
          cacheJac_(rhs.cacheJac_),
          maxTempVarCountState_(rhs.maxTempVarCountState_),
          maxTempVarCountControl_(rhs.maxTempVarCountControl_)
    {
        if (compiled_)
        {
            dynamicLib_ = internal::CGHelpers::loadDynamicLibCppad<OUT_SCALAR>(jitLibName_);
            model_ = std::shared_ptr<CppAD::cg::GenericModel<OUT_SCALAR>>(
                dynamicLib_->model("DynamicsLinearizerADCG" + jitLibName_));
        }
    }

    //! compute and return derivative w.r.t. state
    /*!
     * \warning Call compileJIT() before calling this function
     *
     * @param x state to linearize at
     * @param u control to linearize at
     * @param t time
     * @return Jacobian w.r.t. state
     */
    const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const OUT_SCALAR t = 0.0)
    {
        if (!compiled_)
            throw std::runtime_error(
                "Called getDerivativeState on ADCodegenLinearizer before compiling. Call 'compile()' before");

        // if jacobian is not supposed to be cached or if values change, recompute it
        if (!cacheJac_ || (x != x_at_cache_ || u != u_at_cache_))
            computeJacobian(x, u);

        return dFdx_;
    }

    //! compute and return derivative w.r.t. control
    /*!
     * \warning Call compileJIT() before calling this function
     *
     * @param x state to linearize at
     * @param u control to linearize at
     * @param t time
     * @return Jacobian w.r.t. control
     */
    const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const OUT_SCALAR t = 0.0)
    {
        if (!compiled_)
            throw std::runtime_error(
                "Called getDerivativeState on ADCodegenLinearizer before compiling. Call 'compile()' before");

        // if jacobian is not supposed to be cached or if values change, recompute it
        if (!cacheJac_ || (x != x_at_cache_ || u != u_at_cache_))
            computeJacobian(x, u);

        return dFdu_;
    }

    //! compile just-in-time
    /*!
    * Generates the source code, compiles it and dynamically loads the resulting library.
    *
    * \note If this function takes a long time, consider generating the source code using
    * generateCode() and compile it before runtime.
    */
    void compileJIT(const std::string& libName = "DynamicsLinearizerADCG", bool verbose = false)
    {
        if (compiled_)
            return;

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        // assigning a unique identifier to the library in order to avoid race conditions in JIT
        std::string uniqueID =
            std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) + "_" + std::to_string(ts.tv_nsec);

        jitLibName_ = libName + uniqueID;

        CppAD::cg::ModelCSourceGen<OUT_SCALAR> cgen(this->f_, "DynamicsLinearizerADCG" + jitLibName_);
        cgen.setCreateJacobian(true);
        CppAD::cg::ModelLibraryCSourceGen<OUT_SCALAR> libcgen(cgen);
        std::string tempDir = "cppad_temp" + uniqueID;
        if (verbose)
        {
            std::cout << "Starting to compile " << jitLibName_ << " library ..." << std::endl;
            std::cout << "In temporary directory " << tempDir << std::endl;
        }

        // compile source code
        CppAD::cg::DynamicModelLibraryProcessor<OUT_SCALAR> p(libcgen, jitLibName_);
        compiler_.setTemporaryFolder(tempDir);
        dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<OUT_SCALAR>>(p.createDynamicLibrary(compiler_));

        model_ = std::shared_ptr<CppAD::cg::GenericModel<OUT_SCALAR>>(
            dynamicLib_->model("DynamicsLinearizerADCG" + jitLibName_));

        compiled_ = true;

        if (verbose)
            std::cout << "Successfully compiled " << jitLibName_ << std::endl;
    }

    //! generates source code
    /*!
     * This generates source code for computing the system linearization.
     * The generated string should be passed to a function which writes the
     * generated code to a file.
     *
     * This function uses a template file in which it replaces two placeholders,
     * each identified as the string "AUTOGENERATED_CODE_PLACEHOLDER"
     *
     * @param[out] codeJacA string with the generated code for the A matrix
     * @param[out] codeJacB string with the generated code for the B matrix
     * @param[in]  useReverse if true, uses Auto-Diff reverse mode
     * @param[in]  ignoreZero if true, zero entries are not assigned zero
     */
    void generateCode(std::string& codeJacA, std::string& codeJacB, bool useReverse = false, bool ignoreZero = true)
    {
        this->sparsityA_.clearWork();  //clears the cppad sparsity work possibly done before
        size_t jacDimension = STATE_DIM * STATE_DIM;
        codeJacA = internal::CGHelpers::generateJacobianSource<typename SCALAR::value_type, OUT_SCALAR>(this->f_,
            this->sparsityA_, jacDimension, maxTempVarCountState_, useReverse, ignoreZero, "jac", "x_in", "vX_",
            Base::getOutScalarType());

        this->sparsityB_.clearWork();
        jacDimension = STATE_DIM * CONTROL_DIM;
        codeJacB = internal::CGHelpers::generateJacobianSource<typename SCALAR::value_type, OUT_SCALAR>(this->f_,
            this->sparsityB_, jacDimension, maxTempVarCountControl_, useReverse, ignoreZero, "jac", "x_in", "vU_",
            Base::getOutScalarType());
    }

    //! accessor to maxTempVarCount variables
    void getMaxTempVarCount(size_t& maxTempVarCountState, size_t& maxTempVarCountControl) const
    {
        maxTempVarCountState = maxTempVarCountState_;
        maxTempVarCountControl = maxTempVarCountControl_;
    }

    //! retrieve the dynamic library, e.g. for testing purposes
    const std::shared_ptr<CppAD::cg::DynamicLib<OUT_SCALAR>> getDynamicLib() const { return dynamicLib_; }
protected:
    //! computes the Jacobians
    /*!
     * Given a state and input this method evaluates both Jacobians and caches them
     * @param x state to linearize around
     * @param u input to linearize around
     */
    void computeJacobian(const state_vector_t& x, const control_vector_t& u)
    {
        // copy to dynamic type due to requirements by cppad
        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> input(STATE_DIM + CONTROL_DIM);
        input << x, u;

        Eigen::Matrix<OUT_SCALAR, Eigen::Dynamic, 1> jac(Base::FullJac_entries);

        jac = model_->Jacobian(input);

        Eigen::Map<Eigen::Matrix<OUT_SCALAR, STATE_DIM + CONTROL_DIM, STATE_DIM>> out(jac.data());

        dFdx_ = out.template topRows<STATE_DIM>().transpose();
        dFdu_ = out.template bottomRows<CONTROL_DIM>().transpose();

        x_at_cache_ = x;
        u_at_cache_ = u;
    }


    dynamics_fct_t dynamics_fct_;  //!< function handle to system dynamics

    state_matrix_t dFdx_;          //!< Jacobian wrt state
    state_control_matrix_t dFdu_;  //!< Jacobian wrt input

    state_vector_t x_at_cache_;    //!< state at which Jacobian has been cached
    control_vector_t u_at_cache_;  //!< input at which Jacobian has been cached

    std::string jitLibName_;                       //!< name of the library compiled with JIT
    bool compiled_;                                //!< flag if library from generated code is compiled
    bool cacheJac_;                                //!< flag if Jacobian will be cached
    CppAD::cg::GccCompiler<OUT_SCALAR> compiler_;  //!< compiler instance for JIT compilation

    std::shared_ptr<CppAD::cg::DynamicLib<OUT_SCALAR>> dynamicLib_;  //!< compiled and dynamically loaded library
    std::shared_ptr<CppAD::cg::GenericModel<OUT_SCALAR>> model_;     //!< Auto-Diff model

    size_t maxTempVarCountState_;    //!< number of temporary variables in the source code of the state Jacobian
    size_t maxTempVarCountControl_;  //!< number of temporary variables in the source code of the input Jacobian
};

}  // namespace core
}  // namespace ct

#endif
#endif