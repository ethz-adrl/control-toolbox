/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

#include <ct/core/types/AutoDiff.h>
#include <ct/core/internal/autodiff/CGHelpers.h>
#include <ct/core/math/Derivatives.h>
#include <ct/core/math/DerivativesCppadSettings.h>

#include <cppad/cg/model/llvm/llvm.hpp>

namespace ct {
namespace core {


//! Jacobian using Auto-Diff Codegeneration
/*!
 * Uses Auto-Diff code generation to compute the Jacobian \f$ J(x_s) = \frac{df}{dx} |_{x=x_s} \f$ of
 * a regular vector-valued mathematical function \f$ y = f(x) \f$ .
 *
 * x has IN_DIM dimension and y has OUT_DIM dimension. Thus, they can be
 * scalar functions (IN_DIM = 1, OUT_DIM = 1), fixed or variable size
 * (IN_DIM = -1, OUT_DIM = -1) functions.
 *
 * \note In fact, this class is called Jacobian but computes also zero order derivatives
 *
 * @tparam IN_DIM Input dimensionality of the function (use Eigen::Dynamic (-1) for dynamic size)
 * @tparam OUT_DIM Output dimensionailty of the function (use Eigen::Dynamic (-1) for dynamic size)
 */
template <int IN_DIM, int OUT_DIM>
class DerivativesCppadJIT : public Derivatives<IN_DIM, OUT_DIM, double>  // double on purpose!
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using CG_SCALAR = ADCGScalar;         //!< CG_SCALAR  type
    using CG_VALUE_TYPE = ADCGValueType;  //!< autodiff scalar type

    using IN_TYPE_CG = Eigen::Matrix<CG_SCALAR, IN_DIM, 1>;    //!< function input vector type
    using OUT_TYPE_CG = Eigen::Matrix<CG_SCALAR, OUT_DIM, 1>;  //!< function  output vector type

    using IN_TYPE_D = Eigen::Matrix<double, IN_DIM, 1>;         //!< function input vector type double
    using OUT_TYPE_D = Eigen::Matrix<double, OUT_DIM, 1>;       //!< function output vector type
    using JAC_TYPE_D = Eigen::Matrix<double, OUT_DIM, IN_DIM>;  //!< Jacobian type
    using JAC_TYPE_ROW_MAJOR = Eigen::Matrix<double, OUT_DIM, IN_DIM, Eigen::RowMajor>;  //!< Jac. type in row-major
    using HES_TYPE_D = Eigen::Matrix<double, IN_DIM, IN_DIM>;
    using HES_TYPE_ROW_MAJOR = Eigen::Matrix<double, IN_DIM, IN_DIM, Eigen::RowMajor>;


    using FUN_TYPE_CG = std::function<OUT_TYPE_CG(const IN_TYPE_CG&)>;  //!< function type
    using DerivativesBase = Derivatives<IN_DIM, OUT_DIM>;

    /**
     * @brief      Contructs the derivatives for codegeneration using a
     *             FUN_TYPE_CG function
     * @warning    If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the
     *             actual dimensions of x and y have to be passed here.
     *
     * @param      f          The function to be autodiffed
     * @param[in]  inputDim   inputDim input dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     * @param[in]  outputDim  outputDim output dimension, must be specified if
     *                        template parameter IN_DIM is -1 (dynamic)
     */
    DerivativesCppadJIT(FUN_TYPE_CG& f, int inputDim = IN_DIM, int outputDim = OUT_DIM);

    /*!
     * @brief copy constructor
     * @param arg instance to copy
     * @note  It is  important to not only copy the pointer to the dynamic library, but to load the library properly instead.
     */
    DerivativesCppadJIT(const DerivativesCppadJIT& arg);


    //! update the Jacobian with a new function
    /*!
     * \warning If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the actual dimensions of
     * x and y have to be passed here.
     *
     * @param f new function to compute Jacobian of
     * @param inputDim input dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
     * @param outputDim output dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
     */
    void update(FUN_TYPE_CG& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM);

    virtual ~DerivativesCppadJIT() = default;

    //! deep cloning
    DerivativesCppadJIT* clone() const;

    virtual OUT_TYPE_D forwardZero(const Eigen::VectorXd& x);

    virtual JAC_TYPE_D jacobian(const Eigen::VectorXd& x);

    virtual void sparseJacobian(const Eigen::VectorXd& x,
        Eigen::VectorXd& jac,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol);

    virtual Eigen::VectorXd sparseJacobianValues(const Eigen::VectorXd& x);


    virtual HES_TYPE_D hessian(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda);

    virtual void sparseHessian(const Eigen::VectorXd& x,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol);


    virtual Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda);

    //! get Jacobian sparsity pattern
    /*!
     * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern.
     * @return Sparsity pattern of the Jacobian
     */
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getSparsityPatternJacobian();

    /**
     * @brief      get Hessian sparsity pattern
     *
     * @return     Auto-diff automatically detects the sparsity pattern of the Hessian
     */
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getSparsityPatternHessian();


    //! get Jacobian sparsity pattern
    /*!
     * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern
     * in row-column format. Row and columns contain the indeces of all non-zero entries.
     *
     * @param rows row indeces of non-zero entries
     * @param columns column indeces of non-zero entries
     */
    void getSparsityPatternJacobian(Eigen::VectorXi& rows, Eigen::VectorXi& columns);

    /**
     * @brief      Returns the number of nonzeros in the sparse jacobian
     *
     * @return     The number of nonzeros in the sparse jacobian
     */
    size_t getNumNonZerosJacobian();

    /**
     * @brief      Returns the number of nonzeros in the sparse hessian
     *
     * @return     The number of non zeros in the sparse hessian
     */
    size_t getNumNonZerosHessian();

    //! get Hessian sparsity pattern
    /*!
     * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern
     * in row-column format. Row and columns contain the indeces of all non-zero entries.
     *
     * @param rows row indeces of non-zero entries
     * @param columns column indeces of non-zero entries
     */
    void getSparsityPatternHessian(Eigen::VectorXi& rows, Eigen::VectorXi& columns);


    //! Uses just-in-time compilation to compile the Jacobian and other derivatives
    /*!
     *  This method generates source code for the Jacobian and zero order derivative. It then compiles
     *  the source code to a dynamically loadable library that then gets loaded.
     */
    void compileJIT(const DerivativesCppadSettings& settings,
        const std::string& libName = "unnamedLib",
        bool verbose = false);

    //! retrieve the dynamic library, e.g. for testing purposes
    const std::shared_ptr<CppAD::cg::DynamicLib<double>> getDynamicLib();

#ifdef LLVM_VERSION_MAJOR
    //! retrieve the llvm in-memory library, e.g. for testing purposes
    const std::shared_ptr<CppAD::cg::LlvmModelLibrary<double>> getLlvmLib();
#endif

protected:
    //! record the Auto-Diff terms for code generation
    void recordCg();

    std::function<OUT_TYPE_CG(const IN_TYPE_CG&)> cgStdFun_;  //! the function

    int inputDim_;   //! function input dimension
    int outputDim_;  //! function output dimension

    CppAD::ADFun<CG_VALUE_TYPE> cgCppadFun_;  //!  auto-diff function

    bool compiled_;        //! flag if Jacobian is compiled
    std::string libName_;  //! a unique name for this library

    std::vector<size_t> sparsityRowsJacobian_;
    std::vector<size_t> sparsityColsJacobian_;
    std::vector<size_t> sparsityRowsHessian_;
    std::vector<size_t> sparsityColsHessian_;

    Eigen::VectorXi sparsityRowsJacobianEigen_;
    Eigen::VectorXi sparsityColsJacobianEigen_;
    Eigen::VectorXi sparsityRowsHessianEigen_;
    Eigen::VectorXi sparsityColsHessianEigen_;
    //!
    CppAD::cg::GccCompiler<double> compiler_;  //! compile for codegeneration
    CppAD::cg::ClangCompiler<double> compilerClang_;
    std::shared_ptr<CppAD::cg::DynamicLib<double>> dynamicLib_;          //! dynamic library to load after compilation
    std::shared_ptr<CppAD::cg::GenericModel<double>> model_;             //! the model
#ifdef LLVM_VERSION_MAJOR
    std::shared_ptr<CppAD::cg::LlvmModelLibrary<double>> llvmModelLib_;  //! llvm in-memory library
#endif
};


} /* namespace core */
} /* namespace ct */

#endif

#include "DerivativesCppadJIT-impl.h"
