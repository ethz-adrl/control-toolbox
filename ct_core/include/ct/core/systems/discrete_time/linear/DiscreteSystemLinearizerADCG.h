/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#pragma once

#ifdef CPPADCG

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear DiscreteControlledSystem using Automatic Differentiation with code generation
/*!
 * This class takes a non-linear DiscreteControlledSystem \f$ x[n+1] = f(x[n],u[n],n) \f$
 * and computes the linearization around a certain point \f$ x = x_s \f$, \f$ u = u_s \f$.
 *
 * \f[
 *   x[n+1] = A x[n] + B u[n]
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
 * \note This is generally the most efficient and most accurate way to generate the linearization of system dynamics.
 *
 * Unit test \ref AutoDiffLinearizerTest.cpp illustrates the use of this class.
 *
 *
 * \warning You should ensure that your DiscreteControlledSystem is templated on the scalar type and does not contain branching
 * (if/else statements, switch cases etc.)
 *
 * The linearization is computed using Auto Differentiation which is then used by a code generator framework to generate
 * efficient code. For convenience just-in-time compilation is provided. However, you can also generate source code directly.
 *
 * \warning Depending on the complexity of your system, just-in-time compilation (compileJIT()) can be slow. In that case generate a
 * source code file
 *
 * @tparam STATE_DIM dimension of state vector
 * @tparam CONTROL_DIM dimension of control vector
 * @tparam SCALAR primitive type of resultant linear system
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteSystemLinearizerADCG : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

    typedef CppAD::AD<CppAD::cg::CG<SCALAR>> ADCGScalar;  //!< Autodiff codegen type

    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, ADCGScalar> system_t;  //!< type of system to be linearized
    typedef DynamicsLinearizerADCG<STATE_DIM, CONTROL_DIM, ADCGScalar, int>
        linearizer_t;  //!< type of linearizer to be used


    typedef typename Base::state_vector_t state_vector_t;      //!< state vector type
    typedef typename Base::control_vector_t control_vector_t;  //!< control vector type

    typedef typename Base::state_matrix_t state_matrix_t;                  //!< state Jacobian type (A)
    typedef typename Base::state_control_matrix_t state_control_matrix_t;  //! control Jacobian type (B)

    //! default constructor
    /*!
     * @param nonlinearSystem non-linear system instance to linearize
     */
    DiscreteSystemLinearizerADCG(std::shared_ptr<system_t> nonlinearSystem, bool cacheJac = true)
        : Base(nonlinearSystem->getType()),
          dFdx_(state_matrix_t::Zero()),
          dFdu_(state_control_matrix_t::Zero()),
          cacheJac_(cacheJac),
          nonlinearSystem_(nonlinearSystem),
          linearizer_(std::bind(&system_t::propagateControlledDynamics,
              nonlinearSystem_.get(),
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3,
              std::placeholders::_4))
    {
    }

    //! copy constructor
    DiscreteSystemLinearizerADCG(const DiscreteSystemLinearizerADCG& arg)
        : Base(arg.nonlinearSystem_->getType()),
          dFdx_(arg.dFdx_),
          dFdu_(arg.dFdu_),
          cacheJac_(arg.cacheJac_),
          nonlinearSystem_(arg.nonlinearSystem_->clone()),
          linearizer_(arg.linearizer_)
    {
    }

    //! destructor
    virtual ~DiscreteSystemLinearizerADCG() {}
    //! deep cloning
    DiscreteSystemLinearizerADCG<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
    {
        return new DiscreteSystemLinearizerADCG<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
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
    const state_matrix_t& getDerivativeState(const state_vector_t& x, const control_vector_t& u, const int t = 0)
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, t);
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
    const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const int t = 0)
    {
        dFdu_ = linearizer_.getDerivativeControl(x, u, t);
        return dFdu_;
    }

    //! retrieve discrete-time linear system matrices A and B.
    /*!
     * This computes matrices A and B such that
     * \f[
     *  x_{n+1} = Ax_n + Bu_n
     * \f]
     *
     * Note that the inputs x_next and subSteps are being ignored
     *
     * @param x the state setpoint at n
     * @param u the control setpoint at n
     * @param n the time setpoint
     * @param x_next -> ignored
     * @param subSteps -> ignored
     * @param A the resulting linear system matrix A
     * @param B the resulting linear system matrix B
     */
    void getAandB(const state_vector_t& x,
        const control_vector_t& u,
        const state_vector_t& x_next,
        const int n,
        size_t numSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) override
    {
        dFdx_ = linearizer_.getDerivativeState(x, u, n);
        dFdu_ = linearizer_.getDerivativeControl(x, u, n);

        A = dFdx_;
        B = dFdu_;
    }

    //! compile just-in-time
    /*!
     * Generates the source code, compiles it and dynamically loads the resulting library.
     *
     * \note If this function takes a long time, consider generating the source code using
     * generateCode() and compile it before runtime.
     */
    void compileJIT(const std::string& libName = "DiscreteSystemLinearizerADCG") { linearizer_.compileJIT(libName); }
    //! generates source code and saves it to file
    /*!
     * This generates source code for computing the system linearization and saves it to file. This
     * function uses a template file in which it replaces two placeholders, each identified as the
     * string "AUTOGENERATED_CODE_PLACEHOLDER"
     *
     * @param systemName name of the resulting LinearSystem class
     * @param outputDir output directory
     * @param templateDir directory of the template file
     * @param ns1 first layer namespace
     * @param ns2 second layer namespace
     * @param useReverse if true, uses Auto-Diff reverse mode
     * @param ignoreZero if true, zero entries are not assigned zero
     */
    void generateCode(const std::string& systemName,
        const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
        const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
        const std::string& ns1 = "core",
        const std::string& ns2 = "generated",
        bool useReverse = false,
        bool ignoreZero = true)
    {
        std::string codeJacA, codeJacB;
        linearizer_.generateCode(codeJacA, codeJacB, useReverse, ignoreZero);

        writeCodeFile(
            templateDir, outputDir, systemName, ns1, ns2, codeJacA, codeJacB, "AUTOGENERATED_CODE_PLACEHOLDER");
    }

    //! accessor to the linearizer, e.g. for testing
    const linearizer_t& getLinearizer() const { return linearizer_; }
private:
    //! write code to file
    /*!
     * Writes generated code to file
     * @param templateDir directory of the template file
     * @param outputDir output directory
     * @param systemName name of the resulting system class
     * @param ns1 first layer namespace
     * @param ns2 second layer namespace
     * @param codeJacA code for state Jacobian A
     * @param codeJacB code for input Jacobian B
     * @param codePlaceholder placeholder to search for and to be replaced with code
     */
    void writeCodeFile(const std::string& templateDir,
        const std::string& outputDir,
        const std::string& systemName,
        const std::string& ns1,
        const std::string& ns2,
        const std::string& codeJacA,
        const std::string& codeJacB,
        const std::string& codePlaceholder)
    {
        std::cout << "Generating discrete linear system..." << std::endl;

        size_t maxTempVarCountState, maxTempVarCountControl;
        linearizer_.getMaxTempVarCount(maxTempVarCountState, maxTempVarCountControl);

        std::string header = internal::CGHelpers::parseFile(templateDir + "/DiscreteLinearSystem.tpl.h");
        std::string sourceA = internal::CGHelpers::parseFile(templateDir + "/DiscreteLinearSystem.tplA.cpp");
        std::string sourceB = internal::CGHelpers::parseFile(templateDir + "/DiscreteLinearSystem.tplB.cpp");

        const std::string scalarName(linearizer_.getOutScalarType());

        replaceSizesAndNames(header, systemName, scalarName, ns1, ns2);
        replaceSizesAndNames(sourceA, systemName, scalarName, ns1, ns2);
        replaceSizesAndNames(sourceB, systemName, scalarName, ns1, ns2);

        internal::CGHelpers::replaceOnce(header, "MAX_COUNT_STATE", std::to_string(maxTempVarCountState));
        internal::CGHelpers::replaceOnce(header, "MAX_COUNT_CONTROL", std::to_string(maxTempVarCountControl));

        internal::CGHelpers::replaceOnce(sourceA, codePlaceholder + "_JAC_A", codeJacA);
        internal::CGHelpers::replaceOnce(sourceB, codePlaceholder + "_JAC_B", codeJacB);

        internal::CGHelpers::writeFile(outputDir + "/" + systemName + ".h", header);
        internal::CGHelpers::writeFile(outputDir + "/" + systemName + "_A.cpp", sourceA);
        internal::CGHelpers::writeFile(outputDir + "/" + systemName + "_B.cpp", sourceB);


        std::cout << "... Done! Successfully generated discrete linear system" << std::endl;
    }

    //! replaces size and namespace placeholders in file
    /*!
     * @param file content of the file to perform the modification on
     * @param systemName name of the system
     * @param scalarName name of scalar (e.g., "double")
     * @param ns1 first layer namespace
     * @param ns2 second layer namespace
     */
    void replaceSizesAndNames(std::string& file,
        const std::string& systemName,
        const std::string& scalarName,
        const std::string& ns1,
        const std::string& ns2)
    {
        internal::CGHelpers::replaceAll(file, "LINEAR_SYSTEM_NAME", systemName);
        internal::CGHelpers::replaceAll(file, "NS1", ns1);
        internal::CGHelpers::replaceAll(file, "NS2", ns2);
        internal::CGHelpers::replaceAll(file, "STATE_DIM", std::to_string(STATE_DIM));
        internal::CGHelpers::replaceAll(file, "CONTROL_DIM", std::to_string(CONTROL_DIM));
        internal::CGHelpers::replaceAll(file, "SCALAR", scalarName);
    }

protected:
    state_matrix_t dFdx_;          //!< state Jacobian
    state_control_matrix_t dFdu_;  //!< input Jacobian

    bool cacheJac_;  //!< flag if Jacobian will be cached

    std::shared_ptr<system_t> nonlinearSystem_;  //!< instance of non-linear system

    linearizer_t linearizer_;  //!< instance of ad-linearizer
};

}  // namespace core
}  // namespace ct

#endif