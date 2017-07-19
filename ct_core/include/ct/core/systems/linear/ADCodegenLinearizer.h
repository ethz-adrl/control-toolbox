/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef INCLUDE_CT_CORE_SYSTEMS_LINEAR_ADCODEGENLINEARIZER_H_
#define INCLUDE_CT_CORE_SYSTEMS_LINEAR_ADCODEGENLINEARIZER_H_

#include "internal/ADLinearizerBase.h"
#include <ct/core/internal/autodiff/CGHelpers.h> 

#include <ct/core/templateDir.h>

namespace ct {
namespace core {

//! Computes the linearization of a general non-linear ControlledSystem using Automatic Differentiation with code generation
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
 * \note This is generally the most efficient and most accurate way to generate the linearization of system dynamics.
 *
 * \warning You should ensure that your ControlledSystem is templated on the scalar type and does not contain branching
 * (if/else statements, switch cases etc.)
 *
 * The linearization is computed using Auto Differentiation which is then used by a code generator framework to generate
 * efficient code. For convenience just-in-time compilation is provided. However, you can also generate source code directly.
 *
 * Unit test \ref CodegenTests.cpp illustrates the use of the ADCodeGenLinearizer.
 *
 * \warning Depending on the complexity of your system, just-in-time compilation (compileJIT()) can be slow. In that case generate a
 * source code file
 *
 * @tparam dimension of state vector
 * @tparam dimension of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class ADCodegenLinearizer : public internal::ADLinearizerBase<STATE_DIM, CONTROL_DIM, CppAD::AD<CppAD::cg::CG<double> > >
{

public:

	typedef ADCGScalar SCALAR; //!< scalar type
	typedef ADCGValueType AD_SCALAR; //!< Auto-Diff scalar type
	typedef internal::ADLinearizerBase<STATE_DIM, CONTROL_DIM, SCALAR> Base; //!< base class type

	typedef typename Base::state_vector_t state_vector_t; //!< state vector type
	typedef typename Base::control_vector_t control_vector_t; //!< control vector type
	typedef typename Base::state_matrix_t state_matrix_t; //!< state matrix type
	typedef typename Base::state_control_matrix_t state_control_matrix_t; //!< state control matrix type

	//! default constructor
	/*!
	 * Initializes an Auto-Diff codegen linearizer with a ControlledSystem
	 * @param nonlinearSystem non-linear system instance
	 * @param cacheJac if true, caches the Jacobians to prevent recomputation for the same state/input
	 */
	ADCodegenLinearizer(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> > nonlinearSystem, bool cacheJac = true) :
		Base(nonlinearSystem),
		dFdx_(state_matrix_t::Zero()),
		dFdu_(state_control_matrix_t::Zero()),
		compiled_(false),
		cacheJac_(cacheJac),
		x_at_cache_(state_vector_t::Random()),
		u_at_cache_(control_vector_t::Random()),
		maxTempVarCountState_(0),
		maxTempVarCountControl_(0)
	{}

	/**
	 * @brief      Copy constructor
	 *
	 * @param[in]  arg   The argument
	 */
	ADCodegenLinearizer(const ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>& arg) :
		Base(arg),
		dFdx_(arg.dFdx_),
		dFdu_(arg.dFdu_),
		compiled_(arg.compiled_),
		cacheJac_(arg.cacheJac_),
		x_at_cache_(arg.x_at_cache_),
		u_at_cache_(arg.u_at_cache_),
		dynamicLib_(arg.dynamicLib_),
		maxTempVarCountState_(arg.maxTempVarCountState_),
		maxTempVarCountControl_(arg.maxTempVarCountControl_)
	{
		if(compiled_)
			model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model("ADCodegenLinearizer"));
	}

	//! deep cloning
	ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>* clone() const override {
		return new ADCodegenLinearizer<STATE_DIM, CONTROL_DIM>(*this);
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
	 * \warning Call compileJIT() before calling this function.
	 *
	 * @param x state to linearize at
	 * @param u control to linearize at
	 * @param t time
	 * @return Jacobian wrt state
	 */
	const state_matrix_t& getDerivativeState(const state_vector_t& x, const control_vector_t& u, const double t = 0.0) override
	{
		if(!compiled_)
			throw std::runtime_error("Called getDerivativeState on ADCodegenLinearizer before compiling. Call 'compile()' before");

		// if jacobian is not supposed to be cached or if values change, recompute it
		if (!cacheJac_ || (x != x_at_cache_ || u != u_at_cache_))
			computeJacobian(x, u);

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
	 * \warning Call compileJIT() before calling this function.
	 *
	 * @param x state to linearize at
	 * @param u control to linearize at
	 * @param t time
	 * @return Jacobian wrt input
	 */
	const state_control_matrix_t& getDerivativeControl(const state_vector_t& x, const control_vector_t& u, const double t = 0.0) override
	{
		if(!compiled_)
		{
			throw std::runtime_error("Called getDerivativeState on ADCodegenLinearizer before compiling. Call 'compile()' before");
		}

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
	void compileJIT(const std::string& libName = "threadId" + std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) )
	{
		if (compiled_) return;

	    CppAD::cg::ModelCSourceGen<double> cgen(this->f_, "ADCodegenLinearizer");
	    cgen.setCreateJacobian(true);
	    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

	    // compile source code
	    CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen, libName);

	    dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler_));

	    model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model("ADCodegenLinearizer"));

	    compiled_ = true;
	}

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
	void generateCode(
			const std::string& systemName,
			const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
			const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
			const std::string& ns1 = "core",
			const std::string& ns2 = "generated",
			bool useReverse = false,
			bool ignoreZero = true
			)
	{		
		this->sparsityA_.clearWork(); //clears the cppad sparsity work called by a possible method call before
		size_t jacDimension = STATE_DIM * STATE_DIM;
		std::string codeJacA =
				internal::CGHelpers::generateJacobianSource(
						this->f_,
						this->sparsityA_,
						jacDimension,
						maxTempVarCountState_,
						useReverse,
						ignoreZero,
						"jac", 
						"x_in",
						"vX_");

		this->sparsityB_.clearWork(); //clears the cppad sparsity work called by a possible method call before
		jacDimension = STATE_DIM * CONTROL_DIM;
		std::string codeJacB =
				internal::CGHelpers::generateJacobianSource(
						this->f_,
						this->sparsityB_,
						jacDimension,
						maxTempVarCountControl_,
						useReverse,
						ignoreZero,
						"jac",
						"x_in",
						"vU_");

		writeCodeFile(templateDir, outputDir, systemName, ns1, ns2, codeJacA, codeJacB, "AUTOGENERATED_CODE_PLACEHOLDER");
	}


private:
	//! computes the Jacobians
	/*!
	 * Given a state and input this method evaluates both Jacobians and caches them
	 * @param x state to linearize around
	 * @param u input to linearize around
	 */
	void computeJacobian(const state_vector_t& x, const control_vector_t& u)
	{
		Eigen::Matrix<double, Eigen::Dynamic, 1> input(STATE_DIM+CONTROL_DIM);
		input << x, u;

		Eigen::Matrix<double, Eigen::Dynamic, 1> jac(Base::FullJac_entries);

		jac = model_->Jacobian(input);

		Eigen::Map<Eigen::Matrix<double, STATE_DIM+CONTROL_DIM, STATE_DIM>> out(jac.data());

		dFdx_ = out.template topRows<STATE_DIM>().transpose();
		dFdu_ = out.template bottomRows<CONTROL_DIM>().transpose();

		x_at_cache_ = x;
		u_at_cache_ = u;
	}

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
	void writeCodeFile(
			const std::string& templateDir,
			const std::string& outputDir,
			const std::string& systemName,
			const std::string& ns1,
			const std::string& ns2,
			const std::string& codeJacA,
			const std::string& codeJacB,
			const std::string& codePlaceholder)
	{
		std::cout << "Generating linear system..."<<std::endl;

		std::string header = internal::CGHelpers::parseFile(templateDir+"/LinearSystem.tpl.h");
		std::string source = internal::CGHelpers::parseFile(templateDir+"/LinearSystem.tpl.cpp");

		replaceSizesAndNames(header, systemName, ns1, ns2);
		replaceSizesAndNames(source, systemName, ns1, ns2);

		internal::CGHelpers::replaceOnce(header, "MAX_COUNT_STATE", std::to_string(maxTempVarCountState_));
		internal::CGHelpers::replaceOnce(header, "MAX_COUNT_CONTROL", std::to_string(maxTempVarCountControl_));

		internal::CGHelpers::replaceOnce(source, codePlaceholder+"_JAC_A", codeJacA);
		internal::CGHelpers::replaceOnce(source, codePlaceholder+"_JAC_B", codeJacB);

		internal::CGHelpers::writeFile(outputDir+"/"+systemName+".h", header);
		internal::CGHelpers::writeFile(outputDir+"/"+systemName+".cpp", source);


		std::cout << "... Done! Successfully generated linear system"<<std::endl;
	}

	//! replaces size and namespace placeholders in file
	/*!
	 * @param file content of the file to perform the modification on
	 * @param systemName name of the system
	 * @param ns1 first layer namespace
	 * @param ns2 second layer namespace
	 */
	void replaceSizesAndNames(std::string& file, const std::string& systemName, const std::string& ns1, const std::string& ns2)
	{
		internal::CGHelpers::replaceAll(file, "LINEAR_SYSTEM_NAME", systemName);
		internal::CGHelpers::replaceAll(file, "NS1", ns1);
		internal::CGHelpers::replaceAll(file, "NS2", ns2);
		internal::CGHelpers::replaceAll(file, "STATE_DIM", std::to_string(STATE_DIM));
		internal::CGHelpers::replaceAll(file, "CONTROL_DIM", std::to_string(CONTROL_DIM));
	}

	state_matrix_t dFdx_; //!< state Jacobian
	state_control_matrix_t dFdu_; //!< input Jacobian

	bool compiled_; //!< flag if library is compiled
	bool cacheJac_; //!< flag if Jacobian will be cached
	CppAD::cg::GccCompiler<double> compiler_; //!< compiler instance for JIT compilation

	state_vector_t x_at_cache_; //!< state at which Jacobian has been cached
	control_vector_t u_at_cache_; //!< input at which Jacobian has been cached


	std::shared_ptr<CppAD::cg::DynamicLib<double>> dynamicLib_; //!< compiled and dynamically loaded library
	std::shared_ptr<CppAD::cg::GenericModel<double>> model_; //!< Auto-Diff model

	size_t maxTempVarCountState_; //!< number of temporary variables in the source code of the state Jacobian
	size_t maxTempVarCountControl_; //!< number of temporary variables in the source code of the input Jacobian
};


}
}


#endif /* INCLUDE_CT_CORE_SYSTEMS_LINEAR_ADCODEGENLINEARIZER_H_ */
