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

#ifndef INCLUDE_CT_CORE_FUNCTION_JACOBIANCG_H_
#define INCLUDE_CT_CORE_FUNCTION_JACOBIANCG_H_

#include <ct/core/templateDir.h>

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
class JacobianCG : public Jacobian<IN_DIM, OUT_DIM, double> // double on purpose!
{
public:
	typedef Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double> >, IN_DIM, 1> IN_TYPE; //!< function input vector type
	typedef Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double> >, OUT_DIM, 1> OUT_TYPE; //!< function  output vector type

	typedef ADCGScalar SCALAR; //!< scalar  type
	typedef ADCGValueType AD_SCALAR; //!< autodiff scalar type

	typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> JAC_TYPE; //!< Jacobian type
	typedef Eigen::Matrix<double, OUT_DIM, IN_DIM, Eigen::RowMajor> JAC_TYPE_ROW_MAJOR; //!< Jocobian type in row-major format
	typedef Eigen::Matrix<bool, IN_DIM, OUT_DIM> Sparsity; //!< Sparsity pattern type
	typedef Eigen::Matrix<double, IN_DIM, 1> IN_TYPE_D; //!< function input vector type double
	typedef Eigen::Matrix<double, OUT_DIM, 1> OUT_TYPE_D; //!< function output vector type

	typedef std::function<OUT_TYPE(const IN_TYPE&)> Function; //!< function type

	//! default constructor
	/*!
	 * Constructs a Jacobian from a function, given the dimensions
	 *
	 *  \warning If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the actual dimensions of
	 * x and y have to be passed here.
	 *
	 * @param f function to compute Jacobian of
	 * @param inputDim input dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
	 * @param outputDim output dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
	 */
	JacobianCG(Function& f, int inputDim = IN_DIM, int outputDim = OUT_DIM) :
		f_(f),
		compiled_(false),
		inputDim_(inputDim),
		outputDim_(outputDim)
	{
		if(outputDim_ > 0 && inputDim_ > 0)
			record();
	}

	//! copy constructor
	JacobianCG(const JacobianCG& arg) 
	:
		f_(arg.f_),
		compiled_(arg.compiled_),
		inputDim_(arg.inputDim_),
		outputDim_(arg.outputDim_),
		tmpVarCount_(0)
	{}

	//! destructor
	virtual ~JacobianCG()
	{
	}

	//! update the Jacobian with a new function
	/*!
	 * \warning If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the actual dimensions of
	 * x and y have to be passed here.
	 *
	 * @param f new function to compute Jacobian of
	 * @param inputDim input dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
	 * @param outputDim output dimension, must be specified if template parameter IN_DIM is -1 (dynamic)
	 */
	void update(Function& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM)
	{
		f_ = f;
		outputDim_ = outputDim;
		inputDim_ = inputDim;
		record();
		compiled_ = false;
	}

	//! deep cloning of Jacobian
	JacobianCG* clone() const override { throw std::runtime_error("not implemented"); }

	//! computes zero order derivatives, i.e. evaluates the function
	/*!
	 * Evaluates the function (not Jacobian) using codegen
	 *
	 * \warning Call compileJIT() before calling this function.
	 *
	 * @param x parameter to evaluate the function at
	 * @return value of the function
	 */
	OUT_TYPE_D forwardZero(const Eigen::VectorXd& x) {
		assert(model_->isForwardZeroAvailable() == true);

		if(!compiled_)
			throw std::runtime_error("Called computeJacobian on JacobianCG before compiling. Call 'compile()' before");

		return model_->ForwardZero(x);
	}

	//! evaluates the Jacobian at a given state
	/*!
	 * \warning Call compileJIT() before calling this function.
	 * @param x value at which to evaluate the Jacobian at
	 * @return Jacobian evaluated at x
	 */
	JAC_TYPE operator()(const Eigen::VectorXd& x) override {
		assert(model_->isJacobianAvailable() == true);
		if(!compiled_)
			throw std::runtime_error("Called computeJacobian on JacobianCG before compiling. Call 'compile()' before");

		if(outputDim_ <= 0)
			throw std::runtime_error("Outdim dim smaller 0; Define output dim in JacobianCG constructor");
		
		Eigen::VectorXd jac = model_->Jacobian(x);
		Eigen::MatrixXd out(outputDim_, x.rows());
		out = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Map(jac.data(), outputDim_, x.rows());
		return out;
	}

	//! get Jacobian sparsity pattern
	/*!
	 * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern.
	 * @return Sparsity pattern of the Jacobian
	 */
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getSparsityPattern()
	{
		assert(model_->isJacobianSparsityAvailable() == true);
		
		std::vector<bool> sparsityVec = model_->JacobianSparsityBool();
		Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> sparsityMat(outputDim_, inputDim_);

		std::cout << "outputDim_: " << outputDim_ << std::endl;
		std::cout << "inputDim_: " << inputDim_ << std::endl;
		std::cout << "sparistyvec size: " << sparsityVec.size() << std::endl;



		// assert(sparsityVec.size() == outputDim_ * inputDim_);
		for(size_t row = 0; row < outputDim_; ++row)
			for(size_t col = 0; col < inputDim_; ++col)
			{
				std::cout << "row: " << row << std::endl;
				std::cout << "col: " << col << std::endl;
				std::cout << "ele: " << sparsityVec[col + row * inputDim_] << std::endl;
				sparsityMat(row, col) = sparsityVec[col + row * inputDim_];
			}

		return sparsityMat;
	}


	//! get Jacobian sparsity pattern
	/*!
	 * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern
	 * in row-column format. Row and columns contain the indeces of all non-zero entries.
	 *
	 * @param rows row indeces of non-zero entries
	 * @param columns column indeces of non-zero entries
	 */
	void getSparsityPattern(Eigen::VectorXi& rows, Eigen::VectorXi& columns)
	{
		assert(model_->isJacobianSparsityAvailable() == true);
		std::vector<size_t> rowsVec;
		std::vector<size_t> colsVec;
		Eigen::Matrix<size_t, Eigen::Dynamic, 1> rowsTmp;
		Eigen::Matrix<size_t, Eigen::Dynamic, 1> colsTmp;
		model_->JacobianSparsity(rowsVec, colsVec);
		rows.resize(rowsVec.size());
		columns.resize(colsVec.size());
		rowsTmp = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(rowsVec.data(), rowsVec.size(), 1);
		colsTmp = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(colsVec.data(), colsVec.size(), 1);
		rows = rowsTmp.cast<int>();
		columns = colsTmp.cast<int>();
	}

	//! Uses just-in-time compilation to compile the Jacobian and other derivatives
	/*!
	 *  This method generates source code for the Jacobian and zero order derivative. It then compiles
	 *  the source code to a dynamically loadable library that then gets loaded.
	 */
	void compileJIT(const std::string& libName = "jacCGLib")
	{
		if (compiled_) return;

		CppAD::cg::ModelCSourceGen<double> cgen(this->fAD_, "JacobianCG");
		cgen.setCreateForwardZero(true);
		cgen.setCreateJacobian(true);
		cgen.setCreateSparseJacobian(true);
		CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

		// compile source code
		CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen, libName);

		dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler_));

		model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model("JacobianCG"));

		compiled_ = true;
	}


	//! Generates code for computing the Jacobian and writes it to file
	/*!
	 * This function is optional and can be used if you want to generate code for a Jacobian
	 * and write it to file. This can be useful to pre-compile Jacobians rather than compiling
	 * them at runtime using compileJIT(). This function can make use of a template file in which
	 * the keyword "AUTOGENERATED_CODE_PLACEHOLDER" is replaced with the autogenerated code.
	 *
	 * @param jacobianName name of the resulting Jacobian class
	 * @param outputDir output directory
	 * @param templateDir directory in which template file is located
	 * @param ns1 first namespace layer of the Jacobian class
	 * @param ns2 second namespace layer of the Jacobian class
	 * @param sparsity sparsity pattern to generate sparse Jacobian
	 * @param useReverse if true, uses Auto-Diff reverse mode, otherwise uses forward mode
	 * @param ignoreZero do not assign 0 to sparse entries or zero entries
	 */
	void generateCode(
			const std::string& jacobianName,
			const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
			const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
			const std::string& ns1 = "core",
			const std::string& ns2 = "generated",
			const Sparsity& sparsity = Sparsity::Ones(),
			bool useReverse = true,
			bool ignoreZero = true)
	{
		internal::SparsityPattern pattern;
		pattern.initPattern(sparsity);

		CppAD::vector<AD_SCALAR> jac(IN_DIM*OUT_DIM);
		std::string codeJac =
				internal::CGHelpers::generateJacobianCode(
						this->fAD_,
						inputDim_,
						jac,
						pattern,
						tmpVarCount_,
						useReverse,
						ignoreZero
				);

		writeCodeFile(templateDir, outputDir, jacobianName, ns1, ns2, codeJac, "AUTOGENERATED_CODE_PLACEHOLDER");
	}

	//! Generates code for computing the zero-order derivative and writes it to file
	/*!
	 * This function is optional and can be used if you want to generate code for a zero-order derivative,
	 * i.e. the function itself and write it to file. While it seems weird at first to regenerate code from
	 * existing code, this can help to speed up complex computations. Generating source code can be useful
	 * to pre-compile the zero order dynamics rather than compiling them at runtime using compileJIT().
	 * This function can make use of a template file in which the keyword "AUTOGENERATED_CODE_PLACEHOLDER"
	 * is replaced with the autogenerated code.
	 *
	 * @param forwardZeroName name of the resulting class
	 * @param outputDir output directory
	 * @param templateDir directory in which template file is located
	 * @param ns1 first namespace layer of the class
	 * @param ns2 second namespace layer of the class
	 * @param ignoreZero do not assign 0 to sparse entries or zero entries
	 */
	void generateForwardZeroCode(
			const std::string& forwardZeroName,
			const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
			const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
			const std::string& ns1 = "core",
			const std::string& ns2 = "generated",
			bool ignoreZero = true)
	{
		CppAD::vector<AD_SCALAR> forwardZero(OUT_DIM);
		std::string codeJac =
				internal::CGHelpers::generateForwardZeroCode(
						this->fAD_,
						inputDim_,
						forwardZero,
						tmpVarCount_,
						ignoreZero
				);

		writeCodeFile(templateDir, outputDir, forwardZeroName, ns1, ns2, codeJac, "AUTOGENERATED_CODE_PLACEHOLDER");
	}



private:

	//! record the Auto-Diff terms
	void record()
	{
		// input vector, needs to be dynamic size
		Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> x(inputDim_);

		// declare x as independent
		CppAD::Independent(x);

		// output vector, needs to be dynamic size
		Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> y(outputDim_);

		y = f_(x);

		// store operation sequence in f: x -> y and stop recording
		CppAD::ADFun<typename SCALAR::value_type> f(x, y);

		f.optimize();

		fAD_ = f;
	}

	//! write code to file
	/*!
	 * @param templateDir directory containing template file
	 * @param outputDir output directory to write file to
	 * @param jacName name of the Jacobian class
	 * @param ns1 first layer namespace of the Jacobian class
	 * @param ns2 second layer namespace of the Jacobian class
	 * @param codeJac source code for computing the Jacobian
	 * @param codePlaceholder placeholder pattern in template file to replace with code
	 */
	void writeCodeFile(
				const std::string& templateDir,
				const std::string& outputDir,
				const std::string& jacName,
				const std::string& ns1,
				const std::string& ns2,
				const std::string& codeJac,
				const std::string& codePlaceholder)
	{
		std::cout << "Generating jacobian..."<<std::endl;

		std::string header = internal::CGHelpers::parseFile(templateDir+"/Jacobian.tpl.h");
		std::string source = internal::CGHelpers::parseFile(templateDir+"/Jacobian.tpl.cpp");

		replaceSizesAndNames(header, jacName, ns1, ns2);
		replaceSizesAndNames(source, jacName, ns1, ns2);

		internal::CGHelpers::replaceOnce(header, "MAX_COUNT", std::to_string(tmpVarCount_));
		internal::CGHelpers::replaceOnce(source, codePlaceholder, codeJac);


		internal::CGHelpers::writeFile(outputDir+"/"+jacName+".h", header);
		internal::CGHelpers::writeFile(outputDir+"/"+jacName+".cpp", source);


		std::cout << "... Done! Successfully generated Jacobian"<<std::endl;
	}


	//! replaces the size and namespaces in the template file
	/*!
	 *
	 * @param file filename to pen
	 * @param systemName name of the Jacobian
	 * @param ns1 first layer namespace
	 * @param ns2 second layer namespace
	 */
	void replaceSizesAndNames(std::string& file, const std::string& systemName, const std::string& ns1, const std::string& ns2)
	{
		internal::CGHelpers::replaceAll(file, "JACOBIAN_NAME", systemName);
		internal::CGHelpers::replaceAll(file, "NS1", ns1);
		internal::CGHelpers::replaceAll(file, "NS2", ns2);
		internal::CGHelpers::replaceAll(file, "IN_DIM", std::to_string(IN_DIM));
		internal::CGHelpers::replaceAll(file, "OUT_DIM", std::to_string(OUT_DIM));
	}

	std::function<OUT_TYPE(const IN_TYPE&)> f_; //! the function

	bool compiled_; //! flag if Jacobian is compiled

	int inputDim_; //! function input dimension
	int outputDim_; //! function output dimension

	CppAD::ADFun<typename SCALAR::value_type> fAD_; //!  auto-diff function

	CppAD::cg::GccCompiler<double> compiler_; //! compile for codegeneration
	std::shared_ptr<CppAD::cg::DynamicLib<double>> dynamicLib_; //! dynamic library to load after compilation
	std::shared_ptr<CppAD::cg::GenericModel<double>> model_; //! the model
	size_t tmpVarCount_; //! number of temporary variables in the source code
};

} /* namespace core */
} /* namespace ct */

#endif /* INCLUDE_CT_CORE_FUNCTION_JACOBIANCG_H_ */
