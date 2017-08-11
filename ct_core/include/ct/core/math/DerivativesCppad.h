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

#ifndef INCLUDE_CT_CORE_FUNCTION_DERIVATIVESCPPAD_H_
#define INCLUDE_CT_CORE_FUNCTION_DERIVATIVESCPPAD_H_

#include <ct/core/templateDir.h>
#include "DerivativesCppadSettings.h"

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
class DerivativesCppad : public Derivatives<IN_DIM, OUT_DIM, double> // double on purpose!
{
public:
	typedef ADScalar AD_SCALAR;
	typedef ADCGScalar CG_SCALAR; //!< CG_SCALAR  type
	typedef ADCGValueType CG_VALUE_TYPE; //!< autodiff scalar type
	                                   
	typedef Eigen::Matrix<AD_SCALAR, IN_DIM, 1> IN_TYPE_AD; //!< function input vector type
	typedef Eigen::Matrix<AD_SCALAR, OUT_DIM, 1> OUT_TYPE_AD; //!< function  output vector type	

	typedef Eigen::Matrix<CG_SCALAR, IN_DIM, 1> IN_TYPE_CG; //!< function input vector type
	typedef Eigen::Matrix<CG_SCALAR, OUT_DIM, 1> OUT_TYPE_CG; //!< function  output vector type

	typedef Eigen::Matrix<bool, IN_DIM, OUT_DIM> Sparsity; //!< Sparsity pattern type
    typedef	Eigen::Matrix<bool, IN_DIM, IN_DIM> HessianSparsity;                    

	typedef Eigen::Matrix<double, IN_DIM, 1> IN_TYPE_D; //!< function input vector type double
	typedef Eigen::Matrix<double, OUT_DIM, 1> OUT_TYPE_D; //!< function output vector type
	typedef Eigen::Matrix<double, OUT_DIM, IN_DIM> JAC_TYPE_D; //!< Jacobian type
	typedef Eigen::Matrix<double, OUT_DIM, IN_DIM, Eigen::RowMajor> JAC_TYPE_ROW_MAJOR; //!< Jocobian type in row-major format
    typedef	Eigen::Matrix<double, IN_DIM, IN_DIM> HES_TYPE_D;
    typedef	Eigen::Matrix<double, IN_DIM, IN_DIM, Eigen::RowMajor> HES_TYPE_ROW_MAJOR;


	typedef std::function<OUT_TYPE_CG(const IN_TYPE_CG&)> FUN_TYPE_CG; //!< function type
	typedef std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> FUN_TYPE_AD;


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
	DerivativesCppad(
		const DerivativesCppadSettings settings, 
		FUN_TYPE_CG& f, 
		int inputDim = IN_DIM, 
		int outputDim = OUT_DIM) 
	:
		settings_(settings),
		fCgStd_(f),
		compiled_(false),
		inputDim_(inputDim),
		outputDim_(outputDim)
	{
		if(outputDim_ > 0 && inputDim_ > 0)
			recordCg();
	}

	/**
	 * @brief      Constructs the derivatives for autodiff without
	 *             codegeneration using a FUN_TYPE_AD function
	 *
	 * @warning    If IN_DIM and/our OUT_DIM are set to dynamic (-1), then the
	 *             actual dimensions of x and y have to be passed here.
	 *
	 * @param      f          The function to be autodiffed
	 * @param[in]  inputDim   inputDim input dimension, must be specified if
	 *                        template parameter IN_DIM is -1 (dynamic)
	 * @param[in]  outputDim  outputDim output dimension, must be specified if
	 *                        template parameter IN_DIM is -1 (dynamic)
	 */
	DerivativesCppad(
		const DerivativesCppadSettings settings,
		FUN_TYPE_AD& f, 
		int inputDim = IN_DIM, 
		int outputDim = OUT_DIM) 
	:
		settings_(settings),
		fAdStd_(f),
		compiled_(false),
		inputDim_(inputDim),
		outputDim_(outputDim)
	{
		if(outputDim_ > 0 && inputDim_ > 0)
			recordAd();
	}

	//! copy constructor
	DerivativesCppad(const DerivativesCppad& arg) 
	:
		settings_(arg.settings_),
		fCgStd_(arg.fCgStd_),
		fAdStd_(arg.fAdStd_),
		compiled_(arg.compiled_),
		inputDim_(arg.inputDim_),
		outputDim_(arg.outputDim_),
		dynamicLib_(arg.dynamicLib_),
		tmpVarCount_(arg.tmpVarCount_)
	{
		fCgCppad_ = arg.fCgCppad_;
		fAdCppad_ = arg.fAdCppad_;
		if(compiled_)
			model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model("DerivativesCppad"));
	}

	//! destructor
	virtual ~DerivativesCppad()
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
	void update(FUN_TYPE_CG& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM)
	{
		fCgStd_ = f;
		outputDim_ = outputDim;
		inputDim_ = inputDim;
		recordCg();
		compiled_ = false;
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
	void update(FUN_TYPE_AD& f, const size_t inputDim = IN_DIM, const size_t outputDim = OUT_DIM)
	{
		fAdStd_ = f;
		outputDim_ = outputDim;
		inputDim_ = inputDim;
		recordAd();
		compiled_ = false;
	}

	//! deep cloning of Jacobian
	DerivativesCppad* clone() const override{
		return new DerivativesCppad<IN_DIM, OUT_DIM>(*this);
	}

	virtual OUT_TYPE_D forwardZero(const Eigen::VectorXd& x) override{

		if(compiled_)
		{
			assert(model_->isForwardZeroAvailable() == true);
			return model_->ForwardZero(x);
		}
		else
			return fAdCppad_.Forward(0, x);
	}

	virtual JAC_TYPE_D jacobian(const Eigen::VectorXd& x) override {
		if(outputDim_ <= 0)
			throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");
			
		if(compiled_)
		{
			assert(model_->isJacobianAvailable() == true);
			Eigen::VectorXd jac = model_->SparseJacobian(x);
			JAC_TYPE_D out(outputDim_, x.rows());
			out = JAC_TYPE_ROW_MAJOR::Map(jac.data(), outputDim_, x.rows());
			return out;
		}
		else
		{
			Eigen::VectorXd jac = fAdCppad_.Jacobian(x);
			JAC_TYPE_D out(outputDim_, x.rows());
			out = JAC_TYPE_ROW_MAJOR::Map(jac.data(), outputDim_, x.rows());
			return out;
		}	
	}

	virtual Eigen::VectorXd evaluateSparseJacobian(const Eigen::VectorXd& x) override {
		if(outputDim_ <= 0)
			throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");
			
		if(compiled_)
		{
			assert(model_->isSparseJacobianAvailable() == true);
			std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
			std::vector<double> output;
			model_->SparseJacobian(input, output, sparsityRowsJacobian_, sparsityColsJacobian_);
			return Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
		}
		else
		{
			return fAdCppad_.SparseJacobian(x);
		}	
	}

    virtual void sparseJacobian(
        const Eigen::VectorXd& x,
        Eigen::VectorXd& jac,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol)
    {
		if(outputDim_ <= 0)
			throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");
			
		if(compiled_)
		{
			assert(model_->isSparseJacobianAvailable() == true);
			std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
			std::vector<double> output;
			model_->SparseJacobian(input, output, sparsityRowsJacobian_, sparsityColsJacobian_);
			jac = Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
			iRow = sparsityRowsJacobianEigen_;
			jCol = sparsityColsJacobianEigen_;		
		}
		else
		{
			jac = fAdCppad_.SparseJacobian(x);
		}    	
    }	

	virtual HES_TYPE_D hessian(const Eigen::VectorXd& x, const Eigen::VectorXd& lambda) override {
		if(outputDim_ <= 0)
			throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

		if(compiled_)
		{
			assert(model_->isHessianAvailable() == true);
			Eigen::VectorXd hessian = model_->Hessian(x, lambda);
			HES_TYPE_D out(x.rows(), x.rows());
			out = HES_TYPE_ROW_MAJOR::Map(hessian.data(), x.rows(), x.rows());
			return out;
		}
		else
		{
			Eigen::VectorXd hessian = fAdCppad_.Hessian(x, lambda);
			HES_TYPE_D out(x.rows(), x.rows());
			out = HES_TYPE_ROW_MAJOR::Map(hessian.data(), x.rows(), x.rows());
			return out;
		}
	}


	virtual Eigen::VectorXd evaluateSparseHessian(
		const Eigen::VectorXd& x, 
		const Eigen::VectorXd& lambda) override 
	{
			if(outputDim_ <= 0)
				throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

			if(compiled_)
			{
				assert(model_->isSparseHessianAvailable() == true);
				std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
				std::vector<double> inputLambda(lambda.data(), lambda.data() + lambda.rows() * lambda.cols());
				std::vector<double> output;
				model_->SparseHessian(input, inputLambda, output, sparsityRowsHessian_, sparsityColsHessian_);
				return Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
			}	
			else
				return fAdCppad_.SparseHessian(x, lambda);
	}	

    virtual void sparseHessian(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes,
        Eigen::VectorXi& iRow,
        Eigen::VectorXi& jCol) override 
	{
			if(outputDim_ <= 0)
				throw std::runtime_error("Outdim dim smaller 0; Define output dim in DerivativesCppad constructor");

			if(compiled_)
			{
				assert(model_->isSparseJacobianAvailable() == true);
				std::vector<double> input(x.data(), x.data() + x.rows() * x.cols());
				std::vector<double> inputLambda(lambda.data(), lambda.data() + lambda.rows() * lambda.cols());
				std::vector<double> output;
				model_->SparseHessian(input, inputLambda, output, sparsityRowsHessian_, sparsityColsHessian_);
				hes = Eigen::Map<Eigen::VectorXd>(output.data(), output.size(), 1);
				iRow = sparsityRowsHessianEigen_;
				jCol = sparsityColsHessianEigen_;	
			}	
			else
				hes = fAdCppad_.SparseHessian(x, lambda);
	}


	//! get Jacobian sparsity pattern
	/*!
	 * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern.
	 * @return Sparsity pattern of the Jacobian
	 */
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getSparsityPatternJacobian()
	{
		assert(model_->isJacobianSparsityAvailable() == true);
		
		std::vector<bool> sparsityVec = model_->JacobianSparsityBool();
		Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> sparsityMat(outputDim_, inputDim_);

		assert(sparsityVec.size() == outputDim_ * inputDim_);
		for(size_t row = 0; row < outputDim_; ++row)
			for(size_t col = 0; col < inputDim_; ++col)
				sparsityMat(row, col) = sparsityVec[col + row * inputDim_];

		return sparsityMat;
	}

	/**
	 * @brief      get Hessian sparsity pattern
	 *
	 * @return     Auto-diff automatically detects the sparsity pattern of the Hessian
	 */
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getSparsityPatternHessian()
	{
		assert(model_->isHessianSparsityAvailable() == true);
		
		std::vector<bool> sparsityVec = model_->HessianSparsityBool();
		Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> sparsityMat(inputDim_, inputDim_);

		assert(sparsityVec.size() == inputDim_ * inputDim_);
		for(size_t row = 0; row < inputDim_; ++row)
			for(size_t col = 0; col < inputDim_; ++col)
			{
				// std::cout << "sparsityVec: " << sparsityRowsHessian_[col + row * inputDim_] << std::endl;
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
	void getSparsityPatternJacobian(Eigen::VectorXi& rows, Eigen::VectorXi& columns)
	{
		assert(model_->isJacobianSparsityAvailable() == true);

		rows = sparsityRowsJacobianEigen_;
		columns = sparsityColsJacobianEigen_;
	}

	/**
	 * @brief      Returns the number of nonzeros in the sparse jacobian
	 *
	 * @return     The number of nonzeros in the sparse jacobian
	 */
	size_t getNumNonZerosJacobian()
	{
		assert(model_->isJacobianSparsityAvailable() == true);
		return sparsityRowsJacobian_.size();
	}

	/**
	 * @brief      Returns the number of nonzeros in the sparse hessian
	 *
	 * @return     The number of non zeros in the sparse hessian
	 */
	size_t getNumNonZerosHessian()
	{
		assert(model_->isJacobianSparsityAvailable() == true);
		return sparsityRowsHessian_.size();
	}	

	//! get Hessian sparsity pattern
	/*!
	 * Auto-Diff automatically detects the sparsity pattern of the Jacobian. This method returns the pattern
	 * in row-column format. Row and columns contain the indeces of all non-zero entries.
	 *
	 * @param rows row indeces of non-zero entries
	 * @param columns column indeces of non-zero entries
	 */
	void getSparsityPatternHessian(Eigen::VectorXi& rows, Eigen::VectorXi& columns)
	{
		assert(model_->isHessianSparsityAvailable() == true);

		rows = sparsityRowsHessianEigen_;
		columns = sparsityColsHessianEigen_;
	}


	//! Uses just-in-time compilation to compile the Jacobian and other derivatives
	/*!
	 *  This method generates source code for the Jacobian and zero order derivative. It then compiles
	 *  the source code to a dynamically loadable library that then gets loaded.
	 */
	void compileJIT(const std::string& libName = "threadId" + std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())))
	{
		if (compiled_) return;
		std::cout << "Starting to compile " + libName + " library"  << std::endl;

		CppAD::cg::ModelCSourceGen<double> cgen(fCgCppad_, "DerivativesCppad");

		cgen.setMultiThreading(settings_.multiThreading_);
		cgen.setCreateForwardZero(settings_.createForwardZero_);
		cgen.setCreateForwardOne(settings_.createForwardOne_);
		cgen.setCreateReverseOne(settings_.createReverseOne_);
		cgen.setCreateReverseTwo(settings_.createReverseTwo_);
		cgen.setCreateJacobian(settings_.createJacobian_);
		cgen.setCreateSparseJacobian(settings_.createSparseJacobian_);
		cgen.setCreateHessian(settings_.createHessian_);
		cgen.setCreateSparseHessian(settings_.createSparseHessian_);
		cgen.setMaxAssignmentsPerFunc(settings_.maxAssignements_);

		CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

		// compile source code
		CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen, libName);
		if(settings_.compiler_ == DerivativesCppadSettings::GCC)
		{
			CppAD::cg::GccCompiler<double> compiler;			
			dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler));
		}

		else if(settings_.compiler_ == DerivativesCppadSettings::CLANG)
		{
			CppAD::cg::ClangCompiler<double> compiler;
			dynamicLib_ = std::shared_ptr<CppAD::cg::DynamicLib<double>>(p.createDynamicLibrary(compiler));
		}

		model_ = std::shared_ptr<CppAD::cg::GenericModel<double>>(dynamicLib_->model("DerivativesCppad"));

		compiled_ = true;
		std::cout << "Successfully compiled " << std::endl;

		if(model_->isJacobianSparsityAvailable())
		{
			model_->JacobianSparsity(sparsityRowsJacobian_, sparsityColsJacobian_);
			sparsityRowsJacobianEigen_.resize(sparsityRowsJacobian_.size()); //rowsTmp.resize(rowsVec.size());
			sparsityColsJacobianEigen_.resize(sparsityColsJacobian_.size()); //colsTmp.resize(rowsVec.size());

			Eigen::Matrix<size_t, Eigen::Dynamic, 1> rowsSizeT; rowsSizeT.resize(sparsityRowsJacobian_.size());
			Eigen::Matrix<size_t, Eigen::Dynamic, 1> colsSizeT; colsSizeT.resize(sparsityColsJacobian_.size());

			rowsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(sparsityRowsJacobian_.data(), sparsityRowsJacobian_.size(), 1);
			colsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(sparsityColsJacobian_.data(), sparsityColsJacobian_.size(), 1);

			sparsityRowsJacobianEigen_ = rowsSizeT.cast<int>();
			sparsityColsJacobianEigen_ = colsSizeT.cast<int>();

		}

		if(model_->isHessianSparsityAvailable())
		{
			model_->HessianSparsity(sparsityRowsHessian_, sparsityColsHessian_);
			sparsityRowsHessianEigen_.resize(sparsityRowsHessian_.size()); //rowsTmp.resize(rowsVec.size());
			sparsityColsHessianEigen_.resize(sparsityColsHessian_.size()); //colsTmp.resize(rowsVec.size());

			Eigen::Matrix<size_t, Eigen::Dynamic, 1> rowsSizeT; rowsSizeT.resize(sparsityRowsHessian_.size());
			Eigen::Matrix<size_t, Eigen::Dynamic, 1> colsSizeT; colsSizeT.resize(sparsityColsHessian_.size());

			rowsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(sparsityRowsHessian_.data(), sparsityRowsHessian_.size(), 1);
			colsSizeT = Eigen::Map<Eigen::Matrix<size_t, Eigen::Dynamic, 1>>(sparsityColsHessian_.data(), sparsityColsHessian_.size(), 1);

			sparsityRowsHessianEigen_ = rowsSizeT.cast<int>();
			sparsityColsHessianEigen_ = colsSizeT.cast<int>();
		}
	}


	//! Generates code for computing the Jacobian and writes it to file
	/*!
	 * This function is optional and can be used if you want to generate code for a Jacobian
	 * and write it to file. This can be useful to pre-compile Jacobians rather than compiling
	 * them at runtime using compileJIT(). This function can make use of a template file in which
	 * the keyword "AUTOGENERATED_CODE_PLACEHOLDER" is replaced with the autogenerated code.
	 *
	 * @param derivativeName name of the resulting Jacobian class
	 * @param outputDir output directory
	 * @param templateDir directory in which template file is located
	 * @param ns1 first namespace layer of the Jacobian class
	 * @param ns2 second namespace layer of the Jacobian class
	 * @param sparsity sparsity pattern to generate sparse Jacobian
	 * @param useReverse if true, uses Auto-Diff reverse mode, otherwise uses forward mode
	 * @param ignoreZero do not assign 0 to sparse entries or zero entries
	 */
	void generateJacobianSource(
			const std::string& derivativeName,
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

		size_t jacDimension = IN_DIM * OUT_DIM;

		std::string codeJac =
				internal::CGHelpers::generateJacobianSource(
						fCgCppad_,
						pattern,
						jacDimension,
						tmpVarCount_,
						useReverse,
						ignoreZero
				);

		writeCodeFile(templateDir, "/Jacobian.tpl.h", "/Jacobian.tpl.cpp", outputDir, derivativeName, ns1, ns2, codeJac, "AUTOGENERATED_CODE_PLACEHOLDER");
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
	void generateForwardZeroSource(
			const std::string& forwardZeroName,
			const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
			const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
			const std::string& ns1 = "core",
			const std::string& ns2 = "generated",
			bool ignoreZero = true)
	{
		std::string codeJac =
				internal::CGHelpers::generateForwardZeroSource(
						fCgCppad_,
						tmpVarCount_,
						ignoreZero
				);

		writeCodeFile(templateDir, "/ForwardZero.tpl.h", "/ForwardZero.tpl.cpp", outputDir, forwardZeroName, ns1, ns2, codeJac, "AUTOGENERATED_CODE_PLACEHOLDER");
	}

	//! Generates code for computing the Hessian and writes it to file
	/*!
	 * This function is optional and can be used if you want to generate code for a hessian,
	 * i.e. the function itself and write it to file. While it seems weird at first to regenerate code from
	 * existing code, this can help to speed up complex computations. Generating source code can be useful
	 * to pre-compile the hessian dynamics rather than compiling them at runtime using compileJIT().
	 * This function can make use of a template file in which the keyword "AUTOGENERATED_CODE_PLACEHOLDER"
	 * is replaced with the autogenerated code.
	 *
	 * @param derivativeName name of the resulting class
	 * @param outputDir output directory
	 * @param templateDir directory in which template file is located
	 * @param ns1 first namespace layer of the class
	 * @param ns2 second namespace layer of the class
	 * @param sparsity sparsity pattern to generate sparse Jacobian
	 * @param useReverse if true, uses Auto-Diff reverse mode, otherwise uses forward mode
	 * @param ignoreZero do not assign 0 to sparse entries or zero entries
	 */
	void generateHessianSource(
			const std::string& derivativeName,
			const std::string& outputDir = ct::core::CODEGEN_OUTPUT_DIR,
			const std::string& templateDir = ct::core::CODEGEN_TEMPLATE_DIR,
			const std::string& ns1 = "core",
			const std::string& ns2 = "generated",
			const HessianSparsity& sparsity = HessianSparsity::Ones(),
			bool useReverse = true,
			bool ignoreZero = true)
	{
		internal::SparsityPattern pattern;
		pattern.initPattern(sparsity);

		size_t hesDimension = IN_DIM * IN_DIM;

		std::string codeHes =
				internal::CGHelpers::generateHessianSource(
						fCgCppad_,
						pattern,
						hesDimension,
						tmpVarCount_,
						ignoreZero
				);

		writeCodeFile(templateDir, "/Hessian.tpl.h", "/Hessian.tpl.cpp", outputDir, derivativeName, ns1, ns2, codeHes, "AUTOGENERATED_CODE_PLACEHOLDER");
	}



private:
	DerivativesCppadSettings settings_;
	std::vector<size_t> sparsityRowsJacobian_;
	std::vector<size_t> sparsityColsJacobian_;
	std::vector<size_t> sparsityRowsHessian_;
	std::vector<size_t> sparsityColsHessian_;

	Eigen::VectorXi sparsityRowsJacobianEigen_;
	Eigen::VectorXi sparsityColsJacobianEigen_;
	Eigen::VectorXi sparsityRowsHessianEigen_;
	Eigen::VectorXi sparsityColsHessianEigen_;

	//! record the Auto-Diff terms for code generation
	void recordCg()
	{
		// input vector, needs to be dynamic size
		Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> x(inputDim_);

		// declare x as independent
		CppAD::Independent(x);

		// output vector, needs to be dynamic size
		Eigen::Matrix<CG_SCALAR, Eigen::Dynamic, 1> y(outputDim_);

		y = fCgStd_(x);

		// store operation sequence in f: x -> y and stop recording
		CppAD::ADFun<CG_VALUE_TYPE> fCodeGen(x, y);

		fCodeGen.optimize();

		fCgCppad_ = fCodeGen;
	}

	/**
	 * @brief      Records the auto-diff terms
	 */
	void recordAd()
	{
		// input vector, needs to be dynamic size
		Eigen::Matrix<AD_SCALAR, Eigen::Dynamic, 1> x(inputDim_);

		// declare x as independent
		CppAD::Independent(x);

		// output vector, needs to be dynamic size
		Eigen::Matrix<AD_SCALAR, Eigen::Dynamic, 1> y(outputDim_);

		y = fAdStd_(x);

		// store operation sequence in f: x -> y and stop recording
		CppAD::ADFun<double> fAd(x, y);

		fAd.optimize();

		fAdCppad_ = fAd;		
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
				const std::string& tplHeaderName,
				const std::string& tplSourceName,
				const std::string& outputDir,
				const std::string& derivativeName,
				const std::string& ns1,
				const std::string& ns2,
				const std::string& codeJac,
				const std::string& codePlaceholder)
	{
		std::cout << "Writing code of " + derivativeName + " to file..."<<std::endl;

		std::string header = internal::CGHelpers::parseFile(templateDir + tplHeaderName);
		std::string source = internal::CGHelpers::parseFile(templateDir + tplSourceName);

		replaceSizesAndNames(header, derivativeName, ns1, ns2);
		replaceSizesAndNames(source, derivativeName, ns1, ns2);

		internal::CGHelpers::replaceOnce(header, "MAX_COUNT", std::to_string(tmpVarCount_));
		internal::CGHelpers::replaceOnce(source, codePlaceholder, codeJac);


		internal::CGHelpers::writeFile(outputDir+"/"+derivativeName+".h", header);
		internal::CGHelpers::writeFile(outputDir+"/"+derivativeName+".cpp", source);


		std::cout << "... Done! Successfully generated " + derivativeName << std::endl;
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
		internal::CGHelpers::replaceAll(file, "DERIVATIVE_NAME", systemName);
		internal::CGHelpers::replaceAll(file, "NS1", ns1);
		internal::CGHelpers::replaceAll(file, "NS2", ns2);
		internal::CGHelpers::replaceAll(file, "IN_DIM", std::to_string(IN_DIM));
		internal::CGHelpers::replaceAll(file, "OUT_DIM", std::to_string(OUT_DIM));
	}

	std::function<OUT_TYPE_CG(const IN_TYPE_CG&)> fCgStd_; //! the function
	std::function<OUT_TYPE_AD(const IN_TYPE_AD&)> fAdStd_;
	                                            //! 
	bool compiled_; //! flag if Jacobian is compiled

	int inputDim_; //! function input dimension
	int outputDim_; //! function output dimension

	CppAD::ADFun<CG_VALUE_TYPE> fCgCppad_; //!  auto-diff function
	CppAD::ADFun<double> fAdCppad_;

	CppAD::cg::GccCompiler<double> compiler_; //! compile for codegeneration
 	CppAD::cg::ClangCompiler<double> compilerClang_;
	std::shared_ptr<CppAD::cg::DynamicLib<double>> dynamicLib_; //! dynamic library to load after compilation
	std::shared_ptr<CppAD::cg::GenericModel<double>> model_; //! the model
	size_t tmpVarCount_; //! number of temporary variables in the source code
};

} /* namespace core */
} /* namespace ct */

#endif /* INCLUDE_CT_CORE_FUNCTION_DERIVATIVESCPPAD_H_ */
