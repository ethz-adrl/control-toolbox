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

#ifndef CT_OPTCON_CONSTRAINTS_CONSTRAINTCONTAINERAD_H_
#define CT_OPTCON_CONSTRAINTS_CONSTRAINTCONTAINERAD_H_

#include <ct/core/math/DerivativesCppad.h>

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"
#include <ct/core/internal/traits/CppADCodegenTrait.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Contains all the constraints using with AD generated jacobians
 *
 * @tparam     STATE_DIM  { description }
 * @tparam     CONTROL_DIM  { description }
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintContainerAD : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::DerivativesCppad<STATE_DIM + CONTROL_DIM, -1> JacCG;
	typedef typename JacCG::CG_SCALAR Scalar;

	typedef core::StateVector<STATE_DIM>   state_vector_t;
	typedef core::ControlVector<CONTROL_DIM> input_vector_t;

	typedef ConstraintContainerAD<STATE_DIM, CONTROL_DIM>* ConstraintContainerAD_Raw_Ptr_t;


	/**
	 * @brief      Basic constructor
	 */
	ConstraintContainerAD()
	{
		stateControlD_.setZero();

		fIntermediate_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput){
			return this->evaluateIntermediateCodegen(stateinput);	
		};

		fTerminal_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput){
			return this->evaluateTerminalCodegen(stateinput);	
		};

		intermediateCodegen_ = std::shared_ptr<JacCG>(new JacCG(fIntermediate_, STATE_DIM + CONTROL_DIM, getIntermediateConstraintsCount()));
		terminalCodegen_ = std::shared_ptr<JacCG>(new JacCG(fTerminal_, STATE_DIM + CONTROL_DIM, getTerminalConstraintsCount()));

	}

	/**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	ConstraintContainerAD(const state_vector_t &x, const input_vector_t &u, const double& t = 0.0) 
	{
		//Set to some random number which is != the initguess of the problem
		stateControlD_ << x, u;
	}

	/**
	 * \brief Copy constructor
	 * @param arg constraint class to copy
	 */
	ConstraintContainerAD(const ConstraintContainerAD& arg)
	:
	LinearConstraintContainer<STATE_DIM, CONTROL_DIM>(arg),
	constraintsIntermediate_(arg.constraintsIntermediate_),
	constraintsTerminal_(arg.constraintsTerminal_),
	intermediateCodegen_(arg.intermediateCodegen_),
	terminalCodegen_(arg.terminalCodegen_),
	fIntermediate_(arg.fIntermediate_),
	fTerminal_(arg.fTerminal_),
	sparsityIntermediateRows_(arg.sparsityIntermediateRows_),
	sparsityStateIntermediateRows_(arg.sparsityStateIntermediateRows_),
	sparsityStateIntermediateCols_(arg.sparsityStateIntermediateCols_),
	sparsityInputIntermediateRows_(arg.sparsityInputIntermediateRows_),
	sparsityInputIntermediateCols_(arg.sparsityInputIntermediateCols_),
	sparsityTerminalRows_(arg.sparsityTerminalRows_),
	sparsityStateTerminalRows_(arg.sparsityStateTerminalRows_),
	sparsityStateTerminalCols_(arg.sparsityStateTerminalCols_),
	sparsityInputTerminalRows_(arg.sparsityInputTerminalRows_),
	sparsityInputTerminalCols_(arg.sparsityInputTerminalCols_),
	stateControlD_(arg.stateControlD_)
	{
		constraintsIntermediate_.resize(arg.constraintsIntermediate_.size());
		constraintsTerminal_.resize(arg.constraintsTerminal_.size());

		for(size_t i = 0; i < constraintsIntermediate_.size(); ++i)
			constraintsIntermediate_[i] = std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>> (arg.constraintsIntermediate_[i]->clone());

		for(size_t i = 0; i < constraintsTerminal_.size(); ++i)
			constraintsTerminal_[i] = std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>> (arg.constraintsTerminal_[i]->clone());
	}

	/**
	 * Deep-cloning of Constraint
	 *
	 * @return     base pointer to the clone
	 */
	virtual ConstraintContainerAD_Raw_Ptr_t clone () const override {return new ConstraintContainerAD(*this);}

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintContainerAD() {};

	/**
	 * @brief      Adds an intermediate constraint.
	 *
	 * @param[in]  constraint  The constraint
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addIntermediateConstraint(std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>> constraint, bool verbose)
	{
		constraintsIntermediate_.push_back(constraint);
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as AD intermediate constraint " << std::endl;
		}

		fIntermediate_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput){
			return this->evaluateIntermediateCodegen(stateinput);	
		};

		intermediateCodegen_->update(fIntermediate_, STATE_DIM + CONTROL_DIM, getIntermediateConstraintsCount());

		this->initializedIntermediate_ = false;
	}

	/**
	 * @brief      Adds a terminal constraint.
	 *
	 * @param[in]  constraint  The constraint
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addTerminalConstraint(std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>> constraint, bool verbose)
	{
		constraintsTerminal_.push_back(constraint);
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as AD terminal constraint " << std::endl;
		}

		fTerminal_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput){
			return this->evaluateTerminalCodegen(stateinput);	
		};

		terminalCodegen_->update(fTerminal_, STATE_DIM + CONTROL_DIM, getTerminalConstraintsCount());

		this->initializedTerminal_ = false;
	}


	virtual Eigen::VectorXd evaluateIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("evaluateIntermediateConstraints not initialized yet. Call 'initialize()' before");


		return intermediateCodegen_->forwardZero(stateControlD_);	
	}

	virtual Eigen::VectorXd evaluateTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("evaluateTerminalConstraints not initialized yet. Call 'initialize()' before");

		return terminalCodegen_->forwardZero(stateControlD_);
	}

	virtual size_t getIntermediateConstraintsCount() override
	{
		size_t count = 0;

		for(auto constraint : constraintsIntermediate_)
			count += constraint->getConstraintSize();

		return count;
	}

	virtual size_t getTerminalConstraintsCount() override
	{
		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
			count += constraint->getConstraintSize();

		return count;
	}

	virtual Eigen::VectorXd jacobianStateSparseIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = intermediateCodegen_->jacobian(stateControlD_);

		// std::cout << "jacTot" << std::endl;

		Eigen::VectorXd jacSparse; jacSparse.resize(getJacobianStateNonZeroCountIntermediate());
		for(size_t i = 0; i < getJacobianStateNonZeroCountIntermediate(); ++i)
			jacSparse(i) = (jacTot.template leftCols<STATE_DIM>())(sparsityStateIntermediateRows_(i), sparsityStateIntermediateCols_(i));

		return jacSparse;
	}

	virtual Eigen::MatrixXd jacobianStateIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = intermediateCodegen_->jacobian(stateControlD_);
		return jacTot.template leftCols<STATE_DIM>();
	}

	virtual Eigen::VectorXd jacobianStateSparseTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = terminalCodegen_->jacobian(stateControlD_);
		Eigen::VectorXd jacSparse; jacSparse.resize(getJacobianStateNonZeroCountTerminal());
		for(size_t i = 0; i < getJacobianStateNonZeroCountTerminal(); ++i)
			jacSparse(i) = (jacTot.template leftCols<STATE_DIM>())(sparsityStateTerminalRows_(i), sparsityStateTerminalCols_(i));

		return jacSparse;		
	}

	virtual Eigen::MatrixXd jacobianStateTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = terminalCodegen_->jacobian(stateControlD_);
		return jacTot.template leftCols<STATE_DIM>();
	}

	virtual Eigen::VectorXd jacobianInputSparseIntermediate() override
	{
		Eigen::MatrixXd jacTot = intermediateCodegen_->jacobian(stateControlD_);
		Eigen::VectorXd jacSparse; jacSparse.resize(getJacobianInputNonZeroCountIntermediate());
		for(size_t i = 0; i < getJacobianInputNonZeroCountIntermediate(); ++i)
			jacSparse(i) = (jacTot.template rightCols<CONTROL_DIM>())(sparsityInputIntermediateRows_(i), sparsityInputIntermediateCols_(i));

		return jacSparse;
	}	

	virtual Eigen::MatrixXd jacobianInputIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = intermediateCodegen_->jacobian(stateControlD_);
		return jacTot.template rightCols<CONTROL_DIM>();
	}

	virtual Eigen::VectorXd jacobianInputSparseTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = terminalCodegen_->jacobian(stateControlD_);
		Eigen::VectorXd jacSparse; jacSparse.resize(getJacobianInputNonZeroCountTerminal());
		for(size_t i = 0; i < getJacobianInputNonZeroCountTerminal(); ++i)
			jacSparse(i) = (jacTot.template rightCols<CONTROL_DIM>())(sparsityInputTerminalRows_(i), sparsityInputTerminalCols_(i));

		return jacSparse;
	}

	virtual Eigen::MatrixXd jacobianInputTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacTot = terminalCodegen_->jacobian(stateControlD_);
		return jacTot.template rightCols<CONTROL_DIM>();
	}

	virtual void sparsityPatternStateIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityStateIntermediateRows_;
		jCols = sparsityStateIntermediateCols_;
	}

	virtual void sparsityPatternStateTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityStateTerminalRows_;
		jCols = sparsityStateTerminalCols_;
	}	

	virtual void sparsityPatternInputIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityInputIntermediateRows_;
		jCols = sparsityInputIntermediateCols_;
	}	

	virtual void sparsityPatternInputTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityInputTerminalRows_;
		jCols = sparsityInputTerminalCols_;
	}

	virtual size_t getJacobianStateNonZeroCountIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityStateIntermediateRows_.rows();
	}

	virtual size_t getJacobianStateNonZeroCountTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityStateTerminalRows_.rows();
	}

	virtual size_t getJacobianInputNonZeroCountIntermediate() override
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityInputIntermediateRows_.rows();
	}

	virtual size_t getJacobianInputNonZeroCountTerminal() override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityInputTerminalRows_.rows();
	}

	virtual bool initializeIntermediate() override
	{	

		Eigen::VectorXi sparsityRows;
		Eigen::VectorXi sparsityCols;

		if(getIntermediateConstraintsCount() > 0)
		{
			intermediateCodegen_->compileJIT("intermediateConstraints");
			intermediateCodegen_->getSparsityPatternJacobian(sparsityRows, sparsityCols);

			std::cout << "sparsityPattern Intermediate: " << std::endl << intermediateCodegen_->getSparsityPatternJacobian() << std::endl;
			assert(sparsityRows.rows() == sparsityRows.rows());
			
			int nonZerosState = (sparsityCols.array() < STATE_DIM).count();
			int nonZerosInput = (sparsityCols.array() >= STATE_DIM).count();

			sparsityStateIntermediateRows_.resize(nonZerosState);
			sparsityStateIntermediateCols_.resize(nonZerosState);
			sparsityInputIntermediateRows_.resize(nonZerosInput);
			sparsityInputIntermediateCols_.resize(nonZerosInput);

			size_t count = 0;

			this->lowerBoundsIntermediate_.resize(getIntermediateConstraintsCount());
			this->upperBoundsIntermediate_.resize(getIntermediateConstraintsCount());

			for(auto constraint : constraintsIntermediate_)
			{
				size_t constraintSize = constraint->getConstraintSize();
				this->lowerBoundsIntermediate_.segment(count, constraintSize) = constraint->getLowerBound();
				this->upperBoundsIntermediate_.segment(count, constraintSize) = constraint->getUpperBound();
				count += constraintSize;
			}

			size_t stateIndex = 0;
			size_t inputIndex = 0;

			for(size_t i = 0; i < sparsityRows.rows(); ++i)
			{
				if(sparsityCols(i) < STATE_DIM)
				{	
					sparsityStateIntermediateRows_(stateIndex) = sparsityRows(i);
					sparsityStateIntermediateCols_(stateIndex) = sparsityCols(i);
					stateIndex++;
				}
				else
				{
					sparsityInputIntermediateRows_(inputIndex) = sparsityRows(i);
					sparsityInputIntermediateCols_(inputIndex) = sparsityCols(i) - STATE_DIM;
					inputIndex++;
				}
			}	
		}

		return true;
	}

	virtual bool initializeTerminal() override
	{
		Eigen::VectorXi sparsityRows;
		Eigen::VectorXi sparsityCols;

		if(getTerminalConstraintsCount() > 0)
		{
			terminalCodegen_->compileJIT("terminalConstraints");
			terminalCodegen_->getSparsityPatternJacobian(sparsityRows, sparsityCols);

			std::cout << "sparsityPattern Terminal: " << std::endl << terminalCodegen_->getSparsityPatternJacobian() << std::endl;
			assert(sparsityRows.rows() == sparsityRows.rows());

			int nonZerosState = (sparsityCols.array() < STATE_DIM).count();
			int nonZerosInput = (sparsityCols.array() >= STATE_DIM).count();

			sparsityStateTerminalRows_.resize(nonZerosState);
			sparsityStateTerminalCols_.resize(nonZerosState);
			sparsityInputTerminalRows_.resize(nonZerosInput);
			sparsityInputTerminalCols_.resize(nonZerosInput);

			size_t count = 0;

			this->lowerBoundsTerminal_.resize(getTerminalConstraintsCount());
			this->upperBoundsTerminal_.resize(getTerminalConstraintsCount());

			for(auto constraint : constraintsTerminal_)
			{
				size_t constraintSize = constraint->getConstraintSize();
				this->lowerBoundsTerminal_.segment(count, constraintSize) = constraint->getLowerBound();
				this->upperBoundsTerminal_.segment(count, constraintSize) = constraint->getUpperBound();
				count += constraintSize;
			}

			size_t stateIndex = 0;
			size_t inputIndex = 0;

			for(size_t i = 0; i < sparsityRows.rows(); ++i)
			{
				if(sparsityCols(i) < STATE_DIM)
				{	
					sparsityStateTerminalRows_(stateIndex) = sparsityRows(i);
					sparsityStateTerminalCols_(stateIndex) = sparsityCols(i);
					stateIndex++;
				}
				else
				{
					sparsityInputTerminalRows_(inputIndex) = sparsityRows(i);
					sparsityInputTerminalCols_(inputIndex) = sparsityCols(i) - STATE_DIM;
					inputIndex++;
				}
			}
		}

		return true;
	}	

private:
	// cache the matrix here, no need to call it at every eval matrix
	virtual void update() override { stateControlD_ << this->x_, this->u_; }

	/**
	 * @brief      Helper function to keep track of the constraint evaluation
	 *             used for cppad codegeneration
	 *
	 * @param[in]  stateinput  The stacked state input vector
	 *
	 * @return     The evaluated intermediate constraints
	 */
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> evaluateIntermediateCodegen(const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput)
	{
		size_t count = 0;
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> gLocal;

		for(auto constraint : constraintsIntermediate_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			gLocal.conservativeResize(count + constraint_dim);
			gLocal.segment(count, constraint_dim) = constraint->evaluate(stateinput.segment(0,STATE_DIM), stateinput.segment(STATE_DIM, CONTROL_DIM), Scalar(0.0));
			count += constraint_dim;
		}
		return gLocal;		
	}

	/**
	 * @brief      Helper function to keep track of the constraint evaluation
	 *             used for cppad codegeneration
	 *
	 * @param[in]  stateinput  The stacked state input vector
	 *
	 * @return     The evaluated terminal constraints
	 */
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> evaluateTerminalCodegen(const Eigen::Matrix<Scalar, STATE_DIM + CONTROL_DIM, 1>& stateinput)
	{
		size_t count = 0;
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> gLocal;

		for(auto constraint : constraintsTerminal_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			gLocal.conservativeResize(count + constraint_dim);
			gLocal.segment(count, constraint_dim) = constraint->evaluate(stateinput.segment(0,STATE_DIM), stateinput.segment(STATE_DIM, CONTROL_DIM), Scalar(0.0));
			count += constraint_dim;
		}

		return gLocal;		
	}

	//containers
	std::vector<std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>>> constraintsIntermediate_;
	std::vector<std::shared_ptr<tpl::ConstraintBase<STATE_DIM, CONTROL_DIM, Scalar>>> constraintsTerminal_;

	std::shared_ptr<JacCG> intermediateCodegen_;
	std::shared_ptr<JacCG> terminalCodegen_;

	typename JacCG::FUN_TYPE_CG fIntermediate_;
	typename JacCG::FUN_TYPE_CG fTerminal_;

	Eigen::VectorXi sparsityIntermediateRows_;
	Eigen::VectorXi sparsityStateIntermediateRows_;
	Eigen::VectorXi sparsityStateIntermediateCols_;
	Eigen::VectorXi sparsityInputIntermediateRows_;
	Eigen::VectorXi sparsityInputIntermediateCols_;

	Eigen::VectorXi sparsityTerminalRows_;
	Eigen::VectorXi sparsityStateTerminalRows_;
	Eigen::VectorXi sparsityStateTerminalCols_;
	Eigen::VectorXi sparsityInputTerminalRows_;
	Eigen::VectorXi sparsityInputTerminalCols_;


	Eigen::Matrix<double, STATE_DIM + CONTROL_DIM, 1> stateControlD_; /** contains x, u in stacked form */
};



}// namespace optcon
}// namespace ct

#endif
