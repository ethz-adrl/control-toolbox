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

#ifndef CT_OPTCON_CONSTRAINTS_CONSTRAINTSANALYTICAL_H_
#define CT_OPTCON_CONSTRAINTS_CONSTRAINTSANALYTICAL_H_

#include <cppad/example/cppad_eigen.hpp>

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"


namespace ct {
namespace optcon {

/**
 * @ingroup    Constraint
 *
 * @brief      Contains all the constraints using analytically calculated
 *             jacobians
 *
 * @tparam     STATE_DIM  The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class ConstraintContainerAnalytical : public LinearConstraintContainer<STATE_DIM, CONTROL_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::StateVector<STATE_DIM>   state_vector_t;
	typedef core::ControlVector<CONTROL_DIM> input_vector_t;

	typedef ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM>* ConstraintContainerAnalytical_Raw_Ptr_t;

	ConstraintContainerAnalytical()
	{}

	/**
	 * @brief      Constructor using state, control and time
	 *
	 * @param      x     state vector
	 * @param      u     control vector
	 * @param      t     time
	 */
	ConstraintContainerAnalytical(const state_vector_t &x, const input_vector_t &u, const double& t = 0.0)
	{}

	/**
	 * @brief      Copy constructor
	 *
	 * @param      arg   constraint class to copy
	 */
	ConstraintContainerAnalytical(const ConstraintContainerAnalytical& arg) 
	:
	LinearConstraintContainer<STATE_DIM, CONTROL_DIM>(arg),
	evalIntermediate_(arg.evalIntermediate_),
	evalTerminal_(arg.evalTerminal_),
	constraintsIntermediate_(arg.constraintsIntermediate_),
	evalJacSparseStateIntermediate_(arg.evalJacSparseStateIntermediate_),
	evalJacSparseInputIntermediate_(arg.evalJacSparseInputIntermediate_),
	evalJacDenseStateIntermediate_(arg.evalJacDenseStateIntermediate_),
	evalJacDenseInputIntermediate_(arg.evalJacDenseInputIntermediate_),
	constraintsTerminal_(arg.constraintsTerminal_),
	evalJacSparseStateTerminal_(arg.evalJacSparseStateTerminal_),
	evalJacSparseInputTerminal_(arg.evalJacSparseInputTerminal_),
	evalJacDenseStateTerminal_(arg.evalJacDenseStateTerminal_),
	evalJacDenseInputTerminal_(arg.evalJacDenseInputTerminal_)
	{
		// vectors of terms can be resized easily
		constraintsIntermediate_.resize(arg.constraintsIntermediate_.size());
		constraintsTerminal_.resize(arg.constraintsTerminal_.size());

		for(size_t i = 0; i < constraintsIntermediate_.size(); ++i)
			constraintsIntermediate_[i] = std::shared_ptr< ConstraintBase<STATE_DIM, CONTROL_DIM>> (arg.constraintsIntermediate_[i]->clone());

		for(size_t i = 0; i < constraintsTerminal_.size(); ++i)
			constraintsTerminal_[i] = std::shared_ptr< ConstraintBase<STATE_DIM, CONTROL_DIM>> (arg.constraintsTerminal_[i]->clone());
	}

	/**
	 * @brief      Deep-cloning of Constraint
	 *
	 * @return     Copy of this object.
	 */
	virtual ConstraintContainerAnalytical_Raw_Ptr_t clone () const override {return new ConstraintContainerAnalytical(*this);}

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintContainerAnalytical(){}

	/**
	 * @brief      Adds an intermedaite constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addIntermediateConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM>> constraint, bool verbose)
	{
		constraintsIntermediate_.push_back(constraint);
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as Analytical intermediate constraint " << std::endl;
		}
		this->initializedIntermediate_ = false;
	}

	/**
	 * @brief      Adds a terminal constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addTerminalConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM>> constraint, bool verbose)
	{
		constraintsTerminal_.push_back(constraint);
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as Analytical terminal constraint " << std::endl;
		}
		this->initializedTerminal_ = false;
	}

	virtual Eigen::VectorXd evaluateIntermediate() override
	{
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalIntermediate_.segment(count, constraint_dim) = constraint->eval(this->x_, this->u_, this->t_);
			count += constraint_dim;
		}
		return evalIntermediate_;		
	}

	virtual Eigen::VectorXd evaluateTerminal() override
	{
		checkTerminalConstraints();

		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalTerminal_.segment(count, constraint_dim) = constraint->eval(this->x_, this->u_, this->t_);
			count += constraint_dim;
		}
		return evalTerminal_;		
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
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
		{
			size_t nonZerosState = constraint->getNumNonZerosJacobianState();

			if(nonZerosState != 0)
			{
				evalJacSparseStateIntermediate_.segment(count, nonZerosState) = constraint->jacobianStateSparse(this->x_, this->u_, this->t_);
				count += nonZerosState;
			}

		}
		return evalJacSparseStateIntermediate_;
	}

	virtual Eigen::MatrixXd jacobianStateIntermediate() override
	{
		checkIntermediateConstraints();

		Eigen::MatrixXd jacLocal;
		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalJacDenseStateIntermediate_.block(count, 0, constraint_dim, STATE_DIM) = constraint->jacobianState(this->x_, this->u_, this->t_);
			count += constraint_dim;
		}

		return evalJacDenseStateIntermediate_;
	}

	virtual Eigen::VectorXd jacobianStateSparseTerminal() override
	{
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
		{
			size_t nonZerosState = constraint->getNumNonZerosJacobianState();

			if(nonZerosState != 0)
			{
				evalJacSparseStateTerminal_.segment(count, nonZerosState) = constraint->jacobianStateSparse(this->x_, this->u_, this->t_);
				count += nonZerosState;
			}

		}
		return evalJacSparseStateTerminal_;	
	}


	virtual Eigen::MatrixXd jacobianStateTerminal() override
	{
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalJacDenseStateTerminal_.block(count, 0, constraint_dim, STATE_DIM) = constraint->jacobianState(this->x_, this->u_, this->t_);
			count += constraint_dim;
		}

		return evalJacDenseStateTerminal_;
	}

	virtual Eigen::VectorXd jacobianInputSparseIntermediate() override
	{
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
		{
			size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();

			if(nonZerosInput != 0)
			{
				evalJacSparseInputIntermediate_.segment(count, nonZerosInput) = constraint->jacobianInputSparse(this->x_, this->u_, this->t_);
				count += nonZerosInput;				
			}
		}
		return evalJacSparseInputIntermediate_;
	}

	virtual Eigen::MatrixXd jacobianInputIntermediate() override
	{
		checkIntermediateConstraints();

		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalJacDenseInputIntermediate_.block(count, 0, constraint_dim, CONTROL_DIM) = constraint->jacobianInput(this->x_, this->u_, this->t_);
			count += constraint_dim;			
		}

		return evalJacDenseInputIntermediate_;
	}

	virtual Eigen::VectorXd jacobianInputSparseTerminal() override
	{
		checkTerminalConstraints();

		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
		{
			size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();

			if(nonZerosInput != 0)
			{
				evalJacSparseInputTerminal_.segment(count, nonZerosInput) = constraint->jacobianInputSparse(this->x_, this->u_, this->t_);
				count += nonZerosInput;				
			}
		}
		return evalJacSparseInputTerminal_;
	}

	virtual Eigen::MatrixXd jacobianInputTerminal() override
	{
		checkTerminalConstraints();

		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
		{
			size_t constraint_dim = constraint->getConstraintSize();
			evalJacDenseInputTerminal_.block(count, 0, constraint_dim, CONTROL_DIM) = constraint->jacobianInput(this->x_, this->u_, this->t_);
			count += constraint_dim;			
		}

		return evalJacDenseInputTerminal_;
	}

	virtual void sparsityPatternStateIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		checkIntermediateConstraints();

		Eigen::VectorXi iRowLocal;
		Eigen::VectorXi jColLocal;
		Eigen::VectorXi iRowTot;
		Eigen::VectorXi jColTot;

		size_t count = 0;
		size_t constraintCount = 0;

		for(auto constraint : constraintsIntermediate_)
		{
			size_t nonZerosState = constraint->getNumNonZerosJacobianState();
			if(nonZerosState > 0)
			{
				constraint->sparsityPatternState(iRowLocal, jColLocal);

				iRowTot.conservativeResize(count + nonZerosState);
				iRowTot.segment(count, nonZerosState) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(count + nonZerosState);
				jColTot.segment(count, nonZerosState) = jColLocal;

				count += nonZerosState;
			}
			constraintCount += constraint->getConstraintSize();
		}

		iRows = iRowTot;
		jCols = jColTot;
	}

	virtual void sparsityPatternStateTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("sparsityPatternStateTerminalConstraints not initialized yet. Call 'initialize()' before");

		Eigen::VectorXi iRowLocal;
		Eigen::VectorXi jColLocal;
		Eigen::VectorXi iRowTot;
		Eigen::VectorXi jColTot;

		size_t count = 0;
		size_t constraintCount = 0;

		for(auto constraint : constraintsTerminal_)
		{
			size_t nonZerosState = constraint->getNumNonZerosJacobianState();
			if(nonZerosState > 0)
			{
				constraint->sparsityPatternState(iRowLocal, jColLocal);

				iRowTot.conservativeResize(count + nonZerosState);
				iRowTot.segment(count, nonZerosState) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(count + nonZerosState);
				jColTot.segment(count, nonZerosState) = jColLocal;

				count += nonZerosState;
			}
			constraintCount += constraint->getConstraintSize();
		}

		iRows = iRowTot;
		jCols = jColTot;
	}

	virtual void sparsityPatternInputIntermediate(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		checkIntermediateConstraints();

		Eigen::VectorXi iRowLocal;
		Eigen::VectorXi jColLocal;
		Eigen::VectorXi iRowTot;
		Eigen::VectorXi jColTot;

		size_t count = 0;
		size_t constraintCount = 0;

		for(auto constraint : constraintsIntermediate_)
		{
			size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();
			if(nonZerosInput > 0)
			{
				constraint->sparsityPatternInput(iRowLocal, jColLocal);

				iRowTot.conservativeResize(count + nonZerosInput);
				iRowTot.segment(count, nonZerosInput) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(count + nonZerosInput);
				jColTot.segment(count, nonZerosInput) = jColLocal;

				count += nonZerosInput;
			}
			constraintCount += constraint->getConstraintSize();
		}

		iRows = iRowTot;
		jCols = jColTot;
	}

	virtual void sparsityPatternInputTerminal(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("sparsityPatternInputTerminalConstraints not initialized yet. Call 'initialize()' before");

		Eigen::VectorXi iRowLocal;
		Eigen::VectorXi jColLocal;
		Eigen::VectorXi iRowTot;
		Eigen::VectorXi jColTot;

		size_t count = 0;
		size_t constraintCount = 0;

		for(auto constraint : constraintsTerminal_)
		{
			size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();
			if(nonZerosInput > 0)
			{
				constraint->sparsityPatternInput(iRowLocal, jColLocal);

				iRowTot.conservativeResize(count + nonZerosInput);
				iRowTot.segment(count, nonZerosInput) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(count + nonZerosInput);
				jColTot.segment(count, nonZerosInput) = jColLocal;

				count += nonZerosInput;
			}
			constraintCount += constraint->getConstraintSize();
		}

		iRows = iRowTot;
		jCols = jColTot;
	}

	virtual size_t getJacobianStateNonZeroCountIntermediate() override
	{
		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
			count += constraint->getNumNonZerosJacobianState();

		return count;
	}

	virtual size_t getJacobianStateNonZeroCountTerminal() override
	{
		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
			count += constraint->getNumNonZerosJacobianState();

		return count;
	}

	virtual size_t getJacobianInputNonZeroCountIntermediate() override
	{
		size_t count = 0;
		for(auto constraint : constraintsIntermediate_)
			count += constraint->getNumNonZerosJacobianInput();

		return count;
	}

	virtual size_t getJacobianInputNonZeroCountTerminal() override
	{
		size_t count = 0;
		for(auto constraint : constraintsTerminal_)
			count += constraint->getNumNonZerosJacobianInput();
		
		return count;
	}

	virtual bool initializeIntermediate() override
	{
		if(getIntermediateConstraintsCount() > 0)
		{
			evalIntermediate_.resize(getIntermediateConstraintsCount()); evalIntermediate_.setZero();
			evalJacSparseStateIntermediate_.resize(getJacobianStateNonZeroCountIntermediate()); evalJacSparseStateIntermediate_.setZero();
			evalJacSparseInputIntermediate_.resize(getJacobianInputNonZeroCountIntermediate()); evalJacSparseInputIntermediate_.setZero();
			evalJacDenseStateIntermediate_.resize(getIntermediateConstraintsCount(), STATE_DIM); evalJacDenseStateIntermediate_.setZero();
			evalJacDenseInputIntermediate_.resize(getIntermediateConstraintsCount(), CONTROL_DIM); evalJacDenseInputIntermediate_.setZero();

			size_t count = 0;

			this->lowerBoundsIntermediate_.resize(getIntermediateConstraintsCount()); this->lowerBoundsIntermediate_.setZero();
			this->upperBoundsIntermediate_.resize(getIntermediateConstraintsCount()); this->upperBoundsIntermediate_.setZero();

			for(auto constraint : constraintsIntermediate_)
			{
				size_t constraintSize = constraint->getConstraintSize();
				this->lowerBoundsIntermediate_.segment(count, constraintSize) = constraint->getLowerBound();
				this->upperBoundsIntermediate_.segment(count, constraintSize) = constraint->getUpperBound();
				count += constraintSize;
			}
		}

		return true;
	}

	virtual bool initializeTerminal() override
	{
		if(getTerminalConstraintsCount() > 0)
		{
			evalTerminal_.resize(getTerminalConstraintsCount()); evalTerminal_.setZero();
			evalJacSparseStateTerminal_.resize(getJacobianStateNonZeroCountTerminal()); evalJacSparseStateTerminal_.setZero();
			evalJacSparseInputTerminal_.resize(getJacobianInputNonZeroCountTerminal()); evalJacSparseInputTerminal_.setZero();
			evalJacDenseStateTerminal_.resize(getTerminalConstraintsCount(), STATE_DIM); evalJacDenseStateTerminal_.setZero();
			evalJacDenseInputTerminal_.resize(getTerminalConstraintsCount(), CONTROL_DIM); evalJacDenseInputTerminal_.setZero();

			size_t count = 0;

			this->lowerBoundsTerminal_.resize(getTerminalConstraintsCount()); this->lowerBoundsTerminal_.setZero();
			this->upperBoundsTerminal_.resize(getTerminalConstraintsCount()); this->upperBoundsTerminal_.setZero();

			for(auto constraint : constraintsTerminal_)
			{
				size_t constraintSize = constraint->getConstraintSize();
				this->lowerBoundsTerminal_.segment(count, constraintSize) = constraint->getLowerBound();
				this->upperBoundsTerminal_.segment(count, constraintSize) = constraint->getUpperBound();
				count += constraintSize;
			}
		}

		return true;
	}


private:
	virtual void update() override {}

	/**
	 * @brief      Checks whether the intermediate constraints are initialized.
	 *             Throws a runtime error if not.
	 */
	void checkIntermediateConstraints()
	{
		if(!this->initializedIntermediate_)
			throw std::runtime_error("Error: Intermediate constraints are or not initialized yet. ");
	}

	/**
	 * @brief      Checks whether the terminal constraints are initialized.
	 *             Throws a runtime error if not.
	 */
	void checkTerminalConstraints()
	{
		if(!this->initializedTerminal_)
			throw std::runtime_error("Error: Terminal constraints are either not initialized yet. ");
	}


	std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM>>> constraintsIntermediate_;
	std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM>>> constraintsTerminal_;

	Eigen::VectorXd evalIntermediate_;
	Eigen::VectorXd evalJacSparseStateIntermediate_;
	Eigen::VectorXd evalJacSparseInputIntermediate_;
	Eigen::MatrixXd evalJacDenseStateIntermediate_;
	Eigen::MatrixXd evalJacDenseInputIntermediate_;

	Eigen::VectorXd evalTerminal_;
	Eigen::VectorXd evalJacSparseStateTerminal_;
	Eigen::VectorXd evalJacSparseInputTerminal_;
	Eigen::MatrixXd evalJacDenseStateTerminal_;
	Eigen::MatrixXd evalJacDenseInputTerminal_;

};


}// namespace optcon
}// namespace ct

#endif
