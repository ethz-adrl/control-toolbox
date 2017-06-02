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

#include <ct/core/math/JacobianCG.h>

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
 * @tparam     INPUT_DIM  { description }
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintContainerAD : public LinearConstraintContainer<STATE_DIM, INPUT_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::JacobianCG<STATE_DIM + INPUT_DIM, -1> JacCG;
	typedef typename JacCG::SCALAR Scalar;

	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;
	typedef Eigen::VectorXi VectorXi;
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

	typedef Eigen::VectorXd VectorXd;
	typedef Eigen::MatrixXd MatrixXd;

	typedef core::StateVector<STATE_DIM>   state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef LinearConstraintContainer<STATE_DIM, INPUT_DIM> Base;
	typedef std::shared_ptr<LinearConstraintContainer<STATE_DIM, INPUT_DIM>> ConstraintBase_Shared_Ptr_t;

	typedef ConstraintContainerAD<STATE_DIM, INPUT_DIM>* ConstraintContainerAD_Raw_Ptr_t;

	/**
	 * data for a mapping between constraint index, the term's output dimension, its type and the termId
	 *
	 * START_INDEX: 	in the constraint vector g1 or g2, the START_INDEX describes where the particular constraint term starts
	 * TERM_DIM	  : 	in the constraint vector g1 or g2, the TERM_DIM describes how many scalar entries this term, starting at START_INDEX, comprises
	 * TERM_ID	  :		size_t enumerating the term in it's containing vector. Starts for both AD and analytical terms at 0
	 *
	 */
	enum MAP_DATA {
		START_INDEX = 0,//!< START_INDEX
		TERM_DIM,       //!< TERM_DIM
		TERM_ID         //!< TERM_ID
	};


	/**
	 * mapping for uniquely distinguishing AD and Analytical terms, with the properties defined in MAP_DATA
	 * Therefore, the tuple contains: the entries START_INDEX, TERM_DIM, TYPE, TERM_ID
	 * Implemented using a simple std::vector
	 */
	typedef std::vector<std::tuple<size_t, size_t, size_t>> MAP;

	/**
	 * @brief      Basic constructor
	 */
	ConstraintContainerAD() :
	initialized_(false)
	{
		stateControlD_.setZero();
		//Set to some random number which is != the initguess of the problem
		var_at_cache_.setConstant(777777.120398120938);
		f_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + INPUT_DIM, 1>& stateinput){
			return this->evaluateTot(stateinput);	
		};

		jacCG_ = std::shared_ptr<JacCG>(new JacCG(f_, STATE_DIM + INPUT_DIM, getConstraintCount()));
	}

	/**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	ConstraintContainerAD(const state_vector_t &x, const input_vector_t &u, const double& t = 0.0) :
	initialized_(false)
	{
		//Set to some random number which is != the initguess of the problem
		var_at_cache_.setConstant(777777.120398120938);
		stateControlD_ << x, u;
	}

	/**
	 * \brief Copy constructor
	 * @param arg constraint class to copy
	 */
	ConstraintContainerAD(const ConstraintContainerAD& arg)
	:
	constraintsAD_(arg.constraintsAD_),
	constraintsMap_(arg.constraintsMap_),
	jacCG_(arg.jacCG_),
	f_(arg.f_),
	sparsityRows_(arg.sparsityRows_),
	sparsityCols_(arg.sparsityCols_),
	sparsityStateRows_(arg.sparsityStateRows_),
	sparsityStateCols_(arg.sparsityStateCols_),
	sparsityInputRows_(arg.sparsityInputRows_),
	sparsityInputCols_(arg.sparsityInputCols_),
	jacTot_(arg.jacTot_),
	jacVec_(arg.jacVec_),
	jacState_(arg.jacState_),
	jacInput_(arg.jacInput_),
	nonZerosJac_(arg.nonZerosJac_),
	initialized_(arg.initialized_),	
	var_at_cache_(arg.var_at_cache_),
	stateControlD_(arg.stateControlD_)
	{
		assert(stateControlD_.size() == STATE_DIM+INPUT_DIM);
		// vectors of terms can be resized easily
		constraintsAD_.resize(arg.constraintsAD_.size());

		for(size_t i = 0; i < constraintsAD_.size(); ++i)
		{
			constraintsAD_[i] = std::shared_ptr<tpl::ConstraintBase<STATE_DIM, INPUT_DIM, Scalar>> (arg.constraintsAD_[i]->clone());
		}
	}

	/**
	 * Deep-cloning of Constraint
	 * @return base pointer to the clone
	 */
	virtual ConstraintContainerAD_Raw_Ptr_t clone () const override {return new ConstraintContainerAD(*this);}

	/**
	 * @brief      Destructor
	 */
	virtual ~ConstraintContainerAD() {};

	/**
	 * @brief      Adds a constraint.
	 *
	 * @param[in]  constraint  The constraint to be added
	 * @param[in]  verbose     Flag indicating whether verbosity is on or off
	 */
	void addConstraint(std::shared_ptr<tpl::ConstraintBase<STATE_DIM, INPUT_DIM, Scalar>> constraint, bool verbose)
	{
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as AD state input constraint ";
		}

		addConstraintAndGenerateIndicies(constraint, verbose, constraintsAD_, constraintsMap_);
		f_ = [&] (const Eigen::Matrix<Scalar, STATE_DIM + INPUT_DIM, 1>& stateinput){
			return this->evaluateTot(stateinput);	
		};

		jacCG_->update(f_, STATE_DIM + INPUT_DIM, getConstraintCount());
		initialized_ = false;
	}

	/**
	 * @brief      Initializes the constraints using just in time compilation
	 */
	void initialize()
	{
		jacCG_->compileJIT();
		jacCG_->getSparsityPattern(sparsityRows_, sparsityCols_);
		constraintsCount_ = getConstraintCount();

		// jacCG_->getSparsityPattern();/
		std::cout << "sparsityPattern: " << std::endl << jacCG_->getSparsityPattern() << std::endl;
		assert(sparsityRows_.rows() == sparsityCols_.rows());
		int nonZeroState = (sparsityCols_.array() < STATE_DIM).count();
		int nonZeroInput = (sparsityCols_.array() >= STATE_DIM).count();
		nonZerosJac_ = nonZeroState + nonZeroInput;
		assert(nonZeroState + nonZeroInput == sparsityCols_.size());
		sparsityStateRows_.resize(nonZeroState);
		sparsityStateCols_.resize(nonZeroState);
		sparsityInputRows_.resize(nonZeroInput);
		sparsityInputCols_.resize(nonZeroInput);
		jacTot_.resize(constraintsCount_, STATE_DIM + INPUT_DIM);
		jacState_.resize(constraintsCount_, STATE_DIM);
		jacInput_.resize(constraintsCount_, INPUT_DIM);

		size_t stateIndex = 0;
		size_t inputIndex = 0;
		for(size_t i = 0; i < sparsityRows_.rows(); ++i)
		{
			if(sparsityCols_(i) < STATE_DIM)
			{	
				sparsityStateRows_(stateIndex) = sparsityRows_(i);
				sparsityStateCols_(stateIndex) = sparsityCols_(i);
				stateIndex++;
			}
			else
			{
				sparsityInputRows_(inputIndex) = sparsityRows_(i);
				sparsityInputCols_(inputIndex) = sparsityCols_(i) - STATE_DIM;
				inputIndex++;
			}
		}

		initialized_ = true;
	}

	virtual void setTimeStateInput(const double t, const state_vector_t& x, const input_vector_t& u) override
	{
		stateControlD_.setZero();
		stateControlD_ << x, u;

		// create a autodiff types, too.
		ct::core::StateVector<STATE_DIM, Scalar> x_ad = x.template cast <Scalar>();
		ct::core::ControlVector<INPUT_DIM, Scalar> u_ad = u.template cast <Scalar>();
		Scalar t_ad = (Scalar) t;


		// update state, input and time variables in all the terms
		for(auto constraint : constraintsAD_)
		{
			constraint->setTimeStateInputDouble(x, u, t);
			constraint->setTimeStateInputAd(x_ad, u_ad, t_ad);
		}
	}


	virtual void evaluate(VectorXd& g, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");
		
		count = getConstraintCount();
		g = jacCG_->forwardZero(stateControlD_);
	}

	virtual void getLowerBound(VectorXd& lb, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		count = 0;
		VectorXd lbLocal;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);

			// evaluate analytical constraint
			const size_t constraint_dim = constraintsAD_[constraint_id]->getConstraintsCount();
			lbLocal.conservativeResize(start_index + constraint_dim);
			lbLocal.segment(start_index, constraint_dim) = constraintsAD_[constraint_id]->getLowerBound();
			count += constraint_dim;
		}
		lb = lbLocal;	
	}

	virtual void getUpperBound(VectorXd& ub, size_t& count) override
	{
		count = 0;
		VectorXd ubLocal;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);

			// evaluate analytical constraint
			const size_t constraint_dim = constraintsAD_[constraint_id]->getConstraintsCount();
			ubLocal.conservativeResize(start_index + constraint_dim);
			ubLocal.segment(start_index, constraint_dim) = constraintsAD_[constraint_id]->getUpperBound();
			count += constraint_dim;
		}
		ub = ubLocal;
	}

	VectorXs evaluateTot(const Eigen::Matrix<Scalar, STATE_DIM + INPUT_DIM, 1>& stateinput)
	{
		size_t count = 0;
		VectorXs gLocal;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAD_[constraint_id]->getConstraintsCount();

			gLocal.conservativeResize(start_index + constraint_dim);
			constraintsAD_[constraint_id]->setTimeStateInputAd(stateinput.segment(0,STATE_DIM), stateinput.segment(STATE_DIM, INPUT_DIM), Scalar(0.0));

			gLocal.segment(count, constraint_dim) = constraintsAD_[constraint_id]->evaluate();
			count += constraint_dim;
		}
		return gLocal;		
	}

	void computeConstraintJacobian()
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		jacTot_ = jacCG_->operator()(stateControlD_);

		jacState_ = jacTot_.template leftCols<STATE_DIM>();
		jacInput_ = jacTot_.template rightCols<INPUT_DIM>();

		var_at_cache_ = stateControlD_;		
	}

	virtual void evalJacSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		assert(jacVec.rows() == nonZerosJac_);

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		for(size_t ind = 0; ind < nonZerosJac_; ++ind)
			jacVec(ind) = jacTot_(sparsityRows_(ind), sparsityCols_(ind));

		count = jacVec.rows();
	}

	virtual Eigen::MatrixXd evalJacDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		return jacTot_;
	}	

	virtual void evalJacStateSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		assert(jacVec.rows() == sparsityStateRows_.rows());

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		for(size_t ind = 0; ind < sparsityStateRows_.rows(); ++ind)
			jacVec(ind) = jacState_(sparsityStateRows_(ind), sparsityStateCols_(ind));

		count = jacVec.rows();
	}

	virtual Eigen::MatrixXd evalJacStateDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		return jacState_;			
	}

	virtual void evalJacInputSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		assert(jacVec.rows() == sparsityInputRows_.rows());

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		for(size_t ind = 0; ind < sparsityInputRows_.rows(); ++ind)
			jacVec(ind) = jacInput_(sparsityInputRows_(ind), sparsityInputCols_(ind));

		count = jacVec.rows();
	}


	virtual Eigen::MatrixXd evalJacInputDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		if(stateControlD_ != var_at_cache_)
			computeConstraintJacobian();

		return jacInput_;			
	}

	virtual size_t generateSparsityPatternJacobian(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityRows_;
		jCols = sparsityCols_;
		return iRows.rows();
	}

	virtual size_t generateSparsityPatternJacobianState(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityStateRows_;
		jCols = sparsityStateCols_;
		return sparsityStateRows_.size();
	}

	virtual size_t generateSparsityPatternJacobianInput(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		iRows = sparsityInputRows_;
		jCols = sparsityInputCols_;
		return sparsityInputRows_.size();
	}

	// need to figure out something...
	virtual size_t getConstraintJacobianStateNonZeroCount() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityStateRows_.rows();
	}

	virtual size_t getConstraintJacobianInputNonZeroCount() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return sparsityInputRows_.rows();
	}

	virtual size_t getConstraintJacobianNonZeroCount()
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		return nonZerosJac_;
	}

	virtual size_t getConstraintCount() override
	{
		size_t count = 0;
		for(auto constraint : constraintsAD_)
			count += constraint->getConstraintsCount();

		return count;
	}

	virtual void getConstraintTypes(VectorXd& constraint_types) override
	{
		for(auto map : constraintsMap_)
		{
			const size_t term_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);

			const size_t term_dim = constraintsAD_[term_id]->getConstraintsCount();
			constraint_types.segment(start_index, term_dim) = constraintsAD_[term_id]->getConstraintType() * Eigen::VectorXd::Ones(term_dim);
		}
	}


private:

	/**
	 * @brief      add a term (AD or analytic), and update the mappings between
	 *             the indicies required to identify it uniquely.
	 *
	 *             templated on scalar type (double, CppAD::AD<double)
	 *
	 * @param[in]  constraint          The constraint
	 * @param      verbose             set to true if printout desired
	 * @param      stockOfConstraints  vector terms of same TERM_TYPE (member
	 *                                 variable)
	 * @param      map                 indices mapping
	 */
	void addConstraintAndGenerateIndicies (std::shared_ptr<tpl::ConstraintBase<STATE_DIM, INPUT_DIM, Scalar>> constraint, bool verbose,
			std::vector<std::shared_ptr<tpl::ConstraintBase<STATE_DIM, INPUT_DIM, Scalar>>>& stockOfConstraints, MAP& map)
	{
		stockOfConstraints.push_back(constraint);
		size_t newConstraintId = stockOfConstraints.size()-1;

		size_t constraint_dim = constraint->getConstraintsCount();

		size_t startingIndex = 0;

		// check if map already has value...
		if(map.size()>0)
			startingIndex += (std::get<START_INDEX>(map.back()) + std::get<TERM_DIM>(map.back()));


		auto constraintStartAndDim = std::make_tuple(startingIndex, constraint_dim, newConstraintId);

		map.push_back(constraintStartAndDim);

		if(verbose){
			std::cout << "with starting index " << startingIndex <<", and dimension " << constraint_dim << std::endl;
		}

	}

	virtual void update() override {};	// todo: can we make use of that function ?

	//containers
	std::vector<std::shared_ptr<tpl::ConstraintBase<STATE_DIM, INPUT_DIM, Scalar>>> constraintsAD_;
	MAP constraintsMap_;	/** map containing starting_index, length of constraint sub-vector, term type and term Id for pure state constraint */

	std::shared_ptr<JacCG> jacCG_;
	typename JacCG::Function f_;

	Eigen::VectorXi sparsityRows_;
	Eigen::VectorXi sparsityCols_;
	Eigen::VectorXi sparsityStateRows_;
	Eigen::VectorXi sparsityStateCols_;
	Eigen::VectorXi sparsityInputRows_;
	Eigen::VectorXi sparsityInputCols_;

	Eigen::MatrixXd jacTot_;
	Eigen::MatrixXd jacVec_;
	Eigen::MatrixXd jacState_;
	Eigen::MatrixXd jacInput_;
	size_t nonZerosJac_;
	bool initialized_;
	size_t constraintsCount_;


	Eigen::Matrix<double, STATE_DIM + INPUT_DIM, 1> var_at_cache_;
	Eigen::Matrix<double, STATE_DIM + INPUT_DIM, 1> stateControlD_; /** contains x, u in stacked form */


};



}// namespace optcon
}// namespace ct

#endif
