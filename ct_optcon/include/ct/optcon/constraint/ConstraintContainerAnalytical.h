/*
 * ConstraintsAnalytical.h
 * author: 			mgiftthaler@ethz.ch
 * date created: 	04.10.2016
 *
 */

#ifndef CT_OPTCON_CONSTRAINTSANALYTICAL_H_
#define CT_OPTCON_CONSTRAINTSANALYTICAL_H_

#include <cppad/example/cppad_eigen.hpp>

#include "LinearConstraintContainer.h"
#include "term/ConstraintBase.h"


namespace ct {
namespace optcon {

/**
 * \ingroup Constraint
 *+
 * \brief A class fusing analytical constraint terms and Autodiff constraint terms
 *
 * If analytical Jacobians are provided, add the implemented terms as analytical terms.
 * Otherwise overload the term's scalar types with CppAD::AD<double> types and let
 * this class derive the Jacobians automatically.
 *
 * Note: we need to know the constraint dimensions already at object construction time
 *
 * @tparam STATE_DIM: Dimension of the state vector
 * @tparam INPUT_DIM: Dimension of the control input vector
 * @tparam CONSTRAINT2_DIM: Maximum number of pure state constraints
 * @tparam CONSTRAINT1_DIM: Maximum number of state-input constraints
 */


 // blabla decide on the parameters
template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintContainerAnalytical : public LinearConstraintContainer<STATE_DIM, INPUT_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::VectorXd VectorXd;
	typedef Eigen::VectorXi VectorXi;
	typedef Eigen::MatrixXd MatrixXd;

	typedef core::StateVector<STATE_DIM>   state_vector_t;
	typedef core::ControlVector<INPUT_DIM> input_vector_t;

	typedef ConstraintBase<STATE_DIM, INPUT_DIM> Base;
	typedef std::shared_ptr<ConstraintBase<STATE_DIM, INPUT_DIM>> ConstraintBase_Shared_Ptr_t;

	typedef ConstraintContainerAnalytical<STATE_DIM, INPUT_DIM>* ConstraintContainerAnalytical_Raw_Ptr_t;

	enum TERM_TYPE {
		AD = 0,   //!< AD
		ANALYTICAL = 1//!< ANALYTICAL
	};

	enum MAP_DATA {
		START_INDEX = 0,//!< START_INDEX
		TERM_DIM,       //!< TERM_DIM
		TYPE,           //!< TYPE
		TERM_ID         //!< TERM_ID
	};

	typedef std::vector<std::tuple<size_t, size_t, TERM_TYPE, size_t>> MAP;


	ConstraintContainerAnalytical()
	:
	initialized_(false)
	{}

	/**
	 * \brief Constructor using state, control and time
	 * @param x state vector
	 * @param u control vector
	 * @param t time
	 */
	ConstraintContainerAnalytical(const state_vector_t &x, const input_vector_t &u, const double& t = 0.0)
	{
		initialized_ = false;
	}

	/**
	 * \brief Copy constructor
	 * @param arg constraint class to copy
	 */
	ConstraintContainerAnalytical(const ConstraintContainerAnalytical& arg) 
	:
	constraintsMap_(arg.constraintsMap_),
	eval_(arg.eval_),
	evalJacSparse_(arg.evalJacSparse_),
	evalJacDense_(arg.evalJacDense_),
	evalJacStateSparse_(arg.evalJacStateSparse_),
	evalJacStateDense_(arg.evalJacStateDense_),
	evalJacInputSparse_(arg.evalJacInputSparse_),
	evalJacInputDense_(arg.evalJacInputDense_),
	initialized_(arg.initialized_)
	{
		// vectors of terms can be resized easily
		constraintsAnalytical_.resize(arg.constraintsAnalytical_.size());

		for(size_t i = 0; i < constraintsAnalytical_.size(); ++i)
			constraintsAnalytical_[i] = std::shared_ptr< ConstraintBase<STATE_DIM, INPUT_DIM>> (arg.constraintsAnalytical_[i]->clone());
	}

	virtual ConstraintContainerAnalytical_Raw_Ptr_t clone () const override {return new ConstraintContainerAnalytical(*this);}

	void addConstraint(std::shared_ptr<ConstraintBase<STATE_DIM, INPUT_DIM>> constraint, bool verbose)
	{
		if(verbose){
			std::string name;
			constraint->getName(name);
			std::cout<<"''" << name << "'' added as Analytical state input constraint ";
		}

		addConstraintAndGenerateIndicies(constraint, ANALYTICAL, verbose, constraintsAnalytical_, constraintsMap_);
		initialized_ = false;
	}

	size_t getConstraintsCount()
	{
		size_t count = 0;
		for(auto constraint : constraintsAnalytical_)
			count += constraint->getConstraintsCount();
		return count;
	}

	void initialize()
	{
		size_t nnzStateCount = 0;
		size_t nnzInputCount = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			const size_t nonZerosState = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianState();
			const size_t nonZerosInput = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianInput();

			eval_.conservativeResize(start_index + constraint_dim);
			evalJacSparse_.conservativeResize(nnzStateCount + nnzInputCount + nonZerosState + nonZerosInput);
			evalJacDense_.conservativeResize(start_index + constraint_dim, STATE_DIM + INPUT_DIM);
			evalJacStateSparse_.conservativeResize(nnzStateCount + nonZerosState);
			evalJacStateDense_.conservativeResize(start_index + constraint_dim, STATE_DIM);
			evalJacInputSparse_.conservativeResize(nnzInputCount + nonZerosInput);
			evalJacInputDense_.conservativeResize(start_index + constraint_dim, INPUT_DIM);
			nnzStateCount += nonZerosState;
			nnzInputCount += nonZerosInput;
		}
		initialized_ = true;
	}

	virtual void evaluate(VectorXd& g, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		count = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			eval_.segment(start_index, constraint_dim) = constraintsAnalytical_[constraint_id]->evaluate();
			count += constraint_dim;
		}
		g = eval_;		
	}

	virtual void getLowerBound(VectorXd& lb, size_t& count) override
	{
		count = 0;
		VectorXd lbLocal;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);

			// evaluate analytical constraint
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			lbLocal.conservativeResize(start_index + constraint_dim);
			lbLocal.segment(start_index, constraint_dim) = constraintsAnalytical_[constraint_id]->getLowerBound();
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
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			ubLocal.conservativeResize(start_index + constraint_dim);
			ubLocal.segment(start_index, constraint_dim) = constraintsAnalytical_[constraint_id]->getUpperBound();
			count += constraint_dim;
		}
		ub = ubLocal;
	}

	virtual void evalJacSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		count = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);

			const size_t nonZerosState = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianState();
			const size_t nonZerosInput = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianInput();

			if(nonZerosState != 0)
			{
				evalJacSparse_.segment(count, nonZerosState) = constraintsAnalytical_[constraint_id]->jacobianStateSparse();
				count += nonZerosState;
			}
			if(nonZerosInput != 0)
			{
				evalJacSparse_.segment(count, nonZerosInput) = constraintsAnalytical_[constraint_id]->jacobianInputSparse();
				count += nonZerosInput;				
			}
		}
		jacVec = evalJacSparse_;
	}

	virtual Eigen::MatrixXd evalJacDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		Eigen::MatrixXd jacLocal;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();

			evalJacDense_.block(start_index, 0, constraint_dim, STATE_DIM) = constraintsAnalytical_[constraint_id]->JacobianState();
			evalJacDense_.block(start_index, STATE_DIM, constraint_dim, INPUT_DIM) = constraintsAnalytical_[constraint_id]->JacobianInput();			
		}

		return evalJacDense_;
	}

	virtual void evalJacStateSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		count = 0;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			// evaluate analytical jacobian
			const size_t sparseJacDim = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianState();
			evalJacStateSparse_.segment(count, sparseJacDim) = constraintsAnalytical_[constraint_id]->jacobianStateSparse();
			count += sparseJacDim;
		}
		jacVec = evalJacStateSparse_;
	}

	virtual Eigen::MatrixXd evalJacStateDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		size_t count = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			count += constraint_dim;
			evalJacStateDense_.block(start_index, 0, constraint_dim, STATE_DIM) = constraintsAnalytical_[constraint_id]->JacobianState();
		}

		return evalJacStateDense_;		
	}

	virtual void evalJacInputSparse(VectorXd& jacVec, size_t& count) override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		count = 0;	
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t sparseJacDim = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianInput();
			evalJacInputSparse_.segment(count, sparseJacDim) = constraintsAnalytical_[constraint_id]->jacobianInputSparse();
			count += sparseJacDim;
		}
		jacVec = evalJacInputSparse_;
	}

	virtual Eigen::MatrixXd evalJacInputDense() override
	{
		if(!initialized_)
			throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

		size_t count = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			count += constraint_dim;
			evalJacInputDense_.block(start_index, 0, constraint_dim, INPUT_DIM) = constraintsAnalytical_[constraint_id]->JacobianInput();
		}

		return evalJacInputDense_;		
	}

	virtual size_t generateSparsityPatternJacobianState(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		VectorXi iRowLocal;
		VectorXi jColLocal;
		VectorXi iRowTot;
		VectorXi jColTot;

		size_t count = 0;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);

			size_t nonZerosState = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianState();
			constraintsAnalytical_[constraint_id]->sparsityPatternState(iRowLocal, jColLocal);

			iRowTot.conservativeResize(count + nonZerosState);
			iRowTot.segment(count, nonZerosState) = iRowLocal;

			jColTot.conservativeResize(count + nonZerosState);
			jColTot.segment(count, nonZerosState) = jColLocal;

			count += nonZerosState;
		}

		iRows = iRowTot;
		jCols = jColTot;
		return count;
	}

	virtual size_t generateSparsityPatternJacobianInput(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		VectorXi iRowLocal;
		VectorXi jColLocal;
		VectorXi iRowTot;
		VectorXi jColTot;

		size_t count = 0;

		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);

			const size_t nonZerosInput = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianInput();
			constraintsAnalytical_[constraint_id]->sparsityPatternInput(iRowLocal, jColLocal);

			iRowTot.conservativeResize(count + nonZerosInput);
			iRowTot.segment(count, nonZerosInput) = iRowLocal;

			jColTot.conservativeResize(count + nonZerosInput);
			jColTot.segment(count, nonZerosInput) = jColLocal;

			count += nonZerosInput;
		}

		iRows = iRowTot;
		jCols = jColTot;
		return count;
	}

	virtual size_t generateSparsityPatternJacobian(Eigen::VectorXi& iRows, Eigen::VectorXi& jCols) override
	{
		VectorXi iRowLocal;
		VectorXi jColLocal;
		VectorXi iRowTot;
		VectorXi jColTot;

		size_t nonZeroCount = 0;
		size_t constraintCount = 0;
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			const size_t nonZerosState = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianState();
			const size_t nonZerosInput = constraintsAnalytical_[constraint_id]->getNumNonZerosJacobianInput();

			if(nonZerosState != 0)
			{	
				iRowLocal.resize(nonZerosState);
				jColLocal.resize(nonZerosState);
				constraintsAnalytical_[constraint_id]->sparsityPatternState(iRowLocal, jColLocal);

				iRowTot.conservativeResize(nonZeroCount + nonZerosState);
				iRowTot.segment(nonZeroCount, nonZerosState) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(nonZeroCount + nonZerosState);
				jColTot.segment(nonZeroCount, nonZerosState) = jColLocal;
				nonZeroCount += nonZerosState;				
			}

			if(nonZerosInput != 0)
			{
				iRowLocal.resize(nonZerosInput);
				jColLocal.resize(nonZerosInput);
				constraintsAnalytical_[constraint_id]->sparsityPatternInput(iRowLocal, jColLocal);

				iRowTot.conservativeResize(nonZeroCount + nonZerosInput);
				iRowTot.segment(nonZeroCount, nonZerosInput) = iRowLocal.array() + constraintCount;

				jColTot.conservativeResize(nonZeroCount + nonZerosInput);
				jColTot.segment(nonZeroCount, nonZerosInput) = jColLocal.array() + STATE_DIM;
				nonZeroCount += nonZerosInput;
			}

			if(nonZerosState != 0 || nonZerosInput != 0)
				constraintCount += constraint_dim;

		}

		iRows = iRowTot;
		jCols = jColTot;

		return nonZeroCount;
	}

	virtual void getConstraintTypes(VectorXd& constraint_types) override
	{
		for(auto map : constraintsMap_)
		{
			const size_t constraint_id = std::get<TERM_ID>(map);
			const size_t start_index = std::get<START_INDEX>(map);

			const size_t constraint_dim = constraintsAnalytical_[constraint_id]->getConstraintsCount();
			constraint_types.segment(start_index, constraint_dim) = constraintsAnalytical_[constraint_id]->getConstraintType() * Eigen::VectorXd::Ones(constraint_dim);;
		}
	}

	virtual void setTimeStateInput(const double t, const state_vector_t& x, const input_vector_t& u)  override
	{
		for(auto constraint : constraintsAnalytical_)
		{
			constraint->setTimeStateInputDouble(x, u, t);
			constraint->setTimeStateInputAd(x, u, t);
		}
	}

	virtual size_t getConstraintJacobianNonZeroCount() override
	{
		return getConstraintJacobianStateNonZeroCount() + getConstraintJacobianInputNonZeroCount();
	}

	virtual size_t getConstraintJacobianStateNonZeroCount() override
	{
		size_t count = 0;
		for(auto constraint : constraintsAnalytical_)
		{
			count += constraint->getNumNonZerosJacobianState();
		}
		return count;
	}

	virtual size_t getConstraintJacobianInputNonZeroCount() override
	{
		size_t count = 0;
		for(auto constraint : constraintsAnalytical_)
		{
			count += constraint->getNumNonZerosJacobianInput();
		}
		return count;
	}

	virtual size_t getConstraintCount() override
	{
		size_t count = 0;
		for(auto constraint : constraintsAnalytical_)
		{
			count += constraint->getConstraintsCount();
		}
		return count;
	}


private:
	virtual void update() override {};	// todo: can we make use of that function ?

	void addConstraintAndGenerateIndicies(
		std::shared_ptr< ConstraintBase<STATE_DIM, INPUT_DIM>> constraint, 
		TERM_TYPE constraint_type, 
		bool verbose,
		std::vector<std::shared_ptr< ConstraintBase<STATE_DIM, INPUT_DIM>>>& stockOfConstraints,
		MAP& map)
	{
		stockOfConstraints.push_back(constraint);
		size_t newConstraintId = stockOfConstraints.size()-1;
		size_t constraint_dim = constraint->getConstraintsCount();

		size_t startingIndex = 0;

		// check if map already has value...
		if(map.size()>0)
			startingIndex += (std::get<START_INDEX>(map.back()) + std::get<TERM_DIM>(map.back()));

		auto constraintStartAndDim = std::make_tuple(startingIndex, constraint_dim, constraint_type, newConstraintId);

		map.push_back(constraintStartAndDim);

		if(verbose){
			std::cout << "with starting index " << startingIndex <<", and dimension " << constraint_dim << std::endl;
		}
	}

	std::vector<std::shared_ptr<ConstraintBase<STATE_DIM, INPUT_DIM>>> constraintsAnalytical_;
	MAP constraintsMap_;	/** map containing starting_index, length of constraint sub-vector, constraint type and constraint Id for pure state constraint */
	Eigen::VectorXd eval_;
	VectorXd evalJacSparse_;
	Eigen::MatrixXd evalJacDense_;
	VectorXd evalJacStateSparse_;
	Eigen::MatrixXd evalJacStateDense_;
	VectorXd evalJacInputSparse_;
	Eigen::MatrixXd evalJacInputDense_;
	bool initialized_;

};


}// namespace optcon
}// namespace ct

#endif
