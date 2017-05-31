/*
 * ContinuityConstraint.hpp
 *
 * Created: 16.12.2015
 * Author: mgiftthaler
 * *
 */


#ifndef DMS_CONTINUITY_CONSTRAINT_HPP_
#define DMS_CONTINUITY_CONSTRAINT_HPP_

#include <ct/optcon/nlp/DiscreteConstraintBase.h>

#include <ct/optcon/dms/dms_core/OptVectorDms.hpp>
#include <ct/optcon/dms/dms_core/DmsDimensions.hpp>
#include <ct/optcon/dms/dms_core/ShotContainer.hpp>

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM>
class ContinuityConstraint : public DiscreteConstraintBase
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DiscreteConstraintBase BASE;
	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::time_array_t time_array_t;

	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

	ContinuityConstraint() {}


	ContinuityConstraint(
			std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>> shotContainer,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			size_t shotIndex,
			const DmsSettings settings
			)
	:
		shotContainer_(shotContainer),
		w_(w),
		shotIndex_(shotIndex),
		settings_(settings)
	{
		lb_.setConstant(0.0);
		ub_.setConstant(0.0);

		size_t nr = 0;
		
		switch (settings_.splineType_)
		{
			case DmsSettings::ZERO_ORDER_HOLD:
			{
				nr = STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM + STATE_DIM;
				break;
			}
			case DmsSettings::PIECEWISE_LINEAR:
			{
				nr = STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM + STATE_DIM + STATE_DIM*CONTROL_DIM;
				break;
			}
			default:
			{
				throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
			}
		}

		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
			nr += STATE_DIM;
		jacLocal_.resize(nr);
	}


	virtual Eigen::VectorXd eval() override
	{
		stateNext_ = shotContainer_->getStateIntegrated();
		assert(stateNext_ == stateNext_);
		assert(w_->getOptimizedState(shotIndex_+1) == w_->getOptimizedState(shotIndex_+1));
		return w_->getOptimizedState(shotIndex_+1) - stateNext_;
	}

	virtual Eigen::VectorXd evalJacobian() override
	{
		count_local_ = 0;
		switch (settings_.splineType_)
		{
			case DmsSettings::ZERO_ORDER_HOLD:
			{
				computeXblock();			// add the big block (derivative w.r.t. state s_i)
				computeUblock(); 		// add the smaller block (derivative w.r.t. control q_i)
				computeIblock();	// add the diagonal (derivative w.r.t. state s_(i+1))
				if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
					computeHblock();
				break;
			}
			case DmsSettings::PIECEWISE_LINEAR:
			{
				computeXblock();			// add the big block (derivative w.r.t. state s_i)
				computeUblock(); 		// add the smaller block (derivative w.r.t. control q_i)
				computeIblock();	// add the diagonal (derivative w.r.t. state s_(i+1))
				computeUblock_2(); 	// add the smaller block (derivative w.r.t. control q_(i+1))
				if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
					computeHblock();
				break;
			}
			default:
			{
				throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
			}
		}
		return jacLocal_;
	}

	virtual size_t getNumNonZerosJacobian() override
	{
		size_t no = 0;
		switch (settings_.splineType_)
		{
			case DmsSettings::ZERO_ORDER_HOLD:
			{
				no = STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM + STATE_DIM*1;
				break;
			}
			case DmsSettings::PIECEWISE_LINEAR:
			{
				no = STATE_DIM*STATE_DIM + STATE_DIM*CONTROL_DIM + STATE_DIM*1 + STATE_DIM*CONTROL_DIM;
				break;
			}
			default:
			{
				throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
			}
		}

		/* the derivatives w.r.t. the time optimization variable (h_i) */
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
		{
			no += STATE_DIM;
		}

		return no;		
	}


	virtual void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
	{
		size_t indexNumber = 0;

		switch (settings_.splineType_)
		{
			case DmsSettings::ZERO_ORDER_HOLD:
			{
				// add the big block (derivative w.r.t. state)
				indexNumber += BASE::genBlockIndices(w_->getStateIndex(shotIndex_), 
					STATE_DIM, STATE_DIM, iRow_vec, jCol_vec, indexNumber);

				// add the smaller block (derivative w.r.t. control)
				indexNumber += BASE::genBlockIndices(w_->getControlIndex(shotIndex_), 
					STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);

				// add the diagonal
				indexNumber += BASE::genDiagonalIndices(w_->getStateIndex(shotIndex_ + 1), 
					STATE_DIM, iRow_vec, jCol_vec, indexNumber);
				break;
			}
			case DmsSettings::PIECEWISE_LINEAR:
			{
				// add the big block (derivative w.r.t. state)
				indexNumber += BASE::genBlockIndices(w_->getStateIndex(shotIndex_), 
					STATE_DIM, STATE_DIM, iRow_vec, jCol_vec, indexNumber);

				// add the smaller block (derivative w.r.t. control)
				indexNumber += BASE::genBlockIndices(w_->getControlIndex(shotIndex_), 
					STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);

				// add the diagonal
				indexNumber += BASE::genDiagonalIndices(w_->getStateIndex(shotIndex_ + 1), 
					STATE_DIM, iRow_vec, jCol_vec, indexNumber);

				// add the fourth block (derivative w.r.t. control)
				indexNumber += BASE::genBlockIndices(w_->getControlIndex(shotIndex_ + 1), 
					STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);
				break;
			}
			default:
			{
				throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
			}
		}

		/* for the derivatives w.r.t. the time optimization variables (t_i) */
		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
		{
			indexNumber += BASE::genBlockIndices(w_->getTimeSegmentIndex(shotIndex_), 
				STATE_DIM, 1, iRow_vec, jCol_vec, indexNumber);
		}
	}

	virtual Eigen::VectorXd getLowerBound() override
	{
		return lb_;
	}

	virtual Eigen::VectorXd getUpperBound() override
	{
		return ub_;
	}

	virtual size_t getConstraintSize() override
	{
		return STATE_DIM;
	}

private:

	/* the derivatives of the continuity constraints can be written as functions of the linear system dynamics
	 * See notes in order to read about what we define Xblocks, Ublocks and Iblocks.
	 * important: ensure that states and controls are updated everywhere before calling this */
	 
	void computeXblock();
	void computeUblock();
	void computeUblock_2();
	void computeIblock();
	void computeHblock();

	std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM>> shotContainer_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	size_t shotIndex_;
	//The small, local value vector
	Eigen::VectorXd jacLocal_;
	size_t count_local_;
	state_vector_t stateNext_;

	const DmsSettings settings_;
	
	//Constraint bounds
	state_vector_t lb_;	// lower bound
	state_vector_t ub_;	// upper bound	
};


template <size_t STATE_DIM, size_t CONTROL_DIM>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeHblock()
{
	// take last elements of state and time
	state_vector_t state = shotContainer_->getStateIntegrated();
	ct::core::Time time = shotContainer_->getIntegrationTimeFinal();

	// compute derivative
	state_vector_t mat;
	state_vector_t dynamics;
	shotContainer_->getControlledSystemPtr()->computeDynamics(state, time, dynamics);

	switch (settings_.splineType_)
	{
		case DmsSettings::ZERO_ORDER_HOLD:
		{
			mat = - dynamics;
			break;
		}
		case DmsSettings::PIECEWISE_LINEAR:
		{
			mat = -dynamics - shotContainer_->getdXdHiIntegrated();
			break;
		}
		default:
		{
			throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
		}
	}

	jacLocal_.segment(count_local_, STATE_DIM) = mat;
	count_local_ += STATE_DIM;
}



template <size_t STATE_DIM, size_t CONTROL_DIM>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeXblock()
{
	state_matrix_t mat = - shotContainer_->getdXdSiIntegrated();
	mat.transposeInPlace();
	Eigen::VectorXd dXdSiVec = (Eigen::Map<Eigen::VectorXd> (mat.data(), STATE_DIM*STATE_DIM));

	// fill into value vector with correct indexing
	jacLocal_.segment(count_local_, STATE_DIM*STATE_DIM) = dXdSiVec;

	count_local_ += STATE_DIM*STATE_DIM;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeUblock()
{
	Eigen::MatrixXd mat = - shotContainer_->getdXdQiIntegrated();
	mat.transposeInPlace();
	Eigen::VectorXd dXdQiVec = Eigen::Map<Eigen::VectorXd> (mat.data(), STATE_DIM * CONTROL_DIM);

	// // fill into value vector with correct indexing
	jacLocal_.segment(count_local_, STATE_DIM * CONTROL_DIM) = dXdQiVec;
	count_local_ += STATE_DIM*CONTROL_DIM;
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeUblock_2()
{
	Eigen::MatrixXd mat = - shotContainer_->getdXdQip1Integrated();
	mat.transposeInPlace();
	Eigen::VectorXd dXdU1Vec = Eigen::Map<Eigen::VectorXd> (mat.data(), STATE_DIM*CONTROL_DIM);

	// fill into value vector with correct indexing
	jacLocal_.segment(count_local_, STATE_DIM * CONTROL_DIM) = dXdU1Vec;
	count_local_ += STATE_DIM*CONTROL_DIM;
}

template<size_t STATE_DIM, size_t CONTROL_DIM>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeIblock()
{
	// fill into value vector with correct indexing
	jacLocal_.segment(count_local_, STATE_DIM) = Eigen::VectorXd::Ones(STATE_DIM);
	count_local_ += STATE_DIM;
}

} // namespace optcon
} // namespace ct


#endif
