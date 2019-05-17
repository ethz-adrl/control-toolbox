/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#pragma once

#include <ct/optcon/nlp/DiscreteConstraintBase.h>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/ShotContainer.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      Implementation of the DMS continuity constraints
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ContinuityConstraint : public tpl::DiscreteConstraintBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef tpl::DiscreteConstraintBase<SCALAR> BASE;
    typedef DmsDimensions<STATE_DIM, CONTROL_DIM, SCALAR> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;

    typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
    typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
    typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    /**
	 * @brief      Default constructor
	 */
    ContinuityConstraint() = default;
    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  shotContainer  The shot container
	 * @param[in]  w              The optimization variables
	 * @param[in]  shotIndex      The shot number
	 * @param[in]  settings       The dms settings
	 */
    ContinuityConstraint(std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>> shotContainer,
        std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
        size_t shotIndex,
        const DmsSettings settings)
        : shotContainer_(shotContainer), w_(w), shotIndex_(shotIndex), settings_(settings)
    {
        lb_.setConstant(SCALAR(0.0));
        ub_.setConstant(SCALAR(0.0));

        size_t nr = 0;

        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                nr = STATE_DIM * STATE_DIM + STATE_DIM * CONTROL_DIM + STATE_DIM;
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                nr = STATE_DIM * STATE_DIM + STATE_DIM * CONTROL_DIM + STATE_DIM + STATE_DIM * CONTROL_DIM;
                break;
            }
            default:
            {
                throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
            }
        }

        // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        // 	nr += STATE_DIM;
        jacLocal_.resize(nr);
    }


    VectorXs eval() override { return w_->getOptimizedState(shotIndex_ + 1) - shotContainer_->getStateIntegrated(); }
    VectorXs evalSparseJacobian() override
    {
        count_local_ = 0;
        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                computeXblock();  // add the big block (derivative w.r.t. state s_i)
                computeUblock();  // add the smaller block (derivative w.r.t. control q_i)
                computeIblock();  // add the diagonal (derivative w.r.t. state s_(i+1))
                // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
                // 	computeHblock();
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                computeXblock();    // add the big block (derivative w.r.t. state s_i)
                computeUblock();    // add the smaller block (derivative w.r.t. control q_i)
                computeIblock();    // add the diagonal (derivative w.r.t. state s_(i+1))
                computeUblock_2();  // add the smaller block (derivative w.r.t. control q_(i+1))
                // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
                // 	computeHblock();
                break;
            }
            default:
            {
                throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
            }
        }
        return jacLocal_;
    }

    size_t getNumNonZerosJacobian() override
    {
        size_t no = 0;
        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                no = STATE_DIM * STATE_DIM + STATE_DIM * CONTROL_DIM + STATE_DIM * 1;
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                no = STATE_DIM * STATE_DIM + STATE_DIM * CONTROL_DIM + STATE_DIM * 1 + STATE_DIM * CONTROL_DIM;
                break;
            }
            default:
            {
                throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
            }
        }

        /* the derivatives w.r.t. the time optimization variable (h_i) */
        // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        // {
        // 	no += STATE_DIM;
        // }

        return no;
    }


    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        size_t indexNumber = 0;

        switch (settings_.splineType_)
        {
            case DmsSettings::ZERO_ORDER_HOLD:
            {
                // add the big block (derivative w.r.t. state)
                indexNumber += BASE::genBlockIndices(
                    w_->getStateIndex(shotIndex_), STATE_DIM, STATE_DIM, iRow_vec, jCol_vec, indexNumber);

                // add the smaller block (derivative w.r.t. control)
                indexNumber += BASE::genBlockIndices(
                    w_->getControlIndex(shotIndex_), STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);

                // add the diagonal
                indexNumber += BASE::genDiagonalIndices(
                    w_->getStateIndex(shotIndex_ + 1), STATE_DIM, iRow_vec, jCol_vec, indexNumber);
                break;
            }
            case DmsSettings::PIECEWISE_LINEAR:
            {
                // add the big block (derivative w.r.t. state)
                indexNumber += BASE::genBlockIndices(
                    w_->getStateIndex(shotIndex_), STATE_DIM, STATE_DIM, iRow_vec, jCol_vec, indexNumber);

                // add the smaller block (derivative w.r.t. control)
                indexNumber += BASE::genBlockIndices(
                    w_->getControlIndex(shotIndex_), STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);

                // add the diagonal
                indexNumber += BASE::genDiagonalIndices(
                    w_->getStateIndex(shotIndex_ + 1), STATE_DIM, iRow_vec, jCol_vec, indexNumber);

                // add the fourth block (derivative w.r.t. control)
                indexNumber += BASE::genBlockIndices(
                    w_->getControlIndex(shotIndex_ + 1), STATE_DIM, CONTROL_DIM, iRow_vec, jCol_vec, indexNumber);
                break;
            }
            default:
            {
                throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
            }
        }

        /* for the derivatives w.r.t. the time optimization variables (t_i) */
        // if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID)
        // {
        // 	indexNumber += BASE::genBlockIndices(w_->getTimeSegmentIndex(shotIndex_),
        // 		STATE_DIM, 1, iRow_vec, jCol_vec, indexNumber);
        // }
    }

    VectorXs getLowerBound() override { return lb_; }
    VectorXs getUpperBound() override { return ub_; }
    size_t getConstraintSize() override { return STATE_DIM; }
private:
    /**
	 * @brief      Evaluates the sparse jacobian with respect to the discretized
	 *             states s_i
	 */
    void computeXblock();

    /**
	 * @brief      Evaluates the sparse jacobian with respect to the discretized
	 *             inputs q_i
	 */
    void computeUblock();

    /**
	 * @brief      Evaluates the sparse jacobian with respect to the discretized
	 *             inputs q_{i+1}
	 */
    void computeUblock_2();

    /**
	 * @brief      Evaluates an identity matrix in sparse form
	 */
    void computeIblock();

    /**
	 * @brief      Evaluates the sparse jacobian with respect to the discretized
	 *             time segments h_i
	 */
    void computeHblock();

    std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>> shotContainer_;
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w_;
    size_t shotIndex_;
    const DmsSettings settings_;

    VectorXs jacLocal_;
    size_t count_local_;

    state_vector_t lb_;
    state_vector_t ub_;
};


// template <size_t STATE_DIM, size_t CONTROL_DIM>
// void ContinuityConstraint<STATE_DIM, CONTROL_DIM>::computeHblock()
// {
// 	// take last elements of state and time
// 	state_vector_t state = shotContainer_->getStateIntegrated();
// 	ct::core::Time time = shotContainer_->getIntegrationTimeFinal();

// 	// compute derivative
// 	state_vector_t mat;
// 	state_vector_t dynamics;
// 	shotContainer_->getControlledSystemPtr()->computeDynamics(state, time, dynamics);

// 	switch (settings_.splineType_)
// 	{
// 		case DmsSettings::ZERO_ORDER_HOLD:
// 		{
// 			mat = - dynamics;
// 			break;
// 		}
// 		case DmsSettings::PIECEWISE_LINEAR:
// 		{
// 			mat = -dynamics - shotContainer_->getdXdHiIntegrated();
// 			break;
// 		}
// 		default:
// 		{
// 			throw(std::runtime_error("specified invalid spliner type in ContinuityConstraint-class"));
// 		}
// 	}

// 	jacLocal_.segment(count_local_, STATE_DIM) = mat;
// 	count_local_ += STATE_DIM;
// }


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::computeXblock()
{
    state_matrix_t mat = -shotContainer_->getdXdSiIntegrated();
    mat.transposeInPlace();
    VectorXs dXdSiVec = (Eigen::Map<VectorXs>(mat.data(), STATE_DIM * STATE_DIM));

    // fill into value vector with correct indexing
    jacLocal_.segment(count_local_, STATE_DIM * STATE_DIM) = dXdSiVec;

    count_local_ += STATE_DIM * STATE_DIM;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::computeUblock()
{
    MatrixXs mat = -shotContainer_->getdXdQiIntegrated();
    mat.transposeInPlace();
    VectorXs dXdQiVec = Eigen::Map<VectorXs>(mat.data(), STATE_DIM * CONTROL_DIM);

    // // fill into value vector with correct indexing
    jacLocal_.segment(count_local_, STATE_DIM * CONTROL_DIM) = dXdQiVec;
    count_local_ += STATE_DIM * CONTROL_DIM;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::computeUblock_2()
{
    MatrixXs mat = -shotContainer_->getdXdQip1Integrated();
    mat.transposeInPlace();
    VectorXs dXdU1Vec = Eigen::Map<VectorXs>(mat.data(), STATE_DIM * CONTROL_DIM);

    // fill into value vector with correct indexing
    jacLocal_.segment(count_local_, STATE_DIM * CONTROL_DIM) = dXdU1Vec;
    count_local_ += STATE_DIM * CONTROL_DIM;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>::computeIblock()
{
    // fill into value vector with correct indexing
    jacLocal_.segment(count_local_, STATE_DIM) = VectorXs::Ones(STATE_DIM);
    count_local_ += STATE_DIM;
}

}  // namespace optcon
}  // namespace ct
