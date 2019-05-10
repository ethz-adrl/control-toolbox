/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cmath>

#include <ct/optcon/dms/dms_core/DmsDimensions.h>
#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/ControllerDms.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      This class implements analytical sensitivity generation for the
 *             euler and rk4 integration scheme
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class RKnDerivatives
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

    typedef typename DIMENSIONS::state_vector_t state_vector_t;
    typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
    typedef typename DIMENSIONS::control_vector_t control_vector_t;
    typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
    typedef typename DIMENSIONS::time_array_t time_array_t;
    typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
    typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
    typedef typename DIMENSIONS::state_control_matrix_t state_control_matrix_t;
    typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
    typedef typename DIMENSIONS::state_control_matrix_array_t state_control_matrix_array_t;

    RKnDerivatives() = delete;

    /**
	 * @brief      Custom constructor
	 *
	 * @param[in]  controlSpliner  The control spliner
	 * @param[in]  shotIdx         The shot number
	 * @param[in]  settings        The dms settings
	 */
    RKnDerivatives(std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner,
        size_t shotIdx,
        const DmsSettings& settings)
        : shotIdx_(shotIdx),
          settings_(settings),
          controlSpliner_(controlSpliner),
          x_log_(nullptr),
          u_log_(nullptr),
          t_log_(nullptr),
          A_log_(nullptr),
          B_log_(nullptr),
          dXdSi_history_(state_matrix_array_t(0)),
          dXdQi_history_(state_control_matrix_array_t(0)),
          dXdQip1_history_(state_control_matrix_array_t(0)),
          dXdHi_history_(state_vector_array_t(0))
    {
    }

    void setLogs(std::shared_ptr<state_vector_array_t> x_log,
        std::shared_ptr<control_vector_array_t> u_log,
        std::shared_ptr<time_array_t> t_log,
        std::shared_ptr<state_matrix_array_t> A_log,
        std::shared_ptr<state_control_matrix_array_t> B_log)
    {
        x_log_ = x_log;
        u_log_ = u_log;
        t_log_ = t_log;
        A_log_ = A_log;
        B_log_ = B_log;
    }


    // compute sensitivity of all X in the trajectory w.r.t. s_i
    void compute_dXdSi()
    {
        dXdSi_history_.clear();
        dXdSi_history_.push_back(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM));

        switch (settings_.integrationType_)
        {
            case DmsSettings::EULER:
            {
                size_t nSteps = A_log_->size() / 1;
                for (size_t i = 0; i < nSteps; i++)
                {
                    double dt_sim = (*t_log_)[i + 1] - (*t_log_)[i];

                    state_matrix_t temp = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) + dt_sim * (*A_log_)[i];

                    dXdSi_history_.push_back(temp * dXdSi_history_.back());
                }
                break;
            }
            case DmsSettings::RK4:
            {
                size_t m = 4;
                size_t nCycles = A_log_->size() / m;

                double a_21 = 0.5;
                double a_32 = 0.5;
                double a_43 = 1.0;

                double b1 = 1.0 / 6.0;
                double b2 = 1.0 / 3.0;
                double b3 = 1.0 / 3.0;
                double b4 = 1.0 / 6.0;

                for (size_t i = 0; i < nCycles; i++)
                {
                    double h = (*t_log_)[(i + 1) * m] - (*t_log_)[i * m];

                    // step 1:
                    state_matrix_t dK1dSi = (*A_log_)[i * m] * dXdSi_history_.back();

                    // step 2:
                    state_matrix_t dK2dSi = (*A_log_)[i * m + 1] * (dXdSi_history_.back() + h * a_21 * dK1dSi);

                    // step 3:
                    state_matrix_t dK3dSi = (*A_log_)[i * m + 2] * (dXdSi_history_.back() + h * a_32 * dK2dSi);

                    // step 4:
                    state_matrix_t dK4dSi = (*A_log_)[i * m + 3] * (dXdSi_history_.back() + h * a_43 * dK3dSi);


                    // summation:
                    state_matrix_t temp =
                        dXdSi_history_.back() + h * (b1 * dK1dSi + b2 * dK2dSi + b3 * dK3dSi + b4 * dK4dSi);

                    dXdSi_history_.push_back(temp);
                }
                break;
            }
            default:
            {
                std::cerr << "... ERROR: sensitivity calculation not implemented for this integration type. Exiting"
                          << std::endl;
                exit(0);
            }
        }
    }


    // compute sensitivity of all X in the trajectory w.r.t. q_i
    void compute_dXdQi()
    {
        dXdQi_history_.clear();
        dXdQi_history_.push_back(state_control_matrix_t::Zero());

        switch (settings_.integrationType_)
        {
            case DmsSettings::EULER:
            {
                for (size_t i = 0; i < A_log_->size(); ++i)
                {
                    control_matrix_t dSpldQ0 = controlSpliner_->splineDerivative_q_i((*t_log_)[i], shotIdx_);

                    double dt_sim = (*t_log_)[i + 1] - (*t_log_)[i];

                    dXdQi_history_.push_back(dXdQi_history_.back() +
                                             dt_sim * ((*A_log_)[i] * dXdQi_history_.back() + (*B_log_)[i] * dSpldQ0));
                }
                break;
            }
            case DmsSettings::RK4:
            {
                size_t m = 4;
                size_t nCycles = A_log_->size() / m;

                double a_21 = 0.5;
                double a_32 = 0.5;
                double a_43 = 1;

                double b1 = 1.0 / 6.0;
                double b2 = 1.0 / 3.0;
                double b3 = 1.0 / 3.0;
                double b4 = 1.0 / 6.0;

                for (size_t i = 0; i < nCycles; i++)
                {
                    assert(i * m < A_log_->size());

                    double h = (*t_log_)[(i + 1) * m] - (*t_log_)[i * m];

                    // TODO: write these steps as computationally more efficient loop if they are correct.
                    // step 1:
                    state_control_matrix_t dK1dQi =
                        (*A_log_)[i * m] * dXdQi_history_.back() +
                        (*B_log_)[i * m] * controlSpliner_->splineDerivative_q_i((*t_log_)[i * m], shotIdx_);

                    // step 2:
                    state_control_matrix_t dK2dQi =
                        (*A_log_)[i * m + 1] * (dXdQi_history_.back() + h * a_21 * dK1dQi) +
                        (*B_log_)[i * m + 1] * controlSpliner_->splineDerivative_q_i((*t_log_)[i * m + 1], shotIdx_);

                    // step 3:
                    state_control_matrix_t dK3dQi =
                        (*A_log_)[i * m + 2] * (dXdQi_history_.back() + h * a_32 * dK2dQi) +
                        (*B_log_)[i * m + 2] * controlSpliner_->splineDerivative_q_i((*t_log_)[i * m + 2], shotIdx_);

                    // step 4:
                    state_control_matrix_t dK4dQi =
                        (*A_log_)[i * m + 3] * (dXdQi_history_.back() + h * a_43 * dK3dQi) +
                        (*B_log_)[i * m + 3] * controlSpliner_->splineDerivative_q_i((*t_log_)[i * m + 3], shotIdx_);


                    // summation
                    state_control_matrix_t temp =
                        dXdQi_history_.back() + h * (b1 * dK1dQi + b2 * dK2dQi + b3 * dK3dQi + b4 * dK4dQi);

                    dXdQi_history_.push_back(temp);
                }
                break;
            }
            default:
            {
                std::cerr << "... ERROR: sensitivity calculation not implemented for this integration type. Exiting"
                          << std::endl;
                exit(0);
            }
        }
    }


    // compute sensitivity of all X in the trajectory w.r.t. q_(i+1)
    void compute_dXdQip1()
    {
        dXdQip1_history_.clear();
        dXdQip1_history_.push_back(state_control_matrix_t::Zero());

        assert(t_log_->size() == A_log_->size() + 1);

        switch (settings_.integrationType_)
        {
            case DmsSettings::EULER:
            {
                for (size_t i = 0; i < A_log_->size(); ++i)
                {
                    control_matrix_t dSpldQ1 = controlSpliner_->splineDerivative_q_iplus1((*t_log_)[i], shotIdx_);

                    assert(dSpldQ1 == dSpldQ1);

                    double dt_sim = (*t_log_)[i + 1] - (*t_log_)[i];

                    dXdQip1_history_.push_back(
                        dXdQip1_history_.back() +
                        dt_sim * ((*A_log_)[i] * dXdQip1_history_.back() + (*B_log_)[i] * dSpldQ1));
                }
                break;
            }
            case DmsSettings::RK4:
            {
                size_t m = 4;
                size_t nCycles = A_log_->size() / m;

                double a_21 = 0.5;
                double a_32 = 0.5;
                double a_43 = 1;

                double b1 = 1 / 6.0;
                double b2 = 1 / 3.0;
                double b3 = 1 / 3.0;
                double b4 = 1 / 6.0;

                for (size_t i = 0; i < nCycles; i++)
                {
                    double h = (*t_log_)[(i + 1) * m] - (*t_log_)[i * m];

                    // TODO: write these steps as computationally more efficient loop if they are correct.
                    // step 1:
                    state_control_matrix_t dK1dQip1 =
                        (*A_log_)[i * m] * dXdQip1_history_.back() +
                        (*B_log_)[i * m] * controlSpliner_->splineDerivative_q_iplus1((*t_log_)[i * m], shotIdx_);

                    // step 2:
                    state_control_matrix_t dK2dQip1 =
                        (*A_log_)[i * m + 1] * (dXdQip1_history_.back() + h * a_21 * dK1dQip1) +
                        (*B_log_)[i * m + 1] *
                            controlSpliner_->splineDerivative_q_iplus1((*t_log_)[i * m + 1], shotIdx_);

                    // step 3:
                    state_control_matrix_t dK3dQip1 =
                        (*A_log_)[i * m + 2] * (dXdQip1_history_.back() + h * a_32 * dK2dQip1) +
                        (*B_log_)[i * m + 2] *
                            controlSpliner_->splineDerivative_q_iplus1((*t_log_)[i * m + 2], shotIdx_);

                    // step 4:
                    state_control_matrix_t dK4dQip1 =
                        (*A_log_)[i * m + 3] * (dXdQip1_history_.back() + h * a_43 * dK3dQip1) +
                        (*B_log_)[i * m + 3] *
                            controlSpliner_->splineDerivative_q_iplus1((*t_log_)[i * m + 3], shotIdx_);


                    // summation
                    state_control_matrix_t temp =
                        dXdQip1_history_.back() + h * (b1 * dK1dQip1 + b2 * dK2dQip1 + b3 * dK3dQip1 + b4 * dK4dQip1);

                    dXdQip1_history_.push_back(temp);
                }
                break;
            }
            default:
            {
                throw(std::runtime_error(
                    "... ERROR: sensitivity calculation not implemented for this integration type."));
            }
        }
    }

    void getdXdSiTraj(state_matrix_array_t& dXdSiTraj) { dXdSiTraj = dXdSi_history_; }
    void getdXdQiTraj(state_control_matrix_array_t& dXdQiTraj) { dXdQiTraj = dXdQi_history_; }
    void getdXdQip1Traj(state_control_matrix_array_t& dXdQip1Traj) { dXdQip1Traj = dXdQip1_history_; }
    void getdXdHiTraj(state_vector_array_t& dXdHiTraj) { dXdHiTraj = dXdHi_history_; }
private:
    std::shared_ptr<SplinerBase<control_vector_t>> controlSpliner_;
    const size_t shotIdx_;
    const DmsSettings settings_;

    std::shared_ptr<state_vector_array_t> x_log_;
    std::shared_ptr<control_vector_array_t> u_log_;
    std::shared_ptr<time_array_t> t_log_;
    std::shared_ptr<state_matrix_array_t> A_log_;
    std::shared_ptr<state_control_matrix_array_t> B_log_;

    state_matrix_array_t dXdSi_history_;
    state_control_matrix_array_t dXdQi_history_;
    state_control_matrix_array_t dXdQip1_history_;
    state_vector_array_t dXdHi_history_;
};

}  // namespace optcon
}  // namespace ct
