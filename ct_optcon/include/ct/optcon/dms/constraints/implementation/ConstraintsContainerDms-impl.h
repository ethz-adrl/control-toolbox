/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintsContainerDms(
    std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM, SCALAR>> w,
    std::shared_ptr<tpl::TimeGrid<SCALAR>> timeGrid,
    std::vector<std::shared_ptr<ShotContainer<STATE_DIM, CONTROL_DIM, SCALAR>>> shotContainers,
    std::shared_ptr<ConstraintDiscretizer<STATE_DIM, CONTROL_DIM, SCALAR>> discretizedConstraints,
    const state_vector_t& x0,
    const DmsSettings settings)
    : settings_(settings), shotContainers_(shotContainers)
{
    c_init_ = std::shared_ptr<InitStateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>>(
        new InitStateConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(x0, w));

    this->constraints_.push_back(c_init_);

    for (size_t shotNr = 0; shotNr < settings_.N_; shotNr++)
    {
        std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>> c_i =
            std::shared_ptr<ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>>(
                new ContinuityConstraint<STATE_DIM, CONTROL_DIM, SCALAR>(shotContainers[shotNr], w, shotNr, settings));

        this->constraints_.push_back(c_i);
    }

    if (discretizedConstraints)
    {
        std::cout << "Adding discretized constraints" << std::endl;
        this->constraints_.push_back(discretizedConstraints);
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>::prepareEvaluation()
{
#pragma omp parallel for num_threads(settings_.nThreads_)
    for (auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer)
    {
        (*shotContainer)->integrateShot();
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>::prepareJacobianEvaluation()
{
#pragma omp parallel for num_threads(settings_.nThreads_)
    for (auto shotContainer = shotContainers_.begin(); shotContainer < shotContainers_.end(); ++shotContainer)
    {
        (*shotContainer)->integrateSensitivities();
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintsContainerDms<STATE_DIM, CONTROL_DIM, SCALAR>::changeInitialConstraint(const state_vector_t& x0)
{
    c_init_->updateConstraint(x0);
}
