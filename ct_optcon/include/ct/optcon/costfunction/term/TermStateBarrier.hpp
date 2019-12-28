/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TermBase.hpp"
#include <ct/core/common/activations/Activations.h>
#include <ct/core/internal/traits/TraitSelectorSpecs.h>

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A state barrier term (could also be considered a soft constraint)
 * Note that this term explicitly excludes controls, as there are better
 * ways to limit control effort in a "soft" way, e.g. through the use
 * of sigmoid functions.
 *
 * @todo implement sigmoid barriers
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TermStateBarrier : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> state_vector_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    TermStateBarrier();

    TermStateBarrier(const state_vector_t& ub, const state_vector_t& lb, const state_vector_t& alpha);

    TermStateBarrier(const TermStateBarrier& arg);

    virtual ~TermStateBarrier();

    TermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const override;

    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override;

#ifdef CPPADCG
    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;
#endif

    //! load the term from config file, where the bounds are stored as matrices
    virtual void loadConfigFile(const std::string& filename,
        const std::string& termName,
        bool verbose = false) override;

protected:
    void initialize();

    state_vector_t alpha_;
    state_vector_t ub_;
    state_vector_t lb_;

    std::vector<ct::core::tpl::BarrierActivation<SCALAR, ct::core::tpl::TraitSelector<SCALAR>>> barriers_;
};


}  // namespace optcon
}  // namespace ct
