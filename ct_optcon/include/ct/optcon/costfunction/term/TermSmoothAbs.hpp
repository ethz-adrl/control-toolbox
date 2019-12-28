/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "TermBase.hpp"

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A smooth absolute term of type
 * \f$ J = a sqrt((x-x_ref)^2 + alpha^2) + b sqrt((u-u_ref)^2 + alpha^2) \f$
 * where this calculation is performed component-wise and summed with individual
 * weighting factors a[i], b[i].
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class TermSmoothAbs : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    TermSmoothAbs(const core::StateVector<STATE_DIM, SCALAR_EVAL> a,
        const core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> b,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref,
        const SCALAR_EVAL alpha = 0.2);

    TermSmoothAbs() = default;

    TermSmoothAbs(const TermSmoothAbs& arg);

    TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>* clone() const override;

    virtual ~TermSmoothAbs();

    SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override;

#ifdef CPPADCG
    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
        ct::core::ADCGScalar t) override;
#endif

    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override;

    void loadConfigFile(const std::string& filename,
        const std::string& termName,
        bool verbose = false) override;  // virtual function for data loading

protected:
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x, const Eigen::Matrix<SC, CONTROL_DIM, 1>& u, const SC& t);

    core::StateVector<STATE_DIM, SCALAR_EVAL> a_;
    core::StateVector<STATE_DIM, SCALAR_EVAL> x_ref_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> b_;
    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> u_ref_;
    SCALAR_EVAL alphaSquared_;
};


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC TermSmoothAbs<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1>& x,
    const Eigen::Matrix<SC, CONTROL_DIM, 1>& u,
    const SC& t)
{
    const Eigen::Matrix<SC, STATE_DIM, 1> xerr2 = (x - x_ref_.template cast<SC>()).array().square();
    const Eigen::Matrix<SC, CONTROL_DIM, 1> uerr2 = (u - u_ref_.template cast<SC>()).array().square();

    const Eigen::Matrix<SC, STATE_DIM, 1> xerrAbs =
        (xerr2 + core::StateVector<STATE_DIM, SC>::Ones() * static_cast<SC>(alphaSquared_)).cwiseSqrt();
    const Eigen::Matrix<SC, CONTROL_DIM, 1> uerrAbs =
        (uerr2 + core::ControlVector<CONTROL_DIM, SC>::Ones() * static_cast<SC>(alphaSquared_)).cwiseSqrt();

    return a_.template cast<SC>().dot(xerrAbs) + b_.template cast<SC>().dot(uerrAbs);
}


}  // namespace optcon
}  // namespace ct
