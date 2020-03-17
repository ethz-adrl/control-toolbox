/**********************************************************************************************************************
This file is part of the Control Toolbox
(https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/types/ControlVector.h>
#include <ct/core/types/StateVector.h>

namespace ct {
namespace core {

//! Discrete Linear time-invariant system
/*!
 * This defines a general discrete linear time-invariant system of the form
 *
 * \f[
 *
 *  \begin{aligned}
 *  x_{n+1} &= Ax_n + Bu_n \\
 *  y_{n+1} &= Cx_n + Du_n
 *  \end{aligned}
 *
 * \f]
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class DiscreteLTISystem : public DiscreteLinearSystem<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructs a discrete linear time invariant system
    /*!
     * @param A A matrix
     * @param B B matrix
     * @param C C matrix
     * @param D D matrix
     * @return instance of the DiscreteLTI system
     */
    DiscreteLTISystem(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& A,
        const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& B,
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& C = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity(),
        const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> D = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>::Zero())
        : A_(A), B_(B), C_(C), D_(D)
    {
    }

    //! copy constructor
    DiscreteLTISystem(const DiscreteLTISystem& arg) : A_(arg.A_), B_(arg.B_), C_(arg.C_), D_(arg.D_) {}
    //! deep clone
    DiscreteLTISystem<STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new DiscreteLTISystem<STATE_DIM, CONTROL_DIM>(*this);
    }
    virtual ~DiscreteLTISystem() {}
    //! get A matrix
    Eigen::Matrix<double, STATE_DIM, STATE_DIM>& A() { return A_; }
    //! get B matrix
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& B() { return B_; }
    //! get C matrix
    Eigen::Matrix<double, STATE_DIM, STATE_DIM>& C() { return C_; }
    //! get D matrix
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& D() { return D_; }
    //! computes the system dynamics
    /*!
     * Computes \f$ x_{n+1} = Ax_n + Bu_n \f$
     * @param state current state x
     * @param t time step (gets ignored)
     * @param control control input
     * @param derivative state derivative
     */
    void propagateControlledDynamics(const Eigen::Matrix<double, STATE_DIM, 1>& state,
        const time_t& n,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& control,
        Eigen::Matrix<double, STATE_DIM, 1>& stateNext)
    {
        stateNext = A_ * state + B_ * control;
    }

    //! computes the system output (measurement)
    /*!
     * Computes \f$ y_{n+1} = Cx_n + Du_n \f$
     * @param state current state x
     * @param t time step (gets ignored)
     * @param control control input
     * @param output system output (measurement)
     */
    void computeOutput(const Eigen::Matrix<double, STATE_DIM, 1>& state,
        const time_t& n,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& control,
        Eigen::Matrix<double, STATE_DIM, 1>& output)
    {
        output = C_ * state + D_ * control;
    }

    //! computes the controllability matrix
    /*!
     * Computes the controllability matrix to assess controllability. See
     * isControllable() for the full test.
     *
     * \todo Move to LinearSystem
     *
     * @param CO controllability matrix
     */
    void computeControllabilityMatrix(Eigen::Matrix<double, STATE_DIM, STATE_DIM * CONTROL_DIM>& CO)
    {
        CO.block<STATE_DIM, CONTROL_DIM>(0, 0) = B_;

        for (size_t i = 1; i < STATE_DIM; i++)
        {
            CO.block<STATE_DIM, CONTROL_DIM>(0, i * CONTROL_DIM) =
                A_ * CO.block<STATE_DIM, CONTROL_DIM>(0, (i - 1) * CONTROL_DIM);
        }
    }

    //! computes the controllability gramian by discretization
    /*!
     * Computes the controllability gramian
     *
     * @return Returns the controllability Gramian
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> computeControllabilityGramian(const size_t max_iters = 100,
        const double tolerance = 1e-9)
    {
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> CG_prev = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> CG = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_prev = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        size_t n_iters = 0;
        while (n_iters < max_iters)
        {
            CG = CG_prev + A_prev * B_ * B_.transpose() * A_prev.transpose();

            // check for convergence using matrix 1-norm
            double norm1 = (CG_prev - CG).template lpNorm<1>();
            if (norm1 < tolerance)
            {
                break;
            }

            // update
            A_prev = A_prev * A_;
            CG_prev = CG;
            ++n_iters;
        }
        return CG;
    }

    //! checks if system is fully controllable
    /*!
     * @return true if fully controllable, false if only partially or
     * non-controllable
     */
    bool isControllable()
    {
        Eigen::Matrix<double, STATE_DIM, STATE_DIM * CONTROL_DIM> CO;
        computeControllabilityMatrix(CO);

        Eigen::FullPivLU<Eigen::Matrix<double, STATE_DIM, STATE_DIM * CONTROL_DIM>> LUdecomposition(CO);
        return LUdecomposition.rank() == STATE_DIM;
    }

    //! computes the observability matrix
    /*!
     * Computes the observability matrix to assess observability. See
     * isObservable() for the full test.
     * @param O observability matrix
     */
    void computeObservabilityMatrix(Eigen::Matrix<double, STATE_DIM, STATE_DIM * STATE_DIM>& O)
    {
        O.block<STATE_DIM, STATE_DIM>(0, 0) = C_;

        for (size_t i = 1; i < STATE_DIM; i++)
        {
            O.block<STATE_DIM, STATE_DIM>(i * STATE_DIM, 0) =
                O.block<STATE_DIM, STATE_DIM>(0, (i - 1) * STATE_DIM) * A_;
        }
    }

    //! computes the observability gramian
    /*!
     * Computes the observability gramian
     *
     * @return Returns the observability Gramian
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> computeObservabilityGramian(const size_t max_iters = 100,
        const double tolerance = 1e-9)
    {
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> OG_prev = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> OG = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_prev = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        size_t n_iters = 0;
        while (n_iters < max_iters)
        {
            OG = OG_prev + A_prev.transpose() * C_.transpose() * C_ * A_prev;

            // check for convergence using matrix 1-norm
            double norm1 = (OG_prev - OG).template lpNorm<1>();
            if (norm1 < tolerance)
            {
                break;
            }

            // update
            A_prev = A_prev * A_;
            OG_prev = OG;
            ++n_iters;
        }
        return OG;
    }

    //! checks if system is fully observable
    /*!
     *
     * @return true if fully observable, false if only partially or non-observable
     */
    bool isObservable()
    {
        Eigen::Matrix<double, STATE_DIM, STATE_DIM * STATE_DIM> O;
        computeObservabilityMatrix(O);

        Eigen::FullPivLU<Eigen::Matrix<double, STATE_DIM, STATE_DIM * STATE_DIM>> LUdecomposition(O);
        return LUdecomposition.rank() == STATE_DIM;
    }

private:
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_;    //!< A matrix
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B_;  //!< B matrix

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> C_;    //!< C matrix
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> D_;  //!< D matrix
};

}  // namespace core
}  // namespace ct

#pragma GCC diagnostic pop