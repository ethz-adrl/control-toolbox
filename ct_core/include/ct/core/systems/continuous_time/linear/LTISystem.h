/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-value"

namespace ct {
namespace core {

//! Linear time-invariant system
/*!
 * This defines a general linear time-invariant system of the form
 *
 * \f[
 *
 *  \begin{aligned}
 *  \dot{x} &= Ax + Bu \\
 *  y &= Cx + Du
 *  \end{aligned}
 *
 * \f]
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of control vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class LTISystem : public LinearSystem<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = LinearSystem<STATE_DIM, CONTROL_DIM>;
    using state_vector_t = typename Base::state_vector_t;
    using control_vector_t = typename Base::control_vector_t;
    using state_matrix_t = typename Base::state_matrix_t;
    using state_control_matrix_t = typename Base::state_control_matrix_t;

    //! Constructs a linear time invariant system
    /*!
	 * @param A A matrix
	 * @param B B matrix
	 * @param C C matrix
	 * @param D D matrix
	 * @return instance of the LTI system
	 */
    LTISystem(const state_matrix_t& A,
        const state_control_matrix_t& B,
        const state_matrix_t& C = state_matrix_t::Identity(),
        const state_control_matrix_t& D = state_control_matrix_t::Zero())
        : A_(A), B_(B), C_(C), D_(D)
    {
    }

    //! copy constructor
    LTISystem(const LTISystem& arg) : A_(arg.A_), B_(arg.B_), C_(arg.C_), D_(arg.D_) {}
    //! deep clone
    LTISystem<STATE_DIM, CONTROL_DIM>* clone() const override { return new LTISystem<STATE_DIM, CONTROL_DIM>(*this); }
    virtual ~LTISystem() {}
    //! get A matrix
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const double t = 0.0) override
    {
        return A_;
    }

    //! get B matrix
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const double t = 0.0) override
    {
        return B_;
    }

    //! get A matrix
    state_matrix_t& A() { return A_; }
    //! get B matrix
    state_control_matrix_t& B() { return B_; }
    //! get C matrix
    state_matrix_t& C() { return C_; }
    //! get D matrix
    state_control_matrix_t& D() { return D_; }
    //! computes the system dynamics
    /*!
	 * Computes \f$ \dot{x} = Ax + Bu \f$
	 * @param state current state x
	 * @param t time (gets ignored)
	 * @param control control input
	 * @param derivative state derivative
	 */
    void computeControlledDynamics(const state_vector_t& state,
        const Time& t,
        const control_vector_t& control,
        state_vector_t& derivative)
    {
        derivative = A_ * state + B_ * control;
    }

    //! computes the system output (measurement)
    /*!
	 * Computes \f$ y = Cx + Du \f$
	 * @param state current state x
	 * @param t time (gets ignored)
	 * @param control control input
	 * @param output system output (measurement)
	 */
    void computeOutput(const state_vector_t& state,
        const Time& t,
        const control_vector_t& control,
        state_vector_t& output)
    {
        output = C_ * state + D_ * control;
    }

    //! computes the controllability matrix
    /*!
	 * Computes the controllability matrix to assess controllability. See isControllable() for the full test.
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

    //! checks if system is fully controllable
    /*!
	 * \todo Move to LinearSystem
	 *
	 * @return true if fully controllable, false if only partially or non-controllable
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
	 * \todo Move to LinearSystem
	 *
	 * Computes the observability matrix to assess observability. See isObservable() for the full test.
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

    //! checks if system is fully observable
    /*!
	 * \todo Move to LinearSystem
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
    state_matrix_t A_;          //!< A matrix
    state_control_matrix_t B_;  //!< B matrix
    state_matrix_t C_;          //!< C matrix
    state_control_matrix_t D_;  //!< D matrix
};

}  // namespace core
}  // namespace ct

#pragma GCC diagnostic pop