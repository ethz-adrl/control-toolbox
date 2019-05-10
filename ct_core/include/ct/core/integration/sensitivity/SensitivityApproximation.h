/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <unsupported/Eigen/MatrixFunctions>

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P> \
    typename std::enable_if<(V > 0 && P > 0), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P> \
    typename std::enable_if<(V <= 0 || P <= 0), void>::type

namespace ct {
namespace core {

//! interface class for a general linear system or linearized system
/*!
 * Defines the interface for a linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double>
class SensitivityApproximation : public Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type


    //! constructor
    SensitivityApproximation(const SCALAR& dt,
        const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem = nullptr,
        const SensitivityApproximationSettings::APPROXIMATION& approx =
            SensitivityApproximationSettings::APPROXIMATION::FORWARD_EULER)
        : linearSystem_(linearSystem), settings_(dt, approx)
    {
    }


    //! constructor
    SensitivityApproximation(const SensitivityApproximationSettings& settings,
        const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem = nullptr)
        : linearSystem_(linearSystem), settings_(settings)
    {
    }


    //! copy constructor
    SensitivityApproximation(const SensitivityApproximation& other) : settings_(other.settings_)
    {
        if (other.linearSystem_ != nullptr)
            linearSystem_ = std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>(other.linearSystem_->clone());
    }


    //! destructor
    virtual ~SensitivityApproximation(){};


    //! deep cloning
    virtual SensitivityApproximation<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>* clone() const override
    {
        return new SensitivityApproximation<STATE_DIM, CONTROL_DIM, P_DIM, V_DIM, SCALAR>(*this);
    }


    //! update the approximation type for the discrete-time system
    virtual void setApproximation(const SensitivityApproximationSettings::APPROXIMATION& approx) override
    {
        settings_.approximation_ = approx;
    }


    //! retrieve the approximation type for the discrete-time system
    SensitivityApproximationSettings::APPROXIMATION getApproximation() const { return settings_.approximation_; }
    //! update the linear system
    virtual void setLinearSystem(
        const std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem) override
    {
        linearSystem_ = linearSystem;
    }


    //! update the time discretization
    virtual void setTimeDiscretization(const SCALAR& dt) override { settings_.dt_ = dt; }
    //! update the settings
    void updateSettings(const SensitivityApproximationSettings& settings) { settings_ = settings; }
    //! get A and B matrix for linear time invariant system
    /*!
	 * compute discrete-time linear system matrices A and B
	 * @param x	the state setpoint
	 * @param u the control setpoint
	 * @param n the time setpoint
	 * @param numSteps number of timesteps of trajectory for which to get the sensitivity for
	 * @param A the resulting linear system matrix A
	 * @param B the resulting linear system matrix B
	 */
    virtual void getAandB(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const StateVector<STATE_DIM, SCALAR>& x_next,
        const int n,
        const size_t numSteps,
        state_matrix_t& A,
        state_control_matrix_t& B) override
    {
        if (linearSystem_ == nullptr)
            throw std::runtime_error("Error in SensitivityApproximation: linearSystem not properly set.");

        /*!
		 * for an LTI system A and B won't change with time n, hence the linearizations result from the following LTV special case.
		 */
        switch (settings_.approximation_)
        {
            case SensitivityApproximationSettings::APPROXIMATION::FORWARD_EULER:
            {
                forwardEuler(x, u, n, A, B);
                break;
            }
            case SensitivityApproximationSettings::APPROXIMATION::BACKWARD_EULER:
            {
                backwardEuler(x, u, n + 1, A, B);
                break;
            }
            case SensitivityApproximationSettings::APPROXIMATION::SYMPLECTIC_EULER:
            {
                symplecticEuler<V_DIM, P_DIM>(x, u, x_next, n, A, B);
                break;
            }
            case SensitivityApproximationSettings::APPROXIMATION::TUSTIN:
            {
                /*!
                 * the Tustin (also known as 'Heun') approximation uses the state and control at the *start* and at the *end*
                 * of the ZOH interval to generate linear approximations A and B in a trapezoidal fashion.
                 */

                //! continuous-time A and B matrices
                state_matrix_t Ac_front;
                state_control_matrix_t Bc_front;

                // front derivatives
                linearSystem_->getDerivatives(Ac_front, Bc_front, x, u, n * settings_.dt_);
                Ac_front *= settings_.dt_;

                state_matrix_t Ac_back =
                    settings_.dt_ * linearSystem_->getDerivativeState(x_next, u, (n + 1) * settings_.dt_);


                //! tustin approximation
                state_matrix_t aNewInv;
                aNewInv.template topLeftCorner<STATE_DIM, STATE_DIM>() =
                    (state_matrix_t::Identity() - Ac_back).colPivHouseholderQr().inverse();
                A = aNewInv * (state_matrix_t::Identity() + Ac_front);
                B = aNewInv * settings_.dt_ * Bc_front;
                break;
            }
            case SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL:
            {
                matrixExponential(x, u, n, A, B);
                break;
            }
            default:
                throw std::runtime_error("Unknown Approximation type in SensitivityApproximation.");
        }  // end switch
    }


private:
    void forwardEuler(const StateVector<STATE_DIM, SCALAR>& x_n,
        const ControlVector<CONTROL_DIM, SCALAR>& u_n,
        const int& n,
        state_matrix_t& A_discr,
        state_control_matrix_t& B_discr)
    {
        /*!
		 * the Forward Euler approximation uses the state and control at the *start* of the ZOH interval to
		 * generate linear approximations A and B.
		 */
        state_matrix_t A_cont;
        state_control_matrix_t B_cont;
        linearSystem_->getDerivatives(A_cont, B_cont, x_n, u_n, n * settings_.dt_);

        A_discr = state_matrix_t::Identity() + settings_.dt_ * A_cont;
        B_discr = settings_.dt_ * B_cont;
    }

    void backwardEuler(const StateVector<STATE_DIM, SCALAR>& x_n,
        const ControlVector<CONTROL_DIM, SCALAR>& u_n,
        const int& n,
        state_matrix_t& A_discr,
        state_control_matrix_t& B_discr)
    {
        /*!
		 * the Backward Euler approximation uses the state and control at the *end* of the ZOH interval to
		 * generate linear approximations A and B.
		 */
        state_matrix_t A_cont;
        state_control_matrix_t B_cont;
        linearSystem_->getDerivatives(A_cont, B_cont, x_n, u_n, n * settings_.dt_);

        state_matrix_t aNew = settings_.dt_ * A_cont;
        A_discr.setZero();
        A_discr.template topLeftCorner<STATE_DIM, STATE_DIM>() =
            (state_matrix_t::Identity() - aNew).colPivHouseholderQr().inverse();

        B_discr = A_discr * settings_.dt_ * B_cont;
    }


    void matrixExponential(const StateVector<STATE_DIM, SCALAR>& x_n,
        const ControlVector<CONTROL_DIM, SCALAR>& u_n,
        const int& n,
        state_matrix_t& A_discr,
        state_control_matrix_t& B_discr)
    {
        state_matrix_t Ac;
        state_control_matrix_t Bc;
        linearSystem_->getDerivatives(Ac, Bc, x_n, u_n, n * settings_.dt_);

        state_matrix_t Adt = settings_.dt_ * Ac;

        A_discr.template topLeftCorner<STATE_DIM, STATE_DIM>() = Adt.exp();
        B_discr.template topLeftCorner<STATE_DIM, CONTROL_DIM>() =
            Ac.inverse() * (A_discr - state_matrix_t::Identity()) * Bc;
    }


    //!get the discretized linear system Ax+Bu corresponding for a symplectic integrator with full parameterization
    /*!
	 * @param x	state at start of interval
	 * @param u control at start of interval
	 * @param x_next state at end of interval
	 * @param u_next control at end of interval
	 * @param n time index
	 * @param A_sym resulting symplectic discrete-time A matrix
	 * @param B_sym A_sym resulting symplectic discrete-time B matrix
	 */
    SYMPLECTIC_ENABLED symplecticEuler(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const StateVector<STATE_DIM, SCALAR>& x_next,
        const int& n,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        const SCALAR& dt = settings_.dt_;

        // our implementation of symplectic integrators first updates the positions, we need to reconstruct an intermediate state accordingly
        StateVector<STATE_DIM, SCALAR> x_interm = x;
        x_interm.topRows(P_DIM) = x_next.topRows(P_DIM);

        state_matrix_t Ac1;          // continuous time A matrix for start state and control
        state_control_matrix_t Bc1;  // continuous time B matrix for start state and control
        linearSystem_->getDerivatives(Ac1, Bc1, x, u, n * dt);

        state_matrix_t Ac2;          // continuous time A matrix for intermediate state and control
        state_control_matrix_t Bc2;  // continuous time B matrix for intermediate state and control
        linearSystem_->getDerivatives(Ac2, Bc2, x_interm, u, n * dt);

        getSymplecticEulerApproximation<V_DIM, P_DIM>(Ac1, Ac2, Bc1, Bc2, A_sym, B_sym);
    }


    //!get the discretized linear system Ax+Bu corresponding for a symplectic integrator with reduced
    /*!
	 * version without intermediate state. In this method, we do not consider the updated intermediate state from the symplectic integration step
	 * and compute the discretized A and B matrix using the continuous-time linearization at the starting state and control only.
	 * \note this approximation is less costly but not 100% correct
	 * @param x	state at start of interval
	 * @param u control at start of interval
	 * @param n time index
	 * @param A_sym resulting symplectic discrete-time A matrix
	 * @param B_sym A_sym resulting symplectic discrete-time B matrix
	 */
    SYMPLECTIC_ENABLED symplecticEuler(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const int& n,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        const SCALAR& dt = settings_.dt_;

        state_matrix_t Ac1;          // continuous time A matrix for start state and control
        state_control_matrix_t Bc1;  // continuous time B matrix for start state and control
        linearSystem_->getDerivatives(Ac1, Bc1, x, u, n * dt);

        getSymplecticEulerApproximation<V_DIM, P_DIM>(Ac1, Ac1, Bc1, Bc1, A_sym, B_sym);
    }


    //! performs the symplectic Euler approximation
    /*!
	 * @param Ac1	continuous-time A matrix at start state and control
	 * @param Ac2	continuous-time A matrix at intermediate-step state and start control
	 * @param Bc1	continuous-time B matrix at start state and control
	 * @param Bc2	continuous-time B matrix at intermediate-step state and start control
	 * @param A_sym	resulting discrete-time symplectic A matrix
	 * @param B_sym resulting discrete-time symplectic B matrix
	 */
    SYMPLECTIC_ENABLED getSymplecticEulerApproximation(const state_matrix_t& Ac1,
        const state_matrix_t& Ac2,
        const state_control_matrix_t& Bc1,
        const state_control_matrix_t& Bc2,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        const SCALAR& dt = settings_.dt_;

        typedef Eigen::Matrix<SCALAR, P_DIM, P_DIM> p_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, V_DIM> v_matrix_t;
        typedef Eigen::Matrix<SCALAR, P_DIM, V_DIM> p_v_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, P_DIM> v_p_matrix_t;
        typedef Eigen::Matrix<SCALAR, P_DIM, CONTROL_DIM> p_control_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, CONTROL_DIM> v_control_matrix_t;

        // for ease of notation, make a block-wise map to references
        // elements taken form the linearization at the starting state
        const Eigen::Ref<const p_matrix_t> A11 = Ac1.topLeftCorner(P_DIM, P_DIM);
        const Eigen::Ref<const p_v_matrix_t> A12 = Ac1.topRightCorner(P_DIM, V_DIM);
        const Eigen::Ref<const p_control_matrix_t> B1 = Bc1.topRows(P_DIM);

        // elements taken from the linearization at the intermediate state
        const Eigen::Ref<const v_p_matrix_t> A21 = Ac2.bottomLeftCorner(V_DIM, P_DIM);
        const Eigen::Ref<const v_matrix_t> A22 = Ac2.bottomRightCorner(V_DIM, V_DIM);
        const Eigen::Ref<const v_control_matrix_t> B2 = Bc2.bottomRows(V_DIM);

        // discrete approximation A matrix
        A_sym.topLeftCorner(P_DIM, P_DIM) = p_matrix_t::Identity() + dt * A11;
        A_sym.topRightCorner(P_DIM, V_DIM) = dt * A12;
        A_sym.bottomLeftCorner(V_DIM, P_DIM) = dt * (A21 * (p_matrix_t::Identity() + dt * A11));
        A_sym.bottomRightCorner(V_DIM, V_DIM) = v_matrix_t::Identity() + dt * (A22 + dt * A21 * A12);

        // discrete approximation B matrix
        B_sym.topRows(P_DIM) = dt * B1;
        B_sym.bottomRows(V_DIM) = dt * (B2 + dt * A21 * B1);
    }


    //! gets instantiated in case the system is not symplectic
    SYMPLECTIC_DISABLED symplecticEuler(const StateVector<STATE_DIM, SCALAR>& x_n,
        const ControlVector<CONTROL_DIM, SCALAR>& u_n,
        const StateVector<STATE_DIM, SCALAR>& x_next,
        const int& n,
        state_matrix_t& A,
        state_control_matrix_t& B)
    {
        throw std::runtime_error("SensitivityApproximation : selected symplecticEuler but System is not symplectic.");
    }

    //! gets instantiated in case the system is not symplectic
    SYMPLECTIC_DISABLED symplecticEuler(const StateVector<STATE_DIM, SCALAR>& x,
        const ControlVector<CONTROL_DIM, SCALAR>& u,
        const int& n,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        throw std::runtime_error("SensitivityApproximation : selected symplecticEuler but System is not symplectic.");
    }

    SYMPLECTIC_DISABLED getSymplecticEulerApproximation(const state_matrix_t& Ac1,
        const state_matrix_t& Ac2,
        const state_control_matrix_t& Bc1,
        const state_control_matrix_t& Bc2,
        state_matrix_t& A_sym,
        state_control_matrix_t B_sym)
    {
        throw std::runtime_error("SensitivityApproximation : selected symplecticEuler but System is not symplectic.");
    }

    //! shared_ptr to a continuous time linear system (system to be discretized)
    std::shared_ptr<LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem_;

    //! discretization settings
    SensitivityApproximationSettings settings_;
};


}  // namespace core
}  // namespace ct


#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
