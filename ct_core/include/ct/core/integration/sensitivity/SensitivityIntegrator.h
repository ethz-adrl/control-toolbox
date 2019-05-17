/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include <ct/core/systems/continuous_time/linear/LinearSystem.h>
#include <ct/core/integration/internal/SteppersCT.h>

#define SYMPLECTIC_ENABLED        \
    template <size_t V, size_t P> \
    typename std::enable_if<(V > 0 && P > 0), void>::type
#define SYMPLECTIC_DISABLED       \
    template <size_t V, size_t P> \
    typename std::enable_if<(V <= 0 || P <= 0), void>::type

namespace ct {
namespace core {


/**
 * @brief      This class can integrate a controlled system
 *             Furthermore, it provides first order derivatives with respect to
 *             initial state and control
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 * @tparam     SCALAR       The scalar type
 */
template <size_t STATE_DIM,
    size_t CONTROL_DIM,
    size_t P_DIM = STATE_DIM / 2,
    size_t V_DIM = STATE_DIM / 2,
    typename SCALAR = double>
class SensitivityIntegrator : public Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef StateMatrix<STATE_DIM, SCALAR> state_matrix_t;                              //!< state Jacobian type
    typedef ControlMatrix<CONTROL_DIM, SCALAR> control_matrix_t;                        //!< state Jacobian type
    typedef StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> state_control_matrix_t;  //!< input Jacobian type

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM + CONTROL_DIM> sensitivities_matrix_t;


    /**
     * @brief      Constructor
     *
     * @param[in]  system       The controlled system
     * @param[in]  stepperType  The integration stepper type
     */
    SensitivityIntegrator(const SCALAR dt,
        const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem,
        const std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>>& controller,
        const ct::core::IntegrationType stepperType = ct::core::IntegrationType::EULERCT,
        bool timeVarying = true)
        : timeVarying_(timeVarying), symplectic_(false), dt_(dt), substep_(0), k_(0), controller_(controller)
    {
        setLinearSystem(linearSystem);
        setStepper(stepperType);
        initStepper();
    }


    /**
     * @brief      Destroys the object.
     */
    ~SensitivityIntegrator() override = default;
    /**
     * @brief      Initializes the steppers
     *
     * @param[in]  stepperType  The desired integration stepper type
     */
    void setStepper(const ct::core::IntegrationType stepperType)
    {
        symplectic_ = false;

        switch (stepperType)
        {
            case ct::core::IntegrationType::EULERCT:
            case ct::core::IntegrationType::EULER_SYM:
            case ct::core::IntegrationType::EULER:
            {
                stepper_ = std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>>(
                    new ct::core::internal::StepperEulerCT<sensitivities_matrix_t, SCALAR>());
                break;
            }

            case ct::core::IntegrationType::RK4:
            case ct::core::IntegrationType::RK4CT:
            {
                stepper_ = std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>>(
                    new ct::core::internal::StepperRK4CT<sensitivities_matrix_t, SCALAR>());
                break;
            }

            default:
                throw std::runtime_error("Integration type not supported by sensitivity integrator");
        }

        if (stepperType == ct::core::IntegrationType::EULER_SYM)
            symplectic_ = true;
    }

    /**
     * @brief      Prepares the integrator to provide first order sensitivity
     *             generation by setting a linearsystem, enabling state caching
     *             and settings up the function objects
     *
     * @param[in]  linearSystem  The linearized system
     */
    virtual void setLinearSystem(
        const std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>>& linearSystem) override
    {
        linearSystem_ = linearSystem;
    }

    //! update the time discretization
    virtual void setTimeDiscretization(const SCALAR& dt) override { dt_ = dt; }
    /**
     * @brief          Integrates the sensitivity ODE of the integrator with
     *                 respect to the initial state x0
     *
     * @param[in]      startTime  The start time
     * @param[in]      numSteps   The number of integration steps
     * @param[out]     A          Sensitivity with respect to state
     * @param[out]     B          Sensitivity with respect to control
     */
    void integrateSensitivity(const size_t k, const size_t numSteps, state_matrix_t& A, state_control_matrix_t& B)
    {
        A.setIdentity();
        B.setZero();

        sensitivities_matrix_t AB;
        AB << A, B;

        k_ = k;
        substep_ = 0;

        for (size_t i = 0; i < numSteps; ++i)
        {
            stepper_->do_step(dFdxDot_, AB, k * dt_, dt_);
        }

        A = AB.template leftCols<STATE_DIM>();
        B = AB.template rightCols<CONTROL_DIM>();
    }

    /*!
	 * retrieve discrete-time linear system matrices A and B.
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
#ifdef DEBUG
        if (!(this->xSubstep_) || !(this->uSubstep_))
            throw std::runtime_error("SensitivityIntegrator.h: Cached trajectories not set.");
        if (this->xSubstep_->size() <= n || this->uSubstep_->size() <= n)
        {
            std::cout << "length x: " << this->xSubstep_->size() << std::endl;
            std::cout << "length u: " << this->uSubstep_->size() << std::endl;
            std::cout << "n: " << n << std::endl;
            throw std::runtime_error("SensitivityIntegrator.h: Cached trajectories too short.");
        }
        if (!this->xSubstep_->at(n))
            throw std::runtime_error("no state substeps available for requested time index");
        if (!this->uSubstep_->at(n))
            throw std::runtime_error("no control substeps available for requested time index");

        if (this->xSubstep_->at(n)->size() == 0)
            throw std::runtime_error("substep state trajectory length for given time index is zero");
        if (this->uSubstep_->at(n)->size() == 0)
            throw std::runtime_error("substep control trajectory length for given time index is zero");

        assert(x == this->xSubstep_->operator[](n)->operator[](0) && "cached trajectory does not match provided state");
        assert(u == this->uSubstep_->operator[](n)->operator[](0) && "cached trajectory does not match provided input");
#endif

        x_next_ = x_next;

        if (!timeVarying_)
        {
            Aconst_ = linearSystem_->getDerivativeState(x, u, n * dt_);
            Bconst_ = linearSystem_->getDerivativeControl(x, u, n * dt_);
        }

        integrateSensitivity(n, numSteps, A, B);
    };


private:
    bool timeVarying_;
    bool symplectic_;
    double dt_;
    size_t substep_;
    size_t k_;

    state_vector_t x_next_;

    state_matrix_t Aconst_;
    state_control_matrix_t Bconst_;

    std::shared_ptr<ct::core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>> linearSystem_;
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;

    // Sensitivities
    std::function<void(const sensitivities_matrix_t&, sensitivities_matrix_t&, const SCALAR)> dFdxDot_;

    std::shared_ptr<ct::core::internal::StepperCTBase<sensitivities_matrix_t, SCALAR>> stepper_;


    inline void integrateSensitivities(const state_matrix_t& A,
        const state_control_matrix_t& B,
        const state_vector_t& x,
        const sensitivities_matrix_t& dX0In,
        sensitivities_matrix_t& dX0dt,
        const SCALAR t)
    {
        dX0dt.template leftCols<STATE_DIM>() = A * dX0In.template leftCols<STATE_DIM>();
        dX0dt.template rightCols<CONTROL_DIM>() =
            A * dX0In.template rightCols<CONTROL_DIM>() + B * controller_->getDerivativeU0(x, t);
    }

    void initStepper()
    {
        dFdxDot_ = [this](const sensitivities_matrix_t& dX0In, sensitivities_matrix_t& dX0dt, const SCALAR t) {
#ifdef DEBUG
            if (!this->xSubstep_ || this->xSubstep_->size() <= this->k_)
                throw std::runtime_error("substeps not correctly initialized");
#endif

            const typename Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>::StateVectorArrayPtr& xSubstep =
                this->xSubstep_->operator[](this->k_);
            const typename Sensitivity<STATE_DIM, CONTROL_DIM, SCALAR>::ControlVectorArrayPtr& uSubstep =
                this->uSubstep_->operator[](this->k_);

#ifdef DEBUG

            if (!xSubstep || xSubstep->size() <= this->substep_)
            {
                throw std::runtime_error("substeps not correctly initialized");
            }
#endif
            const state_vector_t& x = xSubstep->operator[](this->substep_);
            const control_vector_t& u = uSubstep->operator[](this->substep_);

            if (symplectic_)
            {
                state_matrix_t A_sym;
                state_control_matrix_t B_sym;
                const state_vector_t* x_next;

                if (this->substep_ + 1 < xSubstep->size())
                    x_next = &xSubstep->operator[](this->substep_ + 1);
                else
                    x_next = &x_next_;

                getSymplecticAandB<V_DIM, P_DIM>(t, x, *x_next, u, A_sym, B_sym);

                integrateSensitivities(A_sym, B_sym, x, dX0In, dX0dt, t);
            }
            else
            {
                if (timeVarying_)
                {
                    state_matrix_t A = linearSystem_->getDerivativeState(x, u, t);
                    state_control_matrix_t B = linearSystem_->getDerivativeControl(x, u, t);

                    integrateSensitivities(A, B, x, dX0In, dX0dt, t);
                }
                else
                {
                    integrateSensitivities(Aconst_, Bconst_, x, dX0In, dX0dt, t);
                }
            }

            this->substep_++;
        };
    }

    SYMPLECTIC_ENABLED getSymplecticAandB(const SCALAR& t,
        const state_vector_t& x,
        const state_vector_t& x_next,
        const control_vector_t& u,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        // our implementation of symplectic integrators first updates the positions, we need to reconstruct an intermediate state accordingly
        state_vector_t x_interm = x;
        x_interm.template topRows<P_DIM>() = x_next.template topRows<P_DIM>();

        state_matrix_t Ac1 =
            linearSystem_->getDerivativeState(x, u, t);  // continuous time A matrix for start state and control
        state_control_matrix_t Bc1 =
            linearSystem_->getDerivativeControl(x, u, t);  // continuous time B matrix for start state and control
        state_matrix_t Ac2 = linearSystem_->getDerivativeState(
            x_interm, u, t);  // continuous time A matrix for intermediate state and control
        state_control_matrix_t Bc2 = linearSystem_->getDerivativeControl(
            x_interm, u, t);  // continuous time B matrix for intermediate state and control


        typedef Eigen::Matrix<SCALAR, P_DIM, P_DIM> p_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, V_DIM> v_matrix_t;
        typedef Eigen::Matrix<SCALAR, P_DIM, V_DIM> p_v_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, P_DIM> v_p_matrix_t;
        typedef Eigen::Matrix<SCALAR, P_DIM, CONTROL_DIM> p_control_matrix_t;
        typedef Eigen::Matrix<SCALAR, V_DIM, CONTROL_DIM> v_control_matrix_t;

        // for ease of notation, make a block-wise map to references
        // elements taken form the linearization at the starting state
        const Eigen::Ref<const p_matrix_t> A11 = Ac1.template topLeftCorner<P_DIM, P_DIM>();
        const Eigen::Ref<const p_v_matrix_t> A12 = Ac1.template topRightCorner<P_DIM, V_DIM>();
        const Eigen::Ref<const p_control_matrix_t> B1 = Bc1.template topRows<P_DIM>();

        // elements taken from the linearization at the intermediate state
        const Eigen::Ref<const v_p_matrix_t> A21 = Ac2.template bottomLeftCorner<V_DIM, P_DIM>();
        const Eigen::Ref<const v_matrix_t> A22 = Ac2.template bottomRightCorner<V_DIM, V_DIM>();
        const Eigen::Ref<const v_control_matrix_t> B2 = Bc2.template bottomRows<V_DIM>();

        // discrete approximation A matrix
        A_sym.template topLeftCorner<P_DIM, P_DIM>() = A11;
        A_sym.template topRightCorner<P_DIM, V_DIM>() = A12;
        A_sym.template bottomLeftCorner<V_DIM, P_DIM>() = (A21 * (p_matrix_t::Identity() + dt_ * A11));
        A_sym.template bottomRightCorner<V_DIM, V_DIM>() = (A22 + dt_ * A21 * A12);

        // discrete approximation B matrix
        B_sym.template topRows<P_DIM>() = B1;
        B_sym.template bottomRows<V_DIM>() = (B2 + dt_ * A21 * B1);
    }

    SYMPLECTIC_DISABLED getSymplecticAandB(const SCALAR& t,
        const state_vector_t& x,
        const state_vector_t& x_next,
        const control_vector_t& u,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        throw std::runtime_error("Cannot compute sensitivities for symplectic system with these dimensions.");
    }
};
}
}


#undef SYMPLECTIC_ENABLED
#undef SYMPLECTIC_DISABLED
