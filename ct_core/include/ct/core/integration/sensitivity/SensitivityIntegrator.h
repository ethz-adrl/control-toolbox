/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/integration/internal/StepperEulerCT.h>
#include <ct/core/integration/internal/StepperRK4CT.h>
#include <ct/core/systems/ControlledSystem.h>
#include <ct/core/types/TypeMacros.h>
#include "Sensitivity.h"

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
template <typename MANIFOLD, size_t CONTROL_DIM>
class SensitivityIntegrator : public Sensitivity<MANIFOLD, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t STATE_DIM = MANIFOLD::TangentDim;

    using Base = Sensitivity<MANIFOLD, CONTROL_DIM>;

    using SCALAR = typename Base::SCALAR;
    using Time_t = typename Base::Time_t;
    using control_vector_t = typename Base::control_vector_t;
    using state_matrix_t = typename Base::state_matrix_t;
    using state_control_matrix_t = typename Base::state_control_matrix_t;
    using control_matrix_t = ControlMatrix<CONTROL_DIM, SCALAR>;

    /**
     * @brief custom (internal) manifold for integrating sensitivities
     */
    class SensitivityMatrixManifold : public Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM + CONTROL_DIM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //static constexpr size_t TangentDim = DIM;
        using Scalar = SCALAR;
        using Base = Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM + CONTROL_DIM>;
        using Tangent = Base;

        SensitivityMatrixManifold() = default;
        virtual ~SensitivityMatrixManifold() = default;

        //!This constructor allows you to construct MyVectorType from Eigen expressions
        template <typename OtherDerived>
        SensitivityMatrixManifold(const Eigen::MatrixBase<OtherDerived>& other) : Base(other)
        {
        }

        //! This method allows you to assign Eigen expressions to MyVectorType
        template <typename OtherDerived>
        SensitivityMatrixManifold& operator=(const Eigen::MatrixBase<OtherDerived>& other)
        {
            this->Base::operator=(other);
            return *this;
        }
    };
    using sensitivities_matrix_t = SensitivityMatrixManifold;

    /**
     * @brief      Constructor
     *
     * @param[in]  system       The controlled system
     * @param[in]  stepperType  The integration stepper type
     */
    SensitivityIntegrator(const SCALAR dt,
        const std::shared_ptr<LinearSystem<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>>& linearSystem,
        const std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>>& controller,
        const IntegrationType stepperType = IntegrationType::EULERCT,
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
    void setStepper(const IntegrationType stepperType)
    {
        symplectic_ = false;

        switch (stepperType)
        {
            case IntegrationType::EULERCT:
            case IntegrationType::EULER_SYM:
            case IntegrationType::EULER:
            {
                stepper_ = std::shared_ptr<internal::StepperCTBase<SensitivityMatrixManifold>>(
                    new internal::StepperEulerCT<SensitivityMatrixManifold>());
                break;
            }

            case IntegrationType::RK4:
            case IntegrationType::RK4CT:
            {
                stepper_ = std::shared_ptr<internal::StepperCTBase<SensitivityMatrixManifold>>(
                    new internal::StepperRK4CT<SensitivityMatrixManifold>());
                break;
            }

            default:
                throw std::runtime_error("Integration type not supported by sensitivity integrator");
        }

        if (stepperType == IntegrationType::EULER_SYM)
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
        const std::shared_ptr<LinearSystem<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>>& linearSystem) override
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

        SensitivityMatrixManifold AB;
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

    const state_matrix_t& getDerivativeState(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t n = Time_t(0)) override
    {
        throw std::runtime_error(
            "getDerivativeState not implemented for SensitivityIntegrator. You need to use getDerivatives() for "
            "efficiency reasons.");
    }

    const state_control_matrix_t& getDerivativeControl(const MANIFOLD& m,
        const control_vector_t& u,
        const Time_t n = Time_t(0)) override
    {
        throw std::runtime_error(
            "getDerivativeState not implemented for SensitivityIntegrator. You need to use getDerivatives() for "
            "efficiency reasons.");
    }

    virtual void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& x,
        const control_vector_t& u,
        const Time_t n = Time_t(0)) override
    {
        getDerivatives(A, B, x, x, u, 1, n);
    }

    void getDerivatives(state_matrix_t& A,
        state_control_matrix_t& B,
        const MANIFOLD& x,
        const MANIFOLD& x_next,
        const control_vector_t& u,
        const size_t numSteps,
        const Time_t n = Time_t(0)) override
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

    MANIFOLD x_next_;

    state_matrix_t Aconst_;
    state_control_matrix_t Bconst_;

    std::shared_ptr<LinearSystem<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>> linearSystem_;
    std::shared_ptr<Controller<MANIFOLD, CONTROL_DIM, CONTINUOUS_TIME>> controller_;

    // Sensitivities
    std::function<void(const sensitivities_matrix_t&, typename sensitivities_matrix_t::Tangent&, const SCALAR)>
        dFdxDot_;

    std::shared_ptr<internal::StepperCTBase<sensitivities_matrix_t>> stepper_;


    inline void integrateSensitivities(const state_matrix_t& A,
        const state_control_matrix_t& B,
        const MANIFOLD& x,
        const sensitivities_matrix_t& dX0In,
        typename sensitivities_matrix_t::Tangent& dX0dt,
        const SCALAR t)
    {
        dX0dt.template leftCols<STATE_DIM>() = A * dX0In.template leftCols<STATE_DIM>();
        dX0dt.template rightCols<CONTROL_DIM>() =
            A * dX0In.template rightCols<CONTROL_DIM>() + B * controller_->getDerivativeU0(x, t);
    }

    void initStepper()
    {
        dFdxDot_ = [this](
            const sensitivities_matrix_t& dX0In, typename sensitivities_matrix_t::Tangent& dX0dt, const SCALAR t) {
#ifdef DEBUG
            if (!this->xSubstep_ || this->xSubstep_->size() <= this->k_)
                throw std::runtime_error("substeps not correctly initialized");
#endif

            const typename Sensitivity<MANIFOLD, CONTROL_DIM>::StateVectorArrayPtr& xSubstep =
                this->xSubstep_->operator[](this->k_);
            const typename Sensitivity<MANIFOLD, CONTROL_DIM>::ControlVectorArrayPtr& uSubstep =
                this->uSubstep_->operator[](this->k_);

#ifdef DEBUG

            if (!xSubstep || xSubstep->size() <= this->substep_)
            {
                throw std::runtime_error("substeps not correctly initialized");
            }
#endif
            const MANIFOLD& x = xSubstep->operator[](this->substep_);
            const control_vector_t& u = uSubstep->operator[](this->substep_);

            if (symplectic_)
            {
                state_matrix_t A_sym;
                state_control_matrix_t B_sym;
                const MANIFOLD* x_next;

                if (this->substep_ + 1 < xSubstep->size())
                    x_next = &xSubstep->operator[](this->substep_ + 1);
                else
                    x_next = &x_next_;

                getSymplecticAandB<MANIFOLD>(t, x, *x_next, u, A_sym, B_sym);

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

    CT_SYMPLECTIC_ENABLED(MANIFOLD)
    getSymplecticAandB(const SCALAR& t,
        const MANIFOLD& x,
        const MANIFOLD& x_next,
        const control_vector_t& u,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        const size_t P_DIM = MANIFOLD::PosDim;
        const size_t V_DIM = MANIFOLD::VelDim;

        // our implementation of symplectic integrators first updates the positions, we need to reconstruct an intermediate state accordingly
        MANIFOLD x_interm = x;
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

    CT_SYMPLECTIC_DISABLED(MANIFOLD)
    getSymplecticAandB(const SCALAR& t,
        const MANIFOLD& x,
        const MANIFOLD& x_next,
        const control_vector_t& u,
        state_matrix_t& A_sym,
        state_control_matrix_t& B_sym)
    {
        throw std::runtime_error("Cannot compute sensitivities for symplectic system with these dimensions.");
    }
};
}
}