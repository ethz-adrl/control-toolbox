/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

/**
 * @brief Cholesky square root decomposition of a symmetric positive-definite matrix
 * @param _MatrixType The matrix type
 * @param _UpLo Square root form (Eigen::Lower or Eigen::Upper)
 */
template <typename _MatrixType, int _UpLo = Eigen::Lower>
class Cholesky : public Eigen::LLT<_MatrixType, _UpLo>
{
public:
    Cholesky() : Eigen::LLT<_MatrixType, _UpLo>() {}
    /**
     * @brief Construct cholesky square root decomposition from matrix
     * @param m The matrix to be decomposed
     */
    Cholesky(const _MatrixType& m) : Eigen::LLT<_MatrixType, _UpLo>(m) {}
    /**
     * @brief Set decomposition to identity
     */
    Cholesky& setIdentity()
    {
        this->m_matrix.setIdentity();
        this->m_isInitialized = true;
        return *this;
    }

    /**
     * @brief Check whether the decomposed matrix is the identity matrix
     */
    bool isIdentity() const
    {
        eigen_assert(this->m_isInitialized && "LLT is not initialized.");
        return this->m_matrix.isIdentity();
    }

    /**
     * @brief Set lower triangular part of the decomposition
     * @param matrix The lower part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setL(const Eigen::MatrixBase<Derived>& matrix)
    {
        this->m_matrix        = matrix.template triangularView<Eigen::Lower>();
        this->m_isInitialized = true;
        return *this;
    }

    /**
     * @brief Set upper triangular part of the decomposition
     * @param matrix The upper part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setU(const Eigen::MatrixBase<Derived>& matrix)
    {
        this->m_matrix        = matrix.template triangularView<Eigen::Upper>().adjoint();
        this->m_isInitialized = true;
        return *this;
    }
};

template <size_t STATE_DIM, typename SCALAR>
struct UnscentedKalmanFilterSettings;

template <size_t STATE_DIM, typename SCALAR = double>
class UnscentedKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base                  = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;

    static constexpr size_t SigmaPointCount = 2 * STATE_DIM + 1;

    template <size_t SIZE>
    using SigmaPoints = Eigen::Matrix<SCALAR, SIZE, SigmaPointCount>;

    template <size_t SIZE>
    using Covariance = Eigen::Matrix<SCALAR, SIZE, SIZE>;

    template <size_t SIZE>
    using CovarianceSquareRoot = Cholesky<Eigen::Matrix<SCALAR, SIZE, SIZE>>;

    UnscentedKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(),
        SCALAR alpha                               = SCALAR(1),
        SCALAR beta                                = SCALAR(2),
        SCALAR kappa                               = SCALAR(0),
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& P0 = ct::core::StateMatrix<STATE_DIM, SCALAR>::Identity())
        : Base(x0), alpha_(alpha), beta_(beta), kappa_(kappa), P_(P0)
    {
    }

    UnscentedKalmanFilter(const UnscentedKalmanFilterSettings<STATE_DIM, SCALAR>& ukf_settings)
        : Base(ukf_settings.x0),
          alpha_(ukf_settings.alpha),
          beta_(ukf_settings.beta),
          kappa_(ukf_settings.kappa),
          P_(ukf_settings.P0)
    {
    }

    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0)
    {
        if (!computeSigmaPoints())
            throw std::runtime_error("UnscentedKalmanFilter : Numerical error.");

        this->x_est_ = this->template computeStatePrediction<CONTROL_DIM>(f, u, t);

        computeCovarianceFromSigmaPoints(
            this->x_est_, sigmaStatePoints_, f.computeDerivativeNoise(this->x_est_, t), P_);

        return this->x_est_;
    }

    template <size_t OUTPUT_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& z,
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
        const ct::core::Time& t = 0)
    {
        SigmaPoints<OUTPUT_DIM> sigmaMeasurementPoints;

        // Predict measurement (and corresponding sigma points)
        ct::core::OutputVector<OUTPUT_DIM, SCALAR> y =
            this->template computeMeasurementPrediction<OUTPUT_DIM>(h, sigmaMeasurementPoints, t);

        // Compute innovation covariance
        Covariance<OUTPUT_DIM> P;
        computeCovarianceFromSigmaPoints(y, sigmaMeasurementPoints, h.computeDerivativeNoise(this->x_est_, t), P);

        Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM> K;
        computeKalmanGain(y, sigmaMeasurementPoints, P, K);

        // Update state
        this->x_est_ += K * (z - y);

        // Update state covariance
        updateStateCovariance<OUTPUT_DIM>(K, P);

        return this->x_est_;
    }

    bool computeSigmaPoints()
    {
        // Get square root of covariance
        CovarianceSquareRoot<STATE_DIM> llt;
        llt.compute(P_);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> _S = llt.matrixL().toDenseMatrix();

        // Set left "block" (first column)
        this->sigmaStatePoints_.template leftCols<1>() = this->x_est_;
        // Set center block with x + gamma_ * S
        this->sigmaStatePoints_.template block<STATE_DIM, STATE_DIM>(0, 1) = (gamma_ * _S).colwise() + this->x_est_;
        // Set right block with x - gamma_ * S
        this->sigmaStatePoints_.template rightCols<STATE_DIM>() = (-gamma_ * _S).colwise() + this->x_est_;

        return true;
    }

    template <size_t SIZE>
    bool computeCovarianceFromSigmaPoints(const Eigen::Matrix<SCALAR, SIZE, 1>& mean,
        const SigmaPoints<SIZE>& sigmaPoints,
        const Covariance<SIZE>& noiseCov,
        Covariance<SIZE>& cov)
    {
        SigmaPoints<SIZE> W   = this->sigmaWeights_c_.transpose().template replicate<SIZE, 1>();
        SigmaPoints<SIZE> tmp = (sigmaPoints.colwise() - mean);
        cov                   = tmp.cwiseProduct(W) * tmp.transpose() + noiseCov;

        return true;
    }

    template <size_t OUTPUT_DIM>
    bool computeKalmanGain(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        const SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const Covariance<OUTPUT_DIM>& P_yy,
        Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM>& K)
    {
        SigmaPoints<STATE_DIM> W = this->sigmaWeights_c_.transpose().template replicate<STATE_DIM, 1>();
        Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM> P_xy =
            (sigmaStatePoints_.colwise() - this->x_est_).cwiseProduct(W).eval() *
            (sigmaMeasurementPoints.colwise() - y).transpose();

        K = P_xy * P_yy.inverse();
        return true;
    }

    template <size_t OUTPUT_DIM>
    bool updateStateCovariance(const Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM>& K, const Covariance<OUTPUT_DIM>& P)
    {
        P_ -= K * P * K.transpose();
        return true;
    }

    template <size_t CONTROL_DIM>
    const state_vector_t& computeStatePrediction(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::Time& t = 0)
    {
        // Pass each sigma point through non-linear state transition function
        computeSigmaPointTransition(f, u, t);

        // Compute predicted state from predicted sigma points
        return computePredictionFromSigmaPoints<STATE_DIM>(sigmaStatePoints_);
    }

    template <size_t OUTPUT_DIM>
    ct::core::OutputVector<OUTPUT_DIM, SCALAR> computeMeasurementPrediction(
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const ct::core::Time& t = 0)
    {
        // Predict measurements for each sigma point
        computeSigmaPointMeasurements<OUTPUT_DIM>(h, sigmaMeasurementPoints, t);

        // Predict measurement from sigma measurement points
        return computePredictionFromSigmaPoints<OUTPUT_DIM>(sigmaMeasurementPoints);
    }

    void computeWeights()
    {
        SCALAR L = SCALAR(STATE_DIM);
        lambda_  = alpha_ * alpha_ * (L + kappa_) - L;
        gamma_   = std::sqrt(L + lambda_);

        // Make sure L != -lambda_ to avoid division by zero
        assert(std::abs(L + lambda_) > 1e-6);

        // Make sure L != -kappa_ to avoid division by zero
        assert(std::abs(L + kappa_) > 1e-6);

        SCALAR W_m_0 = lambda_ / (L + lambda_);
        SCALAR W_c_0 = W_m_0 + (SCALAR(1) - alpha_ * alpha_ + beta_);
        SCALAR W_i   = SCALAR(1) / (SCALAR(2) * alpha_ * alpha_ * (L + kappa_));

        // Make sure W_i > 0 to avoid square-root of negative number
        assert(W_i > SCALAR(0));

        sigmaWeights_m_[0] = W_m_0;
        sigmaWeights_c_[0] = W_c_0;

        for (int i = 1; i < SigmaPointCount; ++i)
        {
            sigmaWeights_m_[i] = W_i;
            sigmaWeights_c_[i] = W_i;
        }
    }

    template <size_t CONTROL_DIM>
    void computeSigmaPointTransition(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::Time& t = 0)
    {
        for (int i = 0; i < SigmaPointCount; ++i)
        {
            // TODO: Update the controller with u. Currently u is just placeholder and the controller of the system is
            // used. Or disable u in this method and set the controller to constant controller before calling it.
            sigmaStatePoints_.col(i) = f.computeDynamics(sigmaStatePoints_.col(i), u, t);
        }
    }

    template <size_t OUTPUT_DIM>
    void computeSigmaPointMeasurements(LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const ct::core::Time& t = 0)
    {
        for (int i = 0; i < SigmaPointCount; ++i)
        {
            sigmaMeasurementPoints.col(i) = h.computeMeasurement(sigmaStatePoints_.col(i), t);
        }
    }

    template <size_t OUTPUT_DIM>
    state_vector_t computePredictionFromSigmaPoints(const SigmaPoints<OUTPUT_DIM>& sigmaPoints)
    {
        // Use efficient matrix x vector computation
        return sigmaPoints * sigmaWeights_m_;
    }

private:
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;
    Eigen::Matrix<SCALAR, SigmaPointCount, 1> sigmaWeights_m_;
    Eigen::Matrix<SCALAR, SigmaPointCount, 1> sigmaWeights_c_;
    SigmaPoints<STATE_DIM> sigmaStatePoints_;
    SCALAR alpha_;  //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
    SCALAR beta_;  //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
    SCALAR kappa_;   //!< Secondary scaling parameter (usually 0)
    SCALAR gamma_;   //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the state dimensionality
    SCALAR lambda_;  //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$ being the state dimensionality
};

}  // optcon
}  // ct
