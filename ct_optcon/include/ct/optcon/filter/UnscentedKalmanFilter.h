/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "EstimatorBase.h"

namespace ct {
namespace optcon {

/*!
 * \ingroup Filter
 *
 * \brief Cholesky square root decomposition of a symmetric positive-definite matrix.
 *
 * @tparam MatrixType The matrix type
 * @tparam UpLo Square root form (Eigen::Lower or Eigen::Upper)
 */
template <typename MatrixType, int UpLo = Eigen::Lower>
class Cholesky : public Eigen::LLT<MatrixType, UpLo>
{
public:
    //! Constructor.
    Cholesky();
    //! Construct cholesky square root decomposition from matrix
    Cholesky(const MatrixType& m);
    //! Set decomposition to identity
    Cholesky& setIdentity();

    //! Check whether the decomposed matrix is the identity matrix
    bool isIdentity() const;

    /*!
     * \brief Set lower triangular part of the decomposition
     * @param matrix The lower part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setL(const Eigen::MatrixBase<Derived>& matrix);

    /*!
     * \brief Set upper triangular part of the decomposition
     * @param matrix The upper part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setU(const Eigen::MatrixBase<Derived>& matrix);
};

template <size_t STATE_DIM, typename SCALAR>
struct UnscentedKalmanFilterSettings;

/*!
 * \ingroup Filter
 *
 * \brief Unscented Kalman Filter is a nonlinear estimator best suited for highly nonlinear systems. It combines the
 *        principles of EKF and particle filter. The downside is the computation complexity.
 *
 * @tparam STATE_DIM
 */
template <size_t STATE_DIM, typename SCALAR = double>
class UnscentedKalmanFilter : public EstimatorBase<STATE_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t STATE_D = STATE_DIM;
    using Base = EstimatorBase<STATE_DIM, SCALAR>;
    using typename Base::state_vector_t;

    static constexpr size_t SigmaPointCount = 2 * STATE_DIM + 1;

    template <size_t SIZE>
    using SigmaPoints = Eigen::Matrix<SCALAR, SIZE, SigmaPointCount>;

    template <size_t SIZE>
    using Covariance = Eigen::Matrix<SCALAR, SIZE, SIZE>;

    template <size_t SIZE>
    using CovarianceSquareRoot = Cholesky<Eigen::Matrix<SCALAR, SIZE, SIZE>>;

    //! Constructor.
    UnscentedKalmanFilter(const state_vector_t& x0 = state_vector_t::Zero(),
        SCALAR alpha = SCALAR(1),
        SCALAR beta = SCALAR(2),
        SCALAR kappa = SCALAR(0),
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& P0 = ct::core::StateMatrix<STATE_DIM, SCALAR>::Identity());

    //! Constructor from settings.
    UnscentedKalmanFilter(const UnscentedKalmanFilterSettings<STATE_DIM, SCALAR>& ukf_settings);

    //! Estimator predict method.
    template <size_t CONTROL_DIM>
    const state_vector_t& predict(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::StateMatrix<STATE_DIM, SCALAR>& Q,
        const ct::core::Time& t = 0);

    //! Estimator update method.
    template <size_t OUTPUT_DIM>
    const state_vector_t& update(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& z,
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        const ct::core::OutputMatrix<OUTPUT_DIM, SCALAR>& R,
        const ct::core::Time& t = 0);

    //! Compute sigma points from current state and covariance estimates.
    bool computeSigmaPoints();

    //! Estimate covariance from sigma points and noise covariance.
    template <size_t SIZE>
    bool computeCovarianceFromSigmaPoints(const Eigen::Matrix<SCALAR, SIZE, 1>& mean,
        const SigmaPoints<SIZE>& sigmaPoints,
        const Covariance<SIZE>& noiseCov,
        Covariance<SIZE>& cov);

    //! Compute the Kalman Gain using sigma points..
    template <size_t OUTPUT_DIM>
    bool computeKalmanGain(const ct::core::OutputVector<OUTPUT_DIM, SCALAR>& y,
        const SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const Covariance<OUTPUT_DIM>& P_yy,
        Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM>& K);

    //! Compute the Kalman Gain using sigma points..
    template <size_t OUTPUT_DIM>
    bool updateStateCovariance(const Eigen::Matrix<SCALAR, STATE_DIM, OUTPUT_DIM>& K, const Covariance<OUTPUT_DIM>& P);

    //! Update state covariance from Kalman gain and sigma points.
    template <size_t CONTROL_DIM>
    const state_vector_t& computeStatePrediction(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::Time& t = 0);

    //! Predict the next measurement.
    template <size_t OUTPUT_DIM>
    ct::core::OutputVector<OUTPUT_DIM, SCALAR> computeMeasurementPrediction(
        LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const ct::core::Time& t = 0);

    //! Compute weights of sigma points.
    void computeWeights();

    /*!
     * \brief Propagate sigma points through the system dynamics. Using CTSystemModel here will not provide a desired
     *        effect since it disregards the control input u.
     */
    template <size_t CONTROL_DIM>
    void computeSigmaPointTransition(SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>& f,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& u,
        const ct::core::Time& t = 0);

    //! Predict measurements of sigma points.
    template <size_t OUTPUT_DIM>
    void computeSigmaPointMeasurements(LinearMeasurementModel<OUTPUT_DIM, STATE_DIM, SCALAR>& h,
        SigmaPoints<OUTPUT_DIM>& sigmaMeasurementPoints,
        const ct::core::Time& t = 0);

    //! Make a prediction based on sigma points.
    template <size_t OUTPUT_DIM>
    state_vector_t computePredictionFromSigmaPoints(const SigmaPoints<OUTPUT_DIM>& sigmaPoints);

private:
    ct::core::StateMatrix<STATE_DIM, SCALAR> P_;                //! Covariance matrix.
    Eigen::Matrix<SCALAR, SigmaPointCount, 1> sigmaWeights_m_;  //! Sigma measurement weights.
    Eigen::Matrix<SCALAR, SigmaPointCount, 1> sigmaWeights_c_;  //! Sigma covariance weights.
    SigmaPoints<STATE_DIM> sigmaStatePoints_;                   //! Sigma points.
    SCALAR alpha_;  //! Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
    SCALAR beta_;   //! Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
    SCALAR kappa_;  //! Secondary scaling parameter (usually 0)
    SCALAR gamma_;  //! \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the state dimensionality
    SCALAR lambda_;  //! \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$ being the state dimensionality
};

}  // optcon
}  // ct
