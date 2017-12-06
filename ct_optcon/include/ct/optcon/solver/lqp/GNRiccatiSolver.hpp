/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "LQOCSolver.hpp"

#ifdef MATLAB_FULL_LOG
#include <ct/optcon/matlab.hpp>
#endif

namespace ct {
namespace optcon {

/*!
 * This class implements an general Riccati backward pass for solving an unconstrained
 *  linear-quadratic Optimal Control problem
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class GNRiccatiSolver : public LQOCSolver<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int state_dim = STATE_DIM;
    static const int control_dim = CONTROL_DIM;

    typedef LQOCProblem<STATE_DIM, CONTROL_DIM, SCALAR> LQOCProblem_t;

    typedef ct::core::StateMatrix<STATE_DIM, SCALAR> StateMatrix;
    typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> StateMatrixArray;
    typedef ct::core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;
    typedef ct::core::ControlMatrix<CONTROL_DIM, SCALAR> ControlMatrix;
    typedef ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> ControlMatrixArray;
    typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> StateControlMatrixArray;
    typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> FeedbackArray;

    typedef ct::core::StateVectorArray<STATE_DIM, SCALAR> StateVectorArray;
    typedef ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> ControlVectorArray;

    GNRiccatiSolver(const std::shared_ptr<LQOCProblem_t>& lqocProblem = nullptr);

    GNRiccatiSolver(int N);

    virtual ~GNRiccatiSolver();

    virtual void solve() override;

    virtual void initializeAndAllocate() override;

    virtual void solveSingleStage(int N) override;

    virtual void configure(const NLOptConSettings& settings) override;

    virtual ct::core::StateVectorArray<STATE_DIM, SCALAR> getSolutionState() override;

    virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getSolutionControl() override;

    virtual ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> getFeedforwardUpdates() override;

    virtual void getFeedback(ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>& K) override;

    //! compute the state and control updates.
    /*!
	 * this method is specific to the GN Riccati solver, since the state updates lx_
	 * need to be completed in an additional forward sweep.
	 *
	 * IMPORTANT: you need to call this method at the right place if you're using solveSingleStage() by yourself.
	 */
    virtual void computeStateAndControlUpdates() override;

    virtual SCALAR getSmallestEigenvalue() override;

protected:
    /*!
	 * resize matrices
	 * @param lqocProblem
	 */
    virtual void setProblemImpl(std::shared_ptr<LQOCProblem_t> lqocProblem) override;

    void changeNumberOfStages(int N);

    void initializeCostToGo();

    void computeCostToGo(size_t k);

    void designController(size_t k);

    void logToMatlab();

    NLOptConSettings settings_;

    ControlVectorArray gv_;
    FeedbackArray G_;

    ControlMatrixArray H_;
    ControlMatrixArray Hi_;
    ControlMatrixArray Hi_inverse_;
    ControlMatrix H_corrFix_;

    ControlVectorArray lv_;
    FeedbackArray L_;

    StateVectorArray sv_;
    StateMatrixArray S_;

    int N_;

    SCALAR smallestEigenvalue_;

    //! Eigenvalue solver, used for inverting the Hessian and for regularization
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM>> eigenvalueSolver_;

//! if building with MATLAB support, include matfile
#ifdef MATLAB_FULL_LOG
    matlab::MatFile matFile_;
#endif  //MATLAB
};


}  // namespace optcon
}  // namespace ct
