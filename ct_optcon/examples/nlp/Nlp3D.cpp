/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example NLp3D.cpp
 *
 * This example shows how to set up and solve a Nonlinear program using the Nlp and Nlpsolver classes. We show both exact
 * Hessian (requires more implementation) and Hessian-approximation versions.
 *
 * max(x1*x2 + x2*x3)
 * s.t: x1*x1 - x2*x2 + x3*x3 <= 2
 *      x1*x1 + x2*x2 + x3*x3 <= 10
 *
 * This is a slightly more difficult problem, with local minima which only converges with nonzero initial guess.
 * Taken from Wikipedia: https://en.wikipedia.org/wiki/Nonlinear_programming#3-dimensional_example.
 * Wikipedia provides a nice visualization online.
 */

#include <ct/optcon/nlp/Nlp>

using namespace ct::optcon;

/**
 * @brief      This class sets up the constraints and its first order
 *             derivatives described previously
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleConstraints final : public tpl::DiscreteConstraintBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    static const size_t dimConstraints = 2;
    static const size_t nnzJac = 6;
    static const size_t nnzHes = 3;  // (triangular view)

    ExampleConstraints(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector)
    {
        lowerBounds_(0) = std::numeric_limits<SCALAR>::min();
        lowerBounds_(1) = std::numeric_limits<SCALAR>::min();

        upperBounds_(0) = static_cast<SCALAR>(2.0);
        upperBounds_(1) = static_cast<SCALAR>(10.0);
    }

    ~ExampleConstraints() override = default;

    VectorXs getLowerBound() override { return lowerBounds_; }
    VectorXs getUpperBound() override { return upperBounds_; }
    VectorXs eval() override
    {
        constraints_.setZero();

        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);
        SCALAR x3 = optVector_->getOptimizationVars()(2);

        constraints_(0) = x1 * x1 - x2 * x2 + x3 * x3;
        constraints_(1) = x1 * x1 + x2 * x2 + x3 * x3;
        return constraints_;
    }

    size_t getConstraintSize() override { return dimConstraints; }
    size_t getNumNonZerosJacobian() override { return nnzJac; }
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec.resize(nnzJac);
        jCol_vec.resize(nnzJac);

        iRow_vec(0) = 0;
        jCol_vec(0) = 0;

        iRow_vec(1) = 0;
        jCol_vec(1) = 1;

        iRow_vec(2) = 0;
        jCol_vec(2) = 2;

        iRow_vec(3) = 1;
        jCol_vec(3) = 0;

        iRow_vec(4) = 1;
        jCol_vec(4) = 1;

        iRow_vec(5) = 1;
        jCol_vec(5) = 2;
    }

    VectorXs evalSparseJacobian() override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);
        SCALAR x3 = optVector_->getOptimizationVars()(2);

        jacobian_.setZero();
        jacobian_(0) = static_cast<SCALAR>(2.0 * x1);
        jacobian_(1) = static_cast<SCALAR>(-2.0 * x2);
        jacobian_(2) = static_cast<SCALAR>(2.0 * x3);
        jacobian_(3) = static_cast<SCALAR>(2.0 * x1);
        jacobian_(4) = static_cast<SCALAR>(2.0 * x2);
        jacobian_(5) = static_cast<SCALAR>(2.0 * x3);
        return jacobian_;
    }

    /*!
     * generate block-diagonal sparsity pattern for Hessian
     * \note this function implementation is only required for the exact-Hessian solver case
     */
    void genSparsityPatternHessian(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec.resize(nnzHes);
        jCol_vec.resize(nnzHes);
        iRow_vec(0) = 0;
        jCol_vec(0) = 0;
        iRow_vec(1) = 1;
        jCol_vec(1) = 1;
        iRow_vec(2) = 2;
        jCol_vec(2) = 2;
    }

    /*!
     * \note this function implementation is only required for the exact-Hessian solver case
     */
    void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& sparseHes) override
    {
        Eigen::VectorXd h1, h2;
        h1.resize(nnzHes);
        h2.resize(nnzHes);
        h1 << 2.0, -2.0, 2.0;
        h2 << 2.0, 2.0, 2.0;

        sparseHes = lambda(0) * h1 + lambda(1) * h2;
    }


private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
    Eigen::Matrix<SCALAR, nnzJac, 1> jacobian_;
    Eigen::Matrix<SCALAR, dimConstraints, 1> constraints_;
    Eigen::Matrix<SCALAR, dimConstraints, 1> lowerBounds_;
    Eigen::Matrix<SCALAR, dimConstraints, 1> upperBounds_;
};


/**
 * @brief      This class implements the cost function and its gradient
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleCostEvaluator final : public tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const size_t nnz_hessian = 2;  //! number non-zero elements in the hessian (triangular view)

    ExampleCostEvaluator(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector) {}
    ~ExampleCostEvaluator() override = default;

    SCALAR eval() override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);
        SCALAR x3 = optVector_->getOptimizationVars()(2);

        return -(x1 * x2 + x2 * x3);
    }

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);
        SCALAR x3 = optVector_->getOptimizationVars()(2);

        grad.resize(grad_length);
        grad(0) = -x2;
        grad(1) = -x1 - x3;
        grad(2) = -x2;
    }

    /*!
    * \note this function implementation is only required for the exact-Hessian solver case
    * \note we only implement the lower-triangular part of the sparsity pattern
    */
    void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol) override
    {
        iRow.resize(nnz_hessian);
        jCol.resize(nnz_hessian);

        iRow(0) = 1;
        jCol(0) = 0;

        iRow(1) = 2;
        jCol(1) = 1;
    }


    /*!
    * \note this function implementation is only required for the exact-Hessian solver case
    */
    void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes) override
    {
        hes.resize(nnz_hessian);
        hes.setConstant(-1.0 * lambda(0));
    }

private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
};

/**
 * @brief      This class acts as a container for multiple constraints. In this
 *             example we capture all the constraints in one constraint clas
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleConstraintsContainer final : public tpl::DiscreteConstraintContainerBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ExampleConstraintsContainer(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector)
    {
        this->constraints_.push_back(
            std::shared_ptr<ExampleConstraints<SCALAR>>(new ExampleConstraints<SCALAR>(optVector_)));
    }
    ~ExampleConstraintsContainer() override = default;

    void prepareEvaluation() override { /* do nothing*/}

    void prepareJacobianEvaluation() override { /* do nothing*/}

private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
};

/**
 * @brief      Sets up the nlp to be solved by an nlpsolver
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleProblem final : public tpl::Nlp<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ExampleProblem()
    {
        this->optVariables_ = std::shared_ptr<tpl::OptVector<SCALAR>>(new tpl::OptVector<SCALAR>(3));

        this->optVariables_->setInitialGuess(Eigen::Vector3d::Identity());

        this->costEvaluator_ =
            std::shared_ptr<ExampleCostEvaluator<SCALAR>>(new ExampleCostEvaluator<SCALAR>(this->optVariables_));

        this->constraints_ = std::shared_ptr<ExampleConstraintsContainer<SCALAR>>(
            new ExampleConstraintsContainer<SCALAR>(this->optVariables_));
    }

    ~ExampleProblem() override = default;

    void updateProblem() override { /* do nothing */}

    Eigen::VectorXd getSolution() { return this->optVariables_->getOptimizationVars(); }
};

int main(int argc, char** argv)
{
    // create problems
    std::shared_ptr<ExampleProblem<double>> problem1(new ExampleProblem<double>());
    std::shared_ptr<ExampleProblem<double>> problem2(new ExampleProblem<double>());

    // settings for exact Hessian solver
    NlpSolverSettings solverSettings1;
    solverSettings1.solverType_ = NlpSolverType::IPOPT;
    solverSettings1.ipoptSettings_.derivativeTest_ = "second-order";
    solverSettings1.ipoptSettings_.hessian_approximation_ = "exact";

    // create exact-Hessian solver and solve
    std::shared_ptr<NlpSolver> nlpSolver1(new IpoptSolver(problem1, solverSettings1));
    nlpSolver1->solve();
    std::cout << "Solution: " << problem1->getSolution().transpose() << std::endl;


    // settings for Hessian approximation solver
    NlpSolverSettings solverSettings2;
    solverSettings2.solverType_ = NlpSolverType::IPOPT;
    solverSettings2.ipoptSettings_.hessian_approximation_ = "limited-memory";

    // create approximate-Hessian solver and solve
    std::shared_ptr<NlpSolver> nlpSolver2(new IpoptSolver(problem2, solverSettings2));
    nlpSolver2->solve();
    std::cout << "Solution: " << problem2->getSolution().transpose() << std::endl;

    return 0;
}
