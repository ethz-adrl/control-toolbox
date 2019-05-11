/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example NLp2D.cpp
 *
 * This example shows how to set up and solve a Nonlinear program using the Nlp and Nlpsolver classes. We show both exact
 * Hessian (requires more implementation) and Hessian-approximation versions.
 * We solve the following NLP:
 *
 * max(x1 + x2)
 * s.t: x1 >= 0
 *      x2 >= 0
 *      1 <= x1^2 + x2^2 <= 2
 *
 * The optimal solution to this problem is x = [1, 1].
 * (taken from Wikipedia: https://en.wikipedia.org/wiki/Nonlinear_programming#2-dimensional_example)
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

    ExampleConstraints(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector)
    {
        lowerBounds_(0) = static_cast<SCALAR>(0.0);
        lowerBounds_(1) = static_cast<SCALAR>(0.0);
        lowerBounds_(2) = static_cast<SCALAR>(1.0);

        upperBounds_(0) = std::numeric_limits<SCALAR>::max();
        upperBounds_(1) = std::numeric_limits<SCALAR>::max();
        upperBounds_(2) = static_cast<SCALAR>(2.0);
    }

    ~ExampleConstraints() override = default;

    VectorXs getLowerBound() override { return lowerBounds_; }
    VectorXs getUpperBound() override { return upperBounds_; }
    VectorXs eval() override
    {
        constraints_.setZero();

        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);

        constraints_(0) = x1;
        constraints_(1) = x2;
        constraints_(2) = x1 * x1 + x2 * x2;
        return constraints_;
    }

    VectorXs evalSparseJacobian() override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);

        jacobian_.setZero();
        jacobian_(0) = static_cast<SCALAR>(1.0);
        jacobian_(1) = static_cast<SCALAR>(1.0);
        jacobian_(2) = static_cast<SCALAR>(2.0 * x1);
        jacobian_(3) = static_cast<SCALAR>(2.0 * x2);
        return jacobian_;
    }

    size_t getConstraintSize() override { return 3; }
    size_t getNumNonZerosJacobian() override { return 4; }
    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec(0) = 0;
        iRow_vec(1) = 1;
        iRow_vec(2) = 2;
        iRow_vec(3) = 2;

        jCol_vec(0) = 0;
        jCol_vec(1) = 1;
        jCol_vec(2) = 0;
        jCol_vec(3) = 1;
    }

    /*
     * When we write down the three sub-hessian for this constraints we see H1 = 0, H2 = 0 and
     * H3 = [2 0; 0 2].
     *
     * \note this function implementation is only required for the exact-hessian solver case
     */
    void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& sparseHes) override
    {
        sparseHes.resize(2);
        sparseHes.setConstant(lambda(2) * 2.0);
    }

    /*
     * generate block-diagonal sparsity pattern
     * \note this function implementation is only required for the exact-hessian solver case
     */
    void genSparsityPatternHessian(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override
    {
        iRow_vec.resize(2);
        jCol_vec.resize(2);
        iRow_vec(0) = 0;
        jCol_vec(0) = 0;
        iRow_vec(1) = 1;
        jCol_vec(1) = 1;
    }


private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
    Eigen::Matrix<SCALAR, 4, 1> jacobian_;
    Eigen::Matrix<SCALAR, 3, 1> constraints_;
    Eigen::Matrix<SCALAR, 3, 1> lowerBounds_;  // lower bound
    Eigen::Matrix<SCALAR, 3, 1> upperBounds_;  // upper bound
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
    ExampleCostEvaluator(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector) {}
    ~ExampleCostEvaluator() override = default;

    SCALAR eval() override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);

        return -static_cast<SCALAR>(x1 + x2);
    }

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        grad.resize(grad_length);
        grad(0) = static_cast<SCALAR>(-1.0);
        grad(1) = static_cast<SCALAR>(-1.0);
    }


    /*
    * \note this function implementation is only required for the exact-hessian solver case
    */
    void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol) override { /* do nothing */}

    void sparseHessianValues(const Eigen::VectorXd& optVec,
        const Eigen::VectorXd& lambda,
        Eigen::VectorXd& hes) override
    {
        /* do nothing, Hessian is all zero */
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
        auto exampleConstraints =
            std::shared_ptr<ExampleConstraints<SCALAR>>(new ExampleConstraints<SCALAR>(optVector_));
        this->constraints_.push_back(exampleConstraints);
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
        this->optVariables_ = std::shared_ptr<tpl::OptVector<SCALAR>>(new tpl::OptVector<SCALAR>(2));

        this->optVariables_->setZero();

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
