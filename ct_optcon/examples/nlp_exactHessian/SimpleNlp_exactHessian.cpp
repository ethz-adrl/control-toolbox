/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example SimpleNlp_exactHessian.cpp
 *
 * This example shows how to set up and solve a Nonlinear program using the Nlp and Nlpsolver classes with exact Hessians.
 * We solve the following NLP:
 * min(x1 + x2)
 * s.t: x1 >= 0
 *      x2 >= 0
 *      1 <= x1^2 + x2^2 <= 2
 *
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
class ExampleConstraints : public tpl::DiscreteConstraintBase<SCALAR> {
public:
    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    ExampleConstraints(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) 
    : optVector_(optVector) {

        lowerBounds_(0) = static_cast<SCALAR>(0.0);
        lowerBounds_(1) = static_cast<SCALAR>(0.0);
        lowerBounds_(2) = static_cast<SCALAR>(1.0);

        upperBounds_(0) = std::numeric_limits<SCALAR>::max();
        upperBounds_(1) = std::numeric_limits<SCALAR>::max();
        upperBounds_(2) = static_cast<SCALAR>(2.0); 
    }

    ~ExampleConstraints() override = default;

    VectorXs eval() override {
        contraints_.setZero();
        contraints_(0) = optVector_->getOptimizationVars()(0);
        contraints_(1) = optVector_->getOptimizationVars()(1);
        contraints_(2) = optVector_->getOptimizationVars()(0) * optVector_->getOptimizationVars()(0) + 
                         optVector_->getOptimizationVars()(1) * optVector_->getOptimizationVars()(1);
        return contraints_;
    }

    VectorXs evalSparseJacobian() override {
        jacobian_.setZero();
        jacobian_(0) = static_cast<SCALAR>(1.0);
        jacobian_(1) = static_cast<SCALAR>(1.0);
        jacobian_(2) = static_cast<SCALAR>(2.0 * optVector_->getOptimizationVars()(0));
        jacobian_(3) = static_cast<SCALAR>(2.0 * optVector_->getOptimizationVars()(1));
        return jacobian_;
    }

    size_t getConstraintSize() override {
        return 3;
    }

    size_t getNumNonZerosJacobian() override {
        return 4;

    }

    void genSparsityPattern(Eigen::VectorXi& iRow_vec, Eigen::VectorXi& jCol_vec) override {
        iRow_vec(0) = 0;
        iRow_vec(1) = 1;
        iRow_vec(2) = 2;
        iRow_vec(3) = 2;

        jCol_vec(0) = 0;
        jCol_vec(1) = 1;
        jCol_vec(2) = 0;
        jCol_vec(3) = 1;
    }

    VectorXs getLowerBound() override {
        return lowerBounds_;
    }

    VectorXs getUpperBound() override {
        return upperBounds_;
    }

private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
    Eigen::Matrix<SCALAR, 4, 1> contraints_;
    Eigen::Matrix<SCALAR, 4, 1> jacobian_;
    Eigen::Matrix<SCALAR, 4, 1> lowerBounds_;  // lower bound
    Eigen::Matrix<SCALAR, 4, 1> upperBounds_;  // upper bound
};

/**
 * @brief      This class implements the cost function and its gradient
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleCostEvaluator : public tpl::DiscreteCostEvaluatorBase<SCALAR> {
public:

    ExampleCostEvaluator(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) 
    : optVector_(optVector) {

    }

    ~ExampleCostEvaluator() override = default;

    SCALAR eval() override {
        return static_cast<SCALAR>(optVector_->getOptimizationVars()(0) + optVector_->getOptimizationVars()(1));
    }

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override {
        grad.setZero();
        grad(0) = static_cast<SCALAR>(1.0);
        grad(1) = static_cast<SCALAR>(1.0);
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
class ExampleConstraintsContainer : public tpl::DiscreteConstraintContainerBase<SCALAR> {
public:
    ExampleConstraintsContainer(std::shared_ptr<tpl::OptVector<SCALAR>> optVector)
    : optVector_(optVector) {
        auto exampleConstraints = std::shared_ptr<ExampleConstraints<SCALAR>> (
            new ExampleConstraints<SCALAR>(optVector_));
        this->constraints_.push_back(exampleConstraints);
    }
    ~ExampleConstraintsContainer() = default;

    void prepareEvaluation() override { /* do nothing*/ }

    void prepareJacobianEvaluation() override { /* do nothing*/ }

private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
};

/**
 * @brief      Sets up the nlp to be solved by an nlpsolver
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleProblem : public tpl::Nlp<SCALAR> {
public:
    ExampleProblem() {
        this->optVariables_ = std::shared_ptr<tpl::OptVector<SCALAR>>(
            new tpl::OptVector<SCALAR>(2));

        this->optVariables_->setZero();

        this->costEvaluator_ = std::shared_ptr<ExampleCostEvaluator<SCALAR>>(
            new ExampleCostEvaluator<SCALAR>(this->optVariables_));

        this->constraints_ = std::shared_ptr<ExampleConstraintsContainer<SCALAR>> (
            new ExampleConstraintsContainer<SCALAR>(this->optVariables_));
    }

    ~ExampleProblem() override = default;

    void updateProblem() override { /* do nothing */ }

    void printSolution() {
        std::cout << "Solution: " << this->optVariables_->getOptimizationVars().transpose() << std::endl;
    }
};

int main(int argc, char** argv) {
    std::shared_ptr<tpl::Nlp<double>> exampleProblem = std::shared_ptr<ExampleProblem<double>> (
        new ExampleProblem<double>());

    NlpSolverSettings exampleNlpSolverSettings;
    exampleNlpSolverSettings.solverType_ = NlpSolverType::IPOPT;
    exampleNlpSolverSettings.ipoptSettings_.derivativeTest_ = "first-order";
    exampleNlpSolverSettings.ipoptSettings_.hessian_approximation_ = "limited-memory";

    std::shared_ptr<NlpSolver> nlpSolver = std::shared_ptr<IpoptSolver>(
        new IpoptSolver(exampleProblem, exampleNlpSolverSettings));

    if (!nlpSolver->isInitialized())
        nlpSolver->configure(exampleNlpSolverSettings);

    nlpSolver->solve();

    std::static_pointer_cast<ExampleProblem<double>>(exampleProblem)->printSolution();

    return 0;
}
