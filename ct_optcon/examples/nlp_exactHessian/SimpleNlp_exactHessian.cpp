/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

/*!
 * \example SimpleNlp_exactHessian.cpp
 *
 * This example shows how to set up and solve a Nonlinear program using the Nlp and Nlpsolver classes with exact Hessians.
 * We solve the following NLP:
 * min (x1 + x2)^2
 * s.t: x1 >= 0
 *      x2 >= 0
 *      1 <= x1^2 + x2^2 <= 2
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

    VectorXs eval() override
    {
        contraints_.setZero();

        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);

        contraints_(0) = x1;
        contraints_(1) = x2;
        contraints_(2) = x1 * x1 + x2 * x2;
        return contraints_;
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
     */
    Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& optVec, const Eigen::VectorXd& lambda) override
    {
    	Eigen::VectorXd hes (2);
    	hes.setConstant(lambda(2) * 2.0);
    	return hes;
    }

    VectorXs getLowerBound() override { return lowerBounds_; }
    VectorXs getUpperBound() override { return upperBounds_; }
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
class ExampleCostEvaluator final : public tpl::DiscreteCostEvaluatorBase<SCALAR>
{
public:
    ExampleCostEvaluator(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector) {}
    ~ExampleCostEvaluator() override = default;

    SCALAR eval() override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);

        return static_cast<SCALAR>((x1 + x2) * (x1 + x2));
    }

    void evalGradient(size_t grad_length, Eigen::Map<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>& grad) override
    {
        SCALAR x1 = optVector_->getOptimizationVars()(0);
        SCALAR x2 = optVector_->getOptimizationVars()(1);
        grad.resize(grad_length);
        grad(0) = static_cast<SCALAR>(2 * (x1 + x2));
        grad(1) = static_cast<SCALAR>(2 * (x1 + x2));
    }

    size_t getNonZeroHessianCount() override { return 4; }
    void getSparsityPatternHessian(Eigen::VectorXi& iRow, Eigen::VectorXi& jCol) override
    {
        iRow.resize(getNonZeroHessianCount());
        jCol.resize(getNonZeroHessianCount());
        iRow(0) = 0;
        jCol(0) = 0;
        iRow(1) = 0;
        jCol(1) = 1;
        iRow(2) = 1;
        jCol(2) = 0;
        iRow(3) = 1;
        jCol(3) = 1;
    }

    Eigen::VectorXd sparseHessianValues(const Eigen::VectorXd& optVec, const Eigen::VectorXd& lambda) override
    {
        Eigen::VectorXd hes(getNonZeroHessianCount());
        hes.setConstant(2.0 * lambda(0,0));
        return hes;
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
    ExampleConstraintsContainer(std::shared_ptr<tpl::OptVector<SCALAR>> optVector) : optVector_(optVector)
    {
        auto exampleConstraints =
            std::shared_ptr<ExampleConstraints<SCALAR>>(new ExampleConstraints<SCALAR>(optVector_));
        this->constraints_.push_back(exampleConstraints);
    }
    ~ExampleConstraintsContainer() = default;

    void prepareEvaluation() override { /* do nothing*/}

    void prepareJacobianEvaluation() override { /* do nothing*/}

private:
    std::shared_ptr<tpl::OptVector<SCALAR>> optVector_;
};

/**
 * @brief      Sets up the nlp to be solved by an nlpsolver
 *
 * @tparam     SCALAR  Scalar type
 */
template <typename SCALAR>
class ExampleProblem final : public tpl::Nlp<SCALAR>
{
public:
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

    void printSolution()
    {
        std::cout << "Solution: " << this->optVariables_->getOptimizationVars().transpose() << std::endl;
    }
};

int main(int argc, char** argv)
{
    std::shared_ptr<tpl::Nlp<double>> exampleProblem =
        std::shared_ptr<ExampleProblem<double>>(new ExampleProblem<double>());

    NlpSolverSettings exampleNlpSolverSettings;
    exampleNlpSolverSettings.solverType_ = NlpSolverType::IPOPT;
    exampleNlpSolverSettings.ipoptSettings_.derivativeTest_ = "second-order";
    exampleNlpSolverSettings.ipoptSettings_.hessian_approximation_ = "exact";


    std::shared_ptr<NlpSolver> nlpSolver =
        std::shared_ptr<IpoptSolver>(new IpoptSolver(exampleProblem, exampleNlpSolverSettings));

    if (!nlpSolver->isInitialized())
        nlpSolver->configure(exampleNlpSolverSettings);

    nlpSolver->solve();

    std::static_pointer_cast<ExampleProblem<double>>(exampleProblem)->printSolution();

    return 0;
}
