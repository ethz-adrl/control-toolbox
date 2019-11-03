/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

namespace ct {
namespace optcon {


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAD()
{
    stateControlD_.setZero();

    fIntermediate_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput) {
        return this->evaluateIntermediateCodegen(stateinput);
    };

    fTerminal_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput) {
        return this->evaluateTerminalCodegen(stateinput);
    };

    intermediateCodegen_ =
        std::shared_ptr<JacCG>(new JacCG(fIntermediate_, STATE_DIM + CONTROL_DIM, getIntermediateConstraintsCount()));
    terminalCodegen_ =
        std::shared_ptr<JacCG>(new JacCG(fTerminal_, STATE_DIM + CONTROL_DIM, getTerminalConstraintsCount()));
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAD(const state_vector_t& x,
    const input_vector_t& u,
    const SCALAR& t)
{
    //Set to some random number which is != the initguess of the problem
    stateControlD_ << x, u;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAD(const ConstraintContainerAD& arg)
    : LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      fIntermediate_(arg.fIntermediate_),
      fTerminal_(arg.fTerminal_),
      sparsityIntermediateRows_(arg.sparsityIntermediateRows_),
      sparsityStateIntermediateRows_(arg.sparsityStateIntermediateRows_),
      sparsityStateIntermediateCols_(arg.sparsityStateIntermediateCols_),
      sparsityInputIntermediateRows_(arg.sparsityInputIntermediateRows_),
      sparsityInputIntermediateCols_(arg.sparsityInputIntermediateCols_),
      sparsityTerminalRows_(arg.sparsityTerminalRows_),
      sparsityStateTerminalRows_(arg.sparsityStateTerminalRows_),
      sparsityStateTerminalCols_(arg.sparsityStateTerminalCols_),
      sparsityInputTerminalRows_(arg.sparsityInputTerminalRows_),
      sparsityInputTerminalCols_(arg.sparsityInputTerminalCols_),
      stateControlD_(arg.stateControlD_)
{
    constraintsIntermediate_.resize(arg.constraintsIntermediate_.size());
    constraintsTerminal_.resize(arg.constraintsTerminal_.size());

    for (size_t i = 0; i < constraintsIntermediate_.size(); ++i)
        constraintsIntermediate_[i] =
            std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.constraintsIntermediate_[i]->clone());

    for (size_t i = 0; i < constraintsTerminal_.size(); ++i)
        constraintsTerminal_[i] =
            std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.constraintsTerminal_[i]->clone());

    intermediateCodegen_ =
        std::shared_ptr<JacCG>(new JacCG(fIntermediate_, STATE_DIM + CONTROL_DIM, getIntermediateConstraintsCount()));
    terminalCodegen_ =
        std::shared_ptr<JacCG>(new JacCG(fTerminal_, STATE_DIM + CONTROL_DIM, getTerminalConstraintsCount()));

    // make sure libraries get compiled in clone
    initializeIntermediate();
    initializeTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAD_Raw_Ptr_t
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new ConstraintContainerAD(*this);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstraintContainerAD()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateConstraint(
    std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
    bool verbose)
{
    constraintsIntermediate_.push_back(constraint);
    if (verbose)
    {
        std::string name;
        constraint->getName(name);
        std::cout << "''" << name << "'' added as AD intermediate constraint " << std::endl;
    }

    fIntermediate_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput) {
        return this->evaluateIntermediateCodegen(stateinput);
    };

    intermediateCodegen_->update(fIntermediate_, STATE_DIM + CONTROL_DIM, getIntermediateConstraintsCount());

    this->initializedIntermediate_ = false;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::addTerminalConstraint(
    std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
    bool verbose)
{
    constraintsTerminal_.push_back(constraint);
    if (verbose)
    {
        std::string name;
        constraint->getName(name);
        std::cout << "''" << name << "'' added as AD terminal constraint " << std::endl;
    }

    fTerminal_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput) {
        return this->evaluateTerminalCodegen(stateinput);
    };

    terminalCodegen_->update(fTerminal_, STATE_DIM + CONTROL_DIM, getTerminalConstraintsCount());

    this->initializedTerminal_ = false;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("evaluateIntermediateConstraints not initialized yet. Call 'initialize()' before");

    return intermediateCodegen_->forwardZero(stateControlD_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("evaluateTerminalConstraints not initialized yet. Call 'initialize()' before");

    return terminalCodegen_->forwardZero(stateControlD_);
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateConstraintsCount()
{
    size_t count = 0;

    for (auto constraint : constraintsIntermediate_)
        count += constraint->getConstraintSize();

    return count;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getTerminalConstraintsCount()
{
    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
        count += constraint->getConstraintSize();

    return count;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = intermediateCodegen_->jacobian(stateControlD_);

    // std::cout << "jacTot" << std::endl;

    VectorXs jacSparse;
    jacSparse.resize(getJacobianStateNonZeroCountIntermediate());
    for (size_t i = 0; i < getJacobianStateNonZeroCountIntermediate(); ++i)
        jacSparse(i) = (jacTot.template leftCols<STATE_DIM>())(
            sparsityStateIntermediateRows_(i), sparsityStateIntermediateCols_(i));

    return jacSparse;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = intermediateCodegen_->jacobian(stateControlD_);
    return jacTot.template leftCols<STATE_DIM>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = terminalCodegen_->jacobian(stateControlD_);
    VectorXs jacSparse;
    jacSparse.resize(getJacobianStateNonZeroCountTerminal());
    for (size_t i = 0; i < getJacobianStateNonZeroCountTerminal(); ++i)
        jacSparse(i) =
            (jacTot.template leftCols<STATE_DIM>())(sparsityStateTerminalRows_(i), sparsityStateTerminalCols_(i));

    return jacSparse;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = terminalCodegen_->jacobian(stateControlD_);
    return jacTot.template leftCols<STATE_DIM>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseIntermediate()
{
    MatrixXs jacTot = intermediateCodegen_->jacobian(stateControlD_);
    VectorXs jacSparse;
    jacSparse.resize(getJacobianInputNonZeroCountIntermediate());
    for (size_t i = 0; i < getJacobianInputNonZeroCountIntermediate(); ++i)
        jacSparse(i) = (jacTot.template rightCols<CONTROL_DIM>())(
            sparsityInputIntermediateRows_(i), sparsityInputIntermediateCols_(i));

    return jacSparse;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = intermediateCodegen_->jacobian(stateControlD_);
    return jacTot.template rightCols<CONTROL_DIM>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = terminalCodegen_->jacobian(stateControlD_);
    VectorXs jacSparse;
    jacSparse.resize(getJacobianInputNonZeroCountTerminal());
    for (size_t i = 0; i < getJacobianInputNonZeroCountTerminal(); ++i)
        jacSparse(i) =
            (jacTot.template rightCols<CONTROL_DIM>())(sparsityInputTerminalRows_(i), sparsityInputTerminalCols_(i));

    return jacSparse;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    MatrixXs jacTot = terminalCodegen_->jacobian(stateControlD_);
    return jacTot.template rightCols<CONTROL_DIM>();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateIntermediate(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    iRows = sparsityStateIntermediateRows_;
    jCols = sparsityStateIntermediateCols_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateTerminal(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    iRows = sparsityStateTerminalRows_;
    jCols = sparsityStateTerminalCols_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputIntermediate(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    iRows = sparsityInputIntermediateRows_;
    jCols = sparsityInputIntermediateCols_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputTerminal(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    iRows = sparsityInputTerminalRows_;
    jCols = sparsityInputTerminalCols_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    return sparsityStateIntermediateRows_.rows();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    return sparsityStateTerminalRows_.rows();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountIntermediate()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    return sparsityInputIntermediateRows_.rows();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountTerminal()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Constraints not initialized yet. Call 'initialize()' before");

    return sparsityInputTerminalRows_.rows();
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::initializeIntermediate()
{
    Eigen::VectorXi sparsityRows;
    Eigen::VectorXi sparsityCols;

    if (getIntermediateConstraintsCount() > 0)
    {
        ct::core::DerivativesCppadSettings settings;
        settings.createForwardZero_ = true;
        settings.createJacobian_ = true;
        settings.createSparseJacobian_ = true;

        intermediateCodegen_->compileJIT(settings, "intermediateConstraints");
        intermediateCodegen_->getSparsityPatternJacobian(sparsityRows, sparsityCols);

        std::cout << "sparsityPattern Intermediate: " << std::endl
                  << intermediateCodegen_->getSparsityPatternJacobian() << std::endl;
        assert(sparsityRows.rows() == sparsityRows.rows());

        int nonZerosState = (sparsityCols.array() < STATE_DIM).count();
        int nonZerosInput = (sparsityCols.array() >= STATE_DIM).count();

        sparsityStateIntermediateRows_.resize(nonZerosState);
        sparsityStateIntermediateCols_.resize(nonZerosState);
        sparsityInputIntermediateRows_.resize(nonZerosInput);
        sparsityInputIntermediateCols_.resize(nonZerosInput);

        size_t count = 0;

        this->lowerBoundsIntermediate_.resize(getIntermediateConstraintsCount());
        this->upperBoundsIntermediate_.resize(getIntermediateConstraintsCount());

        for (auto constraint : constraintsIntermediate_)
        {
            size_t constraintSize = constraint->getConstraintSize();
            this->lowerBoundsIntermediate_.segment(count, constraintSize) = constraint->getLowerBound();
            this->upperBoundsIntermediate_.segment(count, constraintSize) = constraint->getUpperBound();
            count += constraintSize;
        }

        size_t stateIndex = 0;
        size_t inputIndex = 0;

        for (int i = 0; i < sparsityRows.rows(); ++i)
        {
            if (sparsityCols(i) < static_cast<int>(STATE_DIM))
            {
                sparsityStateIntermediateRows_(stateIndex) = sparsityRows(i);
                sparsityStateIntermediateCols_(stateIndex) = sparsityCols(i);
                stateIndex++;
            }
            else
            {
                sparsityInputIntermediateRows_(inputIndex) = sparsityRows(i);
                sparsityInputIntermediateCols_(inputIndex) = sparsityCols(i) - STATE_DIM;
                inputIndex++;
            }
        }
    }

    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::initializeTerminal()
{
    Eigen::VectorXi sparsityRows;
    Eigen::VectorXi sparsityCols;

    if (getTerminalConstraintsCount() > 0)
    {
        ct::core::DerivativesCppadSettings settings;
        settings.createForwardZero_ = true;
        settings.createJacobian_ = true;
        settings.createSparseJacobian_ = true;

        terminalCodegen_->compileJIT(settings, "terminalConstraints");
        terminalCodegen_->getSparsityPatternJacobian(sparsityRows, sparsityCols);

        std::cout << "sparsityPattern Terminal: " << std::endl
                  << terminalCodegen_->getSparsityPatternJacobian() << std::endl;
        assert(sparsityRows.rows() == sparsityRows.rows());

        int nonZerosState = (sparsityCols.array() < STATE_DIM).count();
        int nonZerosInput = (sparsityCols.array() >= STATE_DIM).count();

        sparsityStateTerminalRows_.resize(nonZerosState);
        sparsityStateTerminalCols_.resize(nonZerosState);
        sparsityInputTerminalRows_.resize(nonZerosInput);
        sparsityInputTerminalCols_.resize(nonZerosInput);

        size_t count = 0;

        this->lowerBoundsTerminal_.resize(getTerminalConstraintsCount());
        this->upperBoundsTerminal_.resize(getTerminalConstraintsCount());

        for (auto constraint : constraintsTerminal_)
        {
            size_t constraintSize = constraint->getConstraintSize();
            this->lowerBoundsTerminal_.segment(count, constraintSize) = constraint->getLowerBound();
            this->upperBoundsTerminal_.segment(count, constraintSize) = constraint->getUpperBound();
            count += constraintSize;
        }

        size_t stateIndex = 0;
        size_t inputIndex = 0;

        for (int i = 0; i < sparsityRows.rows(); ++i)
        {
            if (sparsityCols(i) < static_cast<int>(STATE_DIM))
            {
                sparsityStateTerminalRows_(stateIndex) = sparsityRows(i);
                sparsityStateTerminalCols_(stateIndex) = sparsityCols(i);
                stateIndex++;
            }
            else
            {
                sparsityInputTerminalRows_(inputIndex) = sparsityRows(i);
                sparsityInputTerminalCols_(inputIndex) = sparsityCols(i) - static_cast<int>(STATE_DIM);
                inputIndex++;
            }
        }
    }

    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::update()
{
    stateControlD_ << this->x_, this->u_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar, Eigen::Dynamic, 1>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediateCodegen(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput)
{
    size_t count = 0;
    Eigen::Matrix<CGScalar, Eigen::Dynamic, 1> gLocal;

    for (auto constraint : constraintsIntermediate_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        gLocal.conservativeResize(count + constraint_dim);
        gLocal.segment(count, constraint_dim) = constraint->evaluateCppadCg(
            stateinput.segment(0, STATE_DIM), stateinput.segment(STATE_DIM, CONTROL_DIM), CGScalar(0.0));
        count += constraint_dim;
    }
    return gLocal;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
Eigen::Matrix<typename ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar, Eigen::Dynamic, 1>
ConstraintContainerAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminalCodegen(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM, 1>& stateinput)
{
    size_t count = 0;
    Eigen::Matrix<CGScalar, Eigen::Dynamic, 1> gLocal;

    for (auto constraint : constraintsTerminal_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        gLocal.conservativeResize(count + constraint_dim);
        gLocal.segment(count, constraint_dim) = constraint->evaluateCppadCg(
            stateinput.segment(0, STATE_DIM), stateinput.segment(STATE_DIM, CONTROL_DIM), CGScalar(0.0));
        count += constraint_dim;
    }

    return gLocal;
}

}  // namespace optcon
}  // namespace ct

#endif
