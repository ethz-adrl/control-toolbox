/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAnalytical()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAnalytical(const state_vector_t& x,
    const input_vector_t& u,
    const SCALAR& t)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAnalytical(
    const ConstraintContainerAnalytical& arg)
    : LinearConstraintContainer<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      constraintsIntermediate_(arg.constraintsIntermediate_),
      constraintsTerminal_(arg.constraintsTerminal_),
      evalIntermediate_(arg.evalIntermediate_),
      evalJacSparseStateIntermediate_(arg.evalJacSparseStateIntermediate_),
      evalJacSparseInputIntermediate_(arg.evalJacSparseInputIntermediate_),
      evalJacDenseStateIntermediate_(arg.evalJacDenseStateIntermediate_),
      evalJacDenseInputIntermediate_(arg.evalJacDenseInputIntermediate_),
      evalTerminal_(arg.evalTerminal_),
      evalJacSparseStateTerminal_(arg.evalJacSparseStateTerminal_),
      evalJacSparseInputTerminal_(arg.evalJacSparseInputTerminal_),
      evalJacDenseStateTerminal_(arg.evalJacDenseStateTerminal_),
      evalJacDenseInputTerminal_(arg.evalJacDenseInputTerminal_)
{
    // vectors of terms can be resized easily
    constraintsIntermediate_.resize(arg.constraintsIntermediate_.size());
    constraintsTerminal_.resize(arg.constraintsTerminal_.size());

    for (size_t i = 0; i < constraintsIntermediate_.size(); ++i)
        constraintsIntermediate_[i] =
            std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.constraintsIntermediate_[i]->clone());

    for (size_t i = 0; i < constraintsTerminal_.size(); ++i)
        constraintsTerminal_[i] =
            std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>>(arg.constraintsTerminal_[i]->clone());
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::ConstraintContainerAnalytical_Raw_Ptr_t
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::~ConstraintContainerAnalytical()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateConstraint(
    std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
    bool verbose)
{
    constraintsIntermediate_.push_back(constraint);
    if (verbose)
    {
        std::string name;
        constraint->getName(name);
        std::cout << "''" << name << "'' added as Analytical intermediate constraint " << std::endl;
    }
    this->initializedIntermediate_ = false;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::addTerminalConstraint(
    std::shared_ptr<ConstraintBase<STATE_DIM, CONTROL_DIM, SCALAR>> constraint,
    bool verbose)
{
    constraintsTerminal_.push_back(constraint);
    if (verbose)
    {
        std::string name;
        constraint->getName(name);
        std::cout << "''" << name << "'' added as Analytical terminal constraint " << std::endl;
    }
    this->initializedTerminal_ = false;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalIntermediate_.segment(count, constraint_dim) = constraint->evaluate(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }
    return evalIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    checkTerminalConstraints();

    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalTerminal_.segment(count, constraint_dim) = constraint->evaluate(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }
    return evalTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateConstraintsCount()
{
    size_t count = 0;

    for (auto constraint : constraintsIntermediate_)
        count += constraint->getConstraintSize();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getTerminalConstraintsCount()
{
    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
        count += constraint->getConstraintSize();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseIntermediate()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
    {
        size_t nonZerosState = constraint->getNumNonZerosJacobianState();

        if (nonZerosState != 0)
        {
            evalJacSparseStateIntermediate_.segment(count, nonZerosState) =
                constraint->jacobianStateSparse(this->x_, this->u_, this->t_);
            count += nonZerosState;
        }
    }
    return evalJacSparseStateIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateIntermediate()
{
    checkIntermediateConstraints();

    MatrixXs jacLocal;
    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalJacDenseStateIntermediate_.block(count, 0, constraint_dim, STATE_DIM) =
            constraint->jacobianState(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }

    return evalJacDenseStateIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateSparseTerminal()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
    {
        size_t nonZerosState = constraint->getNumNonZerosJacobianState();

        if (nonZerosState != 0)
        {
            evalJacSparseStateTerminal_.segment(count, nonZerosState) =
                constraint->jacobianStateSparse(this->x_, this->u_, this->t_);
            count += nonZerosState;
        }
    }
    return evalJacSparseStateTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianStateTerminal()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalJacDenseStateTerminal_.block(count, 0, constraint_dim, STATE_DIM) =
            constraint->jacobianState(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }

    return evalJacDenseStateTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseIntermediate()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
    {
        size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();

        if (nonZerosInput != 0)
        {
            evalJacSparseInputIntermediate_.segment(count, nonZerosInput) =
                constraint->jacobianInputSparse(this->x_, this->u_, this->t_);
            count += nonZerosInput;
        }
    }
    return evalJacSparseInputIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputIntermediate()
{
    checkIntermediateConstraints();

    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalJacDenseInputIntermediate_.block(count, 0, constraint_dim, CONTROL_DIM) =
            constraint->jacobianInput(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }

    return evalJacDenseInputIntermediate_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::VectorXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputSparseTerminal()
{
    checkTerminalConstraints();

    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
    {
        size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();

        if (nonZerosInput != 0)
        {
            evalJacSparseInputTerminal_.segment(count, nonZerosInput) =
                constraint->jacobianInputSparse(this->x_, this->u_, this->t_);
            count += nonZerosInput;
        }
    }
    return evalJacSparseInputTerminal_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixXs
ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::jacobianInputTerminal()
{
    checkTerminalConstraints();

    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
    {
        size_t constraint_dim = constraint->getConstraintSize();
        evalJacDenseInputTerminal_.block(count, 0, constraint_dim, CONTROL_DIM) =
            constraint->jacobianInput(this->x_, this->u_, this->t_);
        count += constraint_dim;
    }

    return evalJacDenseInputTerminal_;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateIntermediate(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    checkIntermediateConstraints();

    Eigen::VectorXi iRowLocal;
    Eigen::VectorXi jColLocal;
    Eigen::VectorXi iRowTot;
    Eigen::VectorXi jColTot;

    size_t count = 0;
    size_t constraintCount = 0;

    for (auto constraint : constraintsIntermediate_)
    {
        size_t nonZerosState = constraint->getNumNonZerosJacobianState();
        if (nonZerosState > 0)
        {
            constraint->sparsityPatternState(iRowLocal, jColLocal);

            iRowTot.conservativeResize(count + nonZerosState);
            iRowTot.segment(count, nonZerosState) = iRowLocal.array() + constraintCount;

            jColTot.conservativeResize(count + nonZerosState);
            jColTot.segment(count, nonZerosState) = jColLocal;

            count += nonZerosState;
        }
        constraintCount += constraint->getConstraintSize();
    }

    iRows = iRowTot;
    jCols = jColTot;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternStateTerminal(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedTerminal_)
        throw std::runtime_error(
            "sparsityPatternStateTerminalConstraints not initialized yet. Call 'initialize()' before");

    Eigen::VectorXi iRowLocal;
    Eigen::VectorXi jColLocal;
    Eigen::VectorXi iRowTot;
    Eigen::VectorXi jColTot;

    size_t count = 0;
    size_t constraintCount = 0;

    for (auto constraint : constraintsTerminal_)
    {
        size_t nonZerosState = constraint->getNumNonZerosJacobianState();
        if (nonZerosState > 0)
        {
            constraint->sparsityPatternState(iRowLocal, jColLocal);

            iRowTot.conservativeResize(count + nonZerosState);
            iRowTot.segment(count, nonZerosState) = iRowLocal.array() + constraintCount;

            jColTot.conservativeResize(count + nonZerosState);
            jColTot.segment(count, nonZerosState) = jColLocal;

            count += nonZerosState;
        }
        constraintCount += constraint->getConstraintSize();
    }

    iRows = iRowTot;
    jCols = jColTot;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputIntermediate(
    Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    checkIntermediateConstraints();

    Eigen::VectorXi iRowLocal;
    Eigen::VectorXi jColLocal;
    Eigen::VectorXi iRowTot;
    Eigen::VectorXi jColTot;

    size_t count = 0;
    size_t constraintCount = 0;

    for (auto constraint : constraintsIntermediate_)
    {
        size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();
        if (nonZerosInput > 0)
        {
            constraint->sparsityPatternInput(iRowLocal, jColLocal);

            iRowTot.conservativeResize(count + nonZerosInput);
            iRowTot.segment(count, nonZerosInput) = iRowLocal.array() + constraintCount;

            jColTot.conservativeResize(count + nonZerosInput);
            jColTot.segment(count, nonZerosInput) = jColLocal;

            count += nonZerosInput;
        }
        constraintCount += constraint->getConstraintSize();
    }

    iRows = iRowTot;
    jCols = jColTot;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::sparsityPatternInputTerminal(Eigen::VectorXi& iRows,
    Eigen::VectorXi& jCols)
{
    if (!this->initializedTerminal_)
        throw std::runtime_error(
            "sparsityPatternInputTerminalConstraints not initialized yet. Call 'initialize()' before");

    Eigen::VectorXi iRowLocal;
    Eigen::VectorXi jColLocal;
    Eigen::VectorXi iRowTot;
    Eigen::VectorXi jColTot;

    size_t count = 0;
    size_t constraintCount = 0;

    for (auto constraint : constraintsTerminal_)
    {
        size_t nonZerosInput = constraint->getNumNonZerosJacobianInput();
        if (nonZerosInput > 0)
        {
            constraint->sparsityPatternInput(iRowLocal, jColLocal);

            iRowTot.conservativeResize(count + nonZerosInput);
            iRowTot.segment(count, nonZerosInput) = iRowLocal.array() + constraintCount;

            jColTot.conservativeResize(count + nonZerosInput);
            jColTot.segment(count, nonZerosInput) = jColLocal;

            count += nonZerosInput;
        }
        constraintCount += constraint->getConstraintSize();
    }

    iRows = iRowTot;
    jCols = jColTot;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountIntermediate()
{
    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
        count += constraint->getNumNonZerosJacobianState();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianStateNonZeroCountTerminal()
{
    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
        count += constraint->getNumNonZerosJacobianState();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountIntermediate()
{
    size_t count = 0;
    for (auto constraint : constraintsIntermediate_)
        count += constraint->getNumNonZerosJacobianInput();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
size_t ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::getJacobianInputNonZeroCountTerminal()
{
    size_t count = 0;
    for (auto constraint : constraintsTerminal_)
        count += constraint->getNumNonZerosJacobianInput();

    return count;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::initializeIntermediate()
{
    if (getIntermediateConstraintsCount() > 0)
    {
        evalIntermediate_.resize(getIntermediateConstraintsCount());
        evalIntermediate_.setZero();
        evalJacSparseStateIntermediate_.resize(getJacobianStateNonZeroCountIntermediate());
        evalJacSparseStateIntermediate_.setZero();
        evalJacSparseInputIntermediate_.resize(getJacobianInputNonZeroCountIntermediate());
        evalJacSparseInputIntermediate_.setZero();
        evalJacDenseStateIntermediate_.resize(getIntermediateConstraintsCount(), STATE_DIM);
        evalJacDenseStateIntermediate_.setZero();
        evalJacDenseInputIntermediate_.resize(getIntermediateConstraintsCount(), CONTROL_DIM);
        evalJacDenseInputIntermediate_.setZero();

        size_t count = 0;

        this->lowerBoundsIntermediate_.resize(getIntermediateConstraintsCount());
        this->lowerBoundsIntermediate_.setZero();
        this->upperBoundsIntermediate_.resize(getIntermediateConstraintsCount());
        this->upperBoundsIntermediate_.setZero();

        for (auto constraint : constraintsIntermediate_)
        {
            size_t constraintSize = constraint->getConstraintSize();
            this->lowerBoundsIntermediate_.segment(count, constraintSize) = constraint->getLowerBound();
            this->upperBoundsIntermediate_.segment(count, constraintSize) = constraint->getUpperBound();
            count += constraintSize;
        }
    }

    return true;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
bool ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::initializeTerminal()
{
    if (getTerminalConstraintsCount() > 0)
    {
        evalTerminal_.resize(getTerminalConstraintsCount());
        evalTerminal_.setZero();
        evalJacSparseStateTerminal_.resize(getJacobianStateNonZeroCountTerminal());
        evalJacSparseStateTerminal_.setZero();
        evalJacSparseInputTerminal_.resize(getJacobianInputNonZeroCountTerminal());
        evalJacSparseInputTerminal_.setZero();
        evalJacDenseStateTerminal_.resize(getTerminalConstraintsCount(), STATE_DIM);
        evalJacDenseStateTerminal_.setZero();
        evalJacDenseInputTerminal_.resize(getTerminalConstraintsCount(), CONTROL_DIM);
        evalJacDenseInputTerminal_.setZero();

        size_t count = 0;

        this->lowerBoundsTerminal_.resize(getTerminalConstraintsCount());
        this->lowerBoundsTerminal_.setZero();
        this->upperBoundsTerminal_.resize(getTerminalConstraintsCount());
        this->upperBoundsTerminal_.setZero();

        for (auto constraint : constraintsTerminal_)
        {
            size_t constraintSize = constraint->getConstraintSize();
            this->lowerBoundsTerminal_.segment(count, constraintSize) = constraint->getLowerBound();
            this->upperBoundsTerminal_.segment(count, constraintSize) = constraint->getUpperBound();
            count += constraintSize;
        }
    }

    return true;
}


template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::update()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::checkIntermediateConstraints()
{
    if (!this->initializedIntermediate_)
        throw std::runtime_error("Error: Intermediate constraints are or not initialized yet. ");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void ConstraintContainerAnalytical<STATE_DIM, CONTROL_DIM, SCALAR>::checkTerminalConstraints()
{
    if (!this->initializedTerminal_)
        throw std::runtime_error("Error: Terminal constraints are either not initialized yet. ");
}

}  // namespace optcon
}  // namespace ct
