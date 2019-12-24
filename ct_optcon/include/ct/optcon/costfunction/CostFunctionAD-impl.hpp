/**********************************************************************************************************************
 This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
  Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAD()
    : CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(),
      stateControlTime_(Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1, 1>::Zero())
{
    intermediateFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime) {
        return this->evaluateIntermediateCg(stateInputTime);
    };

    finalFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime) {
        return this->evaluateTerminalCg(stateInputTime);
    };

    intermediateCostCodegen_ = std::shared_ptr<JacCG>(new JacCG(intermediateFun_, STATE_DIM + CONTROL_DIM + 1, 1));
    finalCostCodegen_ = std::shared_ptr<JacCG>(new JacCG(finalFun_, STATE_DIM + CONTROL_DIM + 1, 1));

    setCurrentStateAndControl(this->x_, this->u_, this->t_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAD(const CostFunctionAD& arg)
    : CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      stateControlTime_(arg.stateControlTime_),
      intermediateFun_(arg.intermediateFun_),
      finalFun_(arg.finalFun_)
{
    intermediateTerms_.resize(arg.intermediateTerms_.size());
    finalTerms_.resize(arg.finalTerms_.size());

    for (size_t i = 0; i < intermediateTerms_.size(); ++i)
        intermediateTerms_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.intermediateTerms_[i]->clone());

    for (size_t i = 0; i < finalTerms_.size(); ++i)
        finalTerms_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.finalTerms_[i]->clone());

    intermediateCostCodegen_ = std::shared_ptr<JacCG>(arg.intermediateCostCodegen_->clone());
    finalCostCodegen_ = std::shared_ptr<JacCG>(arg.finalCostCodegen_->clone());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CostFunctionAD(const std::string& filename, bool verbose)
    : CostFunctionAD()  //! @warning the delegating constructor in the initializer list is required to call the initial routine in CostFunctionAD()
{
    loadFromConfigFile(filename, verbose);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::~CostFunctionAD()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>* CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new CostFunctionAD(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::initialize()
{
    intermediateFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime) {
        return this->evaluateIntermediateCg(stateInputTime);
    };

    finalFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime) {
        return this->evaluateTerminalCg(stateInputTime);
    };

    intermediateCostCodegen_->update(intermediateFun_, STATE_DIM + CONTROL_DIM + 1, 1);
    finalCostCodegen_->update(finalFun_, STATE_DIM + CONTROL_DIM + 1, 1);

    //! @ todo: this should probably become an option (eg. IPOPT can work without cost Hessians)
    ct::core::DerivativesCppadSettings settings;
    settings.createForwardZero_ = true;
    settings.createJacobian_ = true;
    settings.createHessian_ = true;

    finalCostCodegen_->compileJIT(settings, "finalCosts");
    intermediateCostCodegen_->compileJIT(settings, "intermediateCosts");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::addIntermediateADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    intermediateTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as intermediate AD term" << std::endl;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::addFinalADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    finalTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as final AD term" << std::endl;
    }
}

// set state and control
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    stateControlTime_ << x, u, t;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::loadFromConfigFile(const std::string& filename, bool verbose)
{
    this->intermediateCostAnalytical_.clear();
    this->finalCostAnalytical_.clear();

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    int i = 0;
    std::string currentTerm;
    do
    {
        currentTerm = "term" + std::to_string(i);
        std::string termKind = pt.get<std::string>(currentTerm + ".kind");
        boost::algorithm::to_lower(termKind);
        int currentTermType = pt.get<int>(currentTerm + ".type");
        std::string termName;
        try
        {
            termName = pt.get<std::string>(currentTerm + ".name");
        } catch (boost::property_tree::ptree_bad_path err)
        {
            termName = "Unnamed";
            if (verbose)
            {
                std::cout << "Name field for " + currentTerm + " does not exist" << std::endl;
            }
        }

        std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term;

        CT_LOADABLE_TERMS(SCALAR, CGScalar);

        if (!term)
        {
            throw std::runtime_error("Term type \"" + termKind + "\" not supported");
        }
        else
        {
            if (term)
                addADTerm(filename, currentTerm, currentTermType, term, this, verbose);
            else
                throw std::runtime_error("Term type \"" + termKind + "\" loaded but unsupported.");
        }
        currentTerm = "term" + std::to_string(++i);
    } while (pt.find(currentTerm) != pt.not_found());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixCg
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediateCg(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : intermediateTerms_)
        y += it->evaluateCppadCg(stateInputTime.segment(0, STATE_DIM), stateInputTime.segment(STATE_DIM, CONTROL_DIM),
            stateInputTime(STATE_DIM + CONTROL_DIM));

    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::MatrixCg
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminalCg(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : finalTerms_)
        y += it->evaluateCppadCg(stateInputTime.segment(0, STATE_DIM), stateInputTime.segment(STATE_DIM, CONTROL_DIM),
            stateInputTime(STATE_DIM + CONTROL_DIM));

    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateIntermediate()
{
    return this->evaluateIntermediateBase() + intermediateCostCodegen_->forwardZero(stateControlTime_)(0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
SCALAR CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::evaluateTerminal()
{
    return this->evaluateTerminalBase() + finalCostCodegen_->forwardZero(stateControlTime_)(0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1> jacTot =
        intermediateCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1> jacTot = finalCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1> jacTot =
        intermediateCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose() + this->controlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_vector_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::controlDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1> jacTot = finalCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose() + this->controlDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM) +
           this->controlSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::controlSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM) +
           this->controlSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0) + this->stateControlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::control_state_matrix_t
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::stateControlDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0) + this->stateControlDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<ct::optcon::
        TermBase<STATE_DIM, CONTROL_DIM, SCALAR, typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar>>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateADTermById(const size_t id)
{
    return intermediateTerms_[id];
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<ct::optcon::
        TermBase<STATE_DIM, CONTROL_DIM, SCALAR, typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar>>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::getFinalADTermById(const size_t id)
{
    return finalTerms_[id];
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<
    TermBase<STATE_DIM, CONTROL_DIM, SCALAR, typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar>>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::getIntermediateADTermByName(const std::string& name)
{
    for (auto term : intermediateTerms_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionAD");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
std::shared_ptr<
    TermBase<STATE_DIM, CONTROL_DIM, SCALAR, typename CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::CGScalar>>
CostFunctionAD<STATE_DIM, CONTROL_DIM, SCALAR>::getFinalADTermByName(const std::string& name)
{
    for (auto term : finalTerms_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionAD");
}
}  // namespace optcon
}  // namespace ct

#endif
