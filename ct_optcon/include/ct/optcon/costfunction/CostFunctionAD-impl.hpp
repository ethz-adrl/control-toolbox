/**********************************************************************************************************************
 This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
  Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

#ifdef CPPADCG

namespace ct {
namespace optcon {

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::CostFunctionAD()
    : Base(), stateControlTime_(Eigen::Matrix<SCALAR_EVAL, STATE_DIM + CONTROL_DIM + 1, 1>::Zero())
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

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::CostFunctionAD(const CostFunctionAD& arg)
    : Base(arg),
      stateControlTime_(arg.stateControlTime_),
      intermediateFun_(arg.intermediateFun_),
      finalFun_(arg.finalFun_)
{
    intermediateTerms_.resize(arg.intermediateTerms_.size());
    finalTerms_.resize(arg.finalTerms_.size());

    for (size_t i = 0; i < intermediateTerms_.size(); ++i)
        intermediateTerms_[i] = std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>>(arg.intermediateTerms_[i]->clone());

    for (size_t i = 0; i < finalTerms_.size(); ++i)
        finalTerms_[i] = std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>>(arg.finalTerms_[i]->clone());

    intermediateCostCodegen_ = std::shared_ptr<JacCG>(arg.intermediateCostCodegen_->clone());
    finalCostCodegen_ = std::shared_ptr<JacCG>(arg.finalCostCodegen_->clone());
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::CostFunctionAD(const std::string& filename, bool verbose)
    : CostFunctionAD()  //! @warning the delegating constructor in the initializer list is required to call the initial routine in CostFunctionAD()
{
    loadFromConfigFile(filename, verbose);
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::~CostFunctionAD()
{
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>* CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::clone() const
{
    return new CostFunctionAD(*this);
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::initialize()
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

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::addIntermediateADTerm(
    std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> term,
    bool verbose)
{
    intermediateTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as intermediate AD term" << std::endl;
    }
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::addFinalADTerm(std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> term,
    bool verbose)
{
    finalTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as final AD term" << std::endl;
    }
}

// set state and control
template <typename AD_MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::setCurrentStateAndControl(const EVAL_MANIFOLD& x,
    const control_vector_t& u,
    const SCALAR_EVAL& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    stateControlTime_ << x, u, t;
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
void CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::loadFromConfigFile(const std::string& filename, bool verbose)
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
        } catch (boost::property_tree::ptree_bad_path& err)
        {
            termName = "Unnamed";
            if (verbose)
            {
                std::cout << "Name field for " + currentTerm + " does not exist" << std::endl;
            }
        }

        std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> term;

        CT_LOADABLE_TERMS(AD_MANIFOLD, CONTROL_DIM);

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

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::MatrixCg CostFunctionAD<AD_MANIFOLD,
    CONTROL_DIM>::evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : intermediateTerms_)
        y += it->evaluate(stateInputTime.segment(0, STATE_DIM), stateInputTime.segment(STATE_DIM, CONTROL_DIM),
            stateInputTime(STATE_DIM + CONTROL_DIM));

    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::MatrixCg CostFunctionAD<AD_MANIFOLD,
    CONTROL_DIM>::evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1, 1>& stateInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : finalTerms_)
        y += it->evaluate(stateInputTime.segment(0, STATE_DIM), stateInputTime.segment(STATE_DIM, CONTROL_DIM),
            stateInputTime(STATE_DIM + CONTROL_DIM));

    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::evaluateIntermediate() -> SCALAR_EVAL
{
    return this->evaluateIntermediateBase() + intermediateCostCodegen_->forwardZero(stateControlTime_)(0);
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::evaluateTerminal() -> SCALAR_EVAL
{
    return this->evaluateTerminalBase() + finalCostCodegen_->forwardZero(stateControlTime_)(0);
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateDerivativeIntermediate() -> typename EVAL_MANIFOLD::Tangent
{
    Eigen::Matrix<SCALAR_EVAL, 1, STATE_DIM + CONTROL_DIM + 1> jacTot =
        intermediateCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeIntermediateBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateDerivativeTerminal() -> typename EVAL_MANIFOLD::Tangent
{
    Eigen::Matrix<SCALAR_EVAL, 1, STATE_DIM + CONTROL_DIM + 1> jacTot = finalCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeTerminalBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::controlDerivativeIntermediate() -> control_vector_t
{
    Eigen::Matrix<SCALAR_EVAL, 1, STATE_DIM + CONTROL_DIM + 1> jacTot =
        intermediateCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose() + this->controlDerivativeIntermediateBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::controlDerivativeTerminal() -> control_vector_t
{
    Eigen::Matrix<SCALAR_EVAL, 1, STATE_DIM + CONTROL_DIM + 1> jacTot = finalCostCodegen_->jacobian(stateControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose() + this->controlDerivativeTerminalBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
auto CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateSecondDerivativeIntermediate() -> state_matrix_t
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeIntermediateBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::state_matrix_t
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeTerminalBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::controlSecondDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM) +
           this->controlSecondDerivativeIntermediateBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::control_matrix_t
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::controlSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM) +
           this->controlSecondDerivativeTerminalBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateControlDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0) + this->stateControlDerivativeIntermediateBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
typename CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::control_state_matrix_t
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::stateControlDerivativeTerminal()
{
    Eigen::Matrix<SCALAR_EVAL, 1, 1> w;
    w << SCALAR_EVAL(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0) + this->stateControlDerivativeTerminalBase();
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<ct::optcon::TermBase<AD_MANIFOLD, CONTROL_DIM>>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::getIntermediateADTermById(const size_t id)
{
    return intermediateTerms_[id];
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<ct::optcon::TermBase<AD_MANIFOLD, CONTROL_DIM>>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::getFinalADTermById(const size_t id)
{
    return finalTerms_[id];
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>>
CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::getIntermediateADTermByName(const std::string& name)
{
    for (auto term : intermediateTerms_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionAD");
}

template <typename AD_MANIFOLD, size_t CONTROL_DIM>
std::shared_ptr<TermBase<AD_MANIFOLD, CONTROL_DIM>> CostFunctionAD<AD_MANIFOLD, CONTROL_DIM>::getFinalADTermByName(
    const std::string& name)
{
    for (auto term : finalTerms_)
        if (term->getName() == name)
            return term;

    throw std::runtime_error("Term " + name + " not found in the CostFunctionAD");
}
}  // namespace optcon
}  // namespace ct

#endif
