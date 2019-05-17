/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/optcon/optcon.h>

namespace ct {
namespace optcon {


SnoptMemory::SnoptMemory(const SnoptSolver& self) : self(self)
{
    // Put in memory pool
    auto mem_it = std::find(mempool.begin(), mempool.end(), nullptr);
    if (mem_it == mempool.end())
    {
        // Append to end
        memind = mempool.size();
        mempool.push_back(this);
    }
    else
    {
        // Reuse freed element
        memind = mem_it - mempool.begin();
        *mem_it = this;
    }
}


SnoptMemory::~SnoptMemory()
{
    // Remove from memory pool
    auto mem_it = std::find(mempool.begin(), mempool.end(), this);
    if (mem_it == mempool.end())
    {
        // Should probably shut down program
        std::cout << "Error while destroying SNOPT memory" << std::endl;
    }
    else
    {
        delete[] iAfun_;
        delete[] jAvar_;
        delete[] A_;

        delete[] x_;
        delete[] xlow_;
        delete[] xupp_;
        delete[] xmul_;
        delete[] xstate_;

        delete[] F_;
        delete[] Flow_;
        delete[] Fupp_;
        delete[] Fmul_;
        delete[] Fstate_;

        delete[] iGfun_;
        delete[] jGvar_;

        *mem_it = nullptr;
    }
}


SnoptSolver::SnoptSolver(std::shared_ptr<Nlp> nlp, const NlpSolverSettings& settings)
    : BASE(nlp, settings), settings_(BASE::settings_.snoptSettings_), memoryPtr_(nullptr)
{
    memoryPtr_ = alloc_memory();
    currStartOption_ = Cold_;
}

SnoptSolver::~SnoptSolver()
{
    free_memory(memoryPtr_);
}


void SnoptSolver::init_memory(SnoptMemory* mem) const
{
    mem->x_ = new Number[n_];
    mem->xlow_ = new Number[n_];
    mem->xupp_ = new Number[n_];
    mem->xmul_ = new Number[n_];
    mem->xstate_ = new int[n_];

    mem->F_ = new Number[neF_];
    mem->Flow_ = new Number[neF_];
    mem->Fupp_ = new Number[neF_];
    mem->Fmul_ = new Number[neF_];
    mem->Fstate_ = new int[neF_];

    mem->iAfun_ = new int[lenA_];
    mem->jAvar_ = new int[lenA_];
    mem->A_ = new Number[lenA_];

    mem->iGfun_ = new int[lenG_];
    mem->jGvar_ = new int[lenG_];
}


void SnoptSolver::fill_memory(SnoptMemory* mem) const
{
    MapVecXd xVec(mem->x_, n_);
    MapVecXd x_lVec(mem->xlow_, n_);
    MapVecXd x_uVec(mem->xupp_, n_);
    MapVecXd xMulVec(mem->xmul_, n_);
    MapVecXi xStateVec(mem->xstate_, n_);

    MapVecXd FVec(mem->F_, neF_);
    MapVecXd f_lVec(&mem->Flow_[1], nlp_->getConstraintsCount());  // map from the second element
    MapVecXd f_uVec(&mem->Fupp_[1], nlp_->getConstraintsCount());  // map from the second element
    MapVecXd fMulVec(mem->Fmul_, neF_);
    MapVecXi FStateVec(mem->Fstate_, neF_);

    MapVecXi iAVec(mem->iAfun_, lenA_);
    MapVecXi jAVec(mem->jAvar_, lenA_);
    MapVecXd AVec(mem->A_, lenA_);

    MapVecXi iGVecFull(mem->iGfun_, lenG_);
    MapVecXi jGVecFull(mem->jGvar_, lenG_);
    MapVecXi iGVecJac(&mem->iGfun_[n_], nlp_->getNonZeroJacobianCount());
    MapVecXi jGVecGrad(mem->jGvar_, n_);
    MapVecXi jGVecJac(&mem->jGvar_[n_], nlp_->getNonZeroJacobianCount());

    nlp_->getInitialGuess(n_, xVec);
    nlp_->getVariableBounds(x_lVec, x_uVec, n_);
    nlp_->getOptimizationMultState(n_, xMulVec, xStateVec);

    FVec.setZero();
    mem->Flow_[0] = std::numeric_limits<double>::lowest();
    mem->Fupp_[0] = std::numeric_limits<double>::max();
    nlp_->getConstraintBounds(f_lVec, f_uVec, nlp_->getConstraintsCount());
    nlp_->getConstraintsMultState(neF_, fMulVec, FStateVec);

    iAVec.setZero();
    jAVec.setZero();
    AVec.setZero();

    iGVecFull.setZero();
    jGVecFull.setZero();

    jGVecGrad = Eigen::VectorXi::LinSpaced(n_, 0, n_ - 1);
    nlp_->getSparsityPatternJacobian(nlp_->getNonZeroJacobianCount(), iGVecJac, jGVecJac);
    iGVecJac += Eigen::VectorXi::Ones(nlp_->getNonZeroJacobianCount());
}


void SnoptSolver::configureDerived(const NlpSolverSettings& settings)
{
    std::cout << "calling SNOPT configure derived" << std::endl;
    settings_ = settings.snoptSettings_;

    n_ = nlp_->getVarCount();
    neF_ = nlp_->getConstraintsCount() + 1;

    lenA_ = 0;
    neA_ = 0;

    lenG_ = n_ + nlp_->getNonZeroJacobianCount();
    neG_ = n_ + nlp_->getNonZeroJacobianCount();

    init_memory(memoryPtr_);
    setSolverOptions();
    BASE::isInitialized_ = true;
}


void SnoptSolver::setupSnoptObjects()
{
    snoptApp_.setProblemSize(n_, neF_);
    snoptApp_.setObjective(ObjRow_, ObjAdd_);
    snoptApp_.setX(memoryPtr_->x_, memoryPtr_->xlow_, memoryPtr_->xupp_, memoryPtr_->xmul_, memoryPtr_->xstate_);
    snoptApp_.setF(memoryPtr_->F_, memoryPtr_->Flow_, memoryPtr_->Fupp_, memoryPtr_->Fmul_, memoryPtr_->Fstate_);
    snoptApp_.setA(lenA_, neA_, memoryPtr_->iAfun_, memoryPtr_->jAvar_, memoryPtr_->A_);
    snoptApp_.setG(lenG_, neG_, memoryPtr_->iGfun_, memoryPtr_->jGvar_);
    snoptApp_.setUserFun(NLP_Function);
}


void SnoptSolver::setSolverOptions()
{
    snoptApp_.setIntParameter("Derivative option", settings_.derivative_option_param_);
    snoptApp_.setIntParameter("Verify level", settings_.verify_level_param_);
    snoptApp_.setIntParameter("Major iterations limit", settings_.major_iteration_limit_param_);
    snoptApp_.setIntParameter("Minor iterations limit", settings_.minor_iteration_limit_param_);
    snoptApp_.setIntParameter("Iterations limit", settings_.iteration_limit_param_);
    snoptApp_.setRealParameter("Major optimality tolerance", settings_.major_optimality_tolerance_param_);
    snoptApp_.setRealParameter("Major feasibility tolerance", settings_.major_feasibility_tolerance_param_);
    snoptApp_.setRealParameter("Minor feasibility tolerance", settings_.minor_feasibility_tolerance_param_);
    snoptApp_.setIntParameter("Print file", settings_.print_file_param_);
    snoptApp_.setIntParameter("Minor print level", settings_.minor_print_level_param_);
    snoptApp_.setIntParameter("Major print level", settings_.major_print_level_param_);

    snoptApp_.setIntParameter("New basis file", settings_.new_basis_file_param_);
    snoptApp_.setIntParameter("Old basis file", settings_.old_basis_file_param_);
    snoptApp_.setIntParameter("Backup basis file", settings_.backup_basis_file_param_);
    snoptApp_.setRealParameter("Linesearch tolerance", settings_.line_search_tolerance_param_);
    snoptApp_.setIntParameter("Crash option", settings_.crash_option_);
    snoptApp_.setIntParameter("Hessian updates", settings_.hessian_updates_);
}


bool SnoptSolver::solve()
{
    if (!this->isInitialized_)
        throw std::runtime_error("SNOPT was not initialized before. Call NLPSolver->configure()");

    snoptApp_.setUserI(&(memoryPtr_->memind), 1);
    std::cout << "Ready to solve... " << std::endl;
    fill_memory(memoryPtr_);
    setupSnoptObjects();
    int status = snoptApp_.solve(currStartOption_);
    if (status == 1 || status == 31 || status == 32)  //Optimal Solution Found or Iteration limit reached
    {
        // Store solution to optvector
        MapVecXd xVec(memoryPtr_->x_, n_);
        MapVecXd xMulVec(memoryPtr_->xmul_, n_);
        MapVecXi xStateVec(memoryPtr_->xstate_, n_);
        MapVecXd fMulVec(memoryPtr_->Fmul_, neF_);
        MapVecXi fStateVec(memoryPtr_->Fstate_, neF_);
        nlp_->extractSnoptSolution(xVec, xMulVec, xStateVec, fMulVec, fStateVec);
    }

    return true;
}


void SnoptSolver::prepareWarmStart(size_t maxIterations)
{
    currStartOption_ = Warm_;
    Eigen::Map<Eigen::VectorXi> xState_local(memoryPtr_->xstate_, n_);
    Eigen::Map<Eigen::VectorXi> Fstate_local(memoryPtr_->Fstate_, neF_);
    Fstate_local.setConstant(3);
    xState_local.setConstant(3);
    snoptApp_.setIntParameter("Major iterations limit", (int)maxIterations);
    snoptApp_.setIntParameter("Minor iterations limit", 50);
    snoptApp_.setIntParameter("Iterations limit", 500);
    snoptApp_.setIntParameter("Summary file", 0);
}


void SnoptSolver::validateSNOPTStatus(const int& status) const
{
    std::string message;
    switch (status)
    {
        case 2:
        {
            message = "Optimal Solution Found";
            break;
        }
        case 3:
            message = "Problem is Infeasible";
            break;
        case 4:
            message = "Unbounded";
            break;
        case 5:
            message = "Iteration Limit";
            break;
        case 6:
            message = "Numerical Difficulties";
            break;
        case 7:
            message = "Error in user supplied functions";
            break;
        case 8:
            message = "Undefined user supplied functions";
            break;
        case 9:
            message = "User requested termination";
            break;
        case 10:
            message = "Insufficient storage allocated";
            break;
        case 11:
            message = "Input arguments out of range";
            break;
        case 12:
            message = "System error";
            break;
        default:
            message = "Unknown error";
            break;
    }
    std::cout << message << std::endl;
}


void SnoptSolver::NLP_Function(int* Status,
    int* n,
    double x[],
    int* needF,
    int* neF,
    double F[],
    int* needG,
    int* neG,
    double G[],
    char* cu,
    int* lencu,
    int iu[],
    int* leniu,
    double ru[],
    int* lenru)
{
    auto m = SnoptMemory::mempool.at(iu[0]);
    m->self.NLP_Function(m, Status, n, x, needF, neF, F, needG, neG, G, cu, lencu, iu, leniu, ru, lenru);
}


void SnoptSolver::NLP_Function(SnoptMemory* m,
    int* Status,
    int* n,
    double x[],
    int* needF,
    int* neF,
    double F[],
    int* needG,
    int* neG,
    double G[],
    char* cu,
    int* lencu,
    int iu[],
    int* leniu,
    double ru[],
    int* lenru) const
{
    if (*Status > 1)
    {
        m->self.validateSNOPTStatus(*Status);
    }

    if (*needF > 0 || needG > 0)
    {
        Eigen::Map<const Eigen::VectorXd> xVec(&x[0], *n);
        m->self.nlp_->extractOptimizationVars(xVec, true);
    }

    if (*needF > 0)
    {
        F[0] = m->self.nlp_->evaluateCostFun();
        MapVecXd fVec(&F[1], *neF - 1);
        m->self.nlp_->evaluateConstraints(fVec);
    }

    if (*needG > 0)
    {
        size_t grad_len = m->self.nlp_->getVarCount();
        MapVecXd gradVec(&G[0], grad_len);
        m->self.nlp_->evaluateCostGradient(grad_len, gradVec);
        MapVecXd jacVec(&G[grad_len], *neG - grad_len);
        m->self.nlp_->evaluateConstraintJacobian(*neG - grad_len, jacVec);
    }
}

std::vector<SnoptMemory*> SnoptMemory::mempool;

}  // namespace optcon
}  // namespace ct
