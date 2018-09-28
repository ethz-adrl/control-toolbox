#pragma once

namespace ct {
namespace rbd {

template <typename KINEMATICS, size_t NJOINTS>
class IKNLP : public ct::optcon::Nlp
{
public:
    IKNLP(std::shared_ptr<ct::rbd::IKCostEvaluator> costEvaluator){
    	this->optVariables_ = std::shared_ptr<ct::optcon::OptVector>(new ct::optcon::OptVector(NJOINTS));
    	this->costEvaluator_ = costEvauator;
    }

    virtual void updateProblem() override
    {
        // todo implement
    }

    virtual ~IKNLP() = default;

    void setInitialGuess(const ct::rbd::JointState<NJOINTS>::Position& q_init)
    {
        this->optVariables_->setInitGuess(x_init_guess, u_init_guess);
    }

private:
};

}  // rbd
}  // ct
