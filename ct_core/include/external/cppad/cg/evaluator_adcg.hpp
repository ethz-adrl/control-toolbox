#ifndef CPPAD_CG_EVALUATOR_ADCG_INCLUDED
#define CPPAD_CG_EVALUATOR_ADCG_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

/**
 * Specialization for an output active type of AD<CG<Base>>
 */
template<class ScalarIn, class BaseOut>
class Evaluator<ScalarIn, CG<BaseOut>, CppAD::AD<CG<BaseOut> > > : public EvaluatorAD<ScalarIn, CG<BaseOut>, Evaluator<ScalarIn, CG<BaseOut>, CppAD::AD<CG<BaseOut> > > > {
    /**
     * must be friends with one of its super classes since there is a cast to
     * this type due to the  curiously recurring template pattern (CRTP)
     */
    friend EvaluatorBase<ScalarIn, CG<BaseOut>, CppAD::AD<CG<BaseOut> >, Evaluator<ScalarIn, CG<BaseOut>, CppAD::AD<CG<BaseOut> > > >;
public:
    typedef CG<BaseOut> ScalarOut;
    typedef CppAD::AD<ScalarOut> ActiveOut;
    typedef EvaluatorAD<ScalarIn, ScalarOut, Evaluator<ScalarIn, CG<BaseOut>, CppAD::AD<CG<BaseOut> > > > Super;
protected:
    using Super::evalsAtomic_;
    using Super::atomicFunctions_;
    using Super::handler_;
    using Super::evalArrayCreationOperation;
protected:
    /**
     * Whenever set to true it will add a CppAD::PrintFor(0, "", var, name)
     * to every variable with a name so that names can be recovered using
     * a OperationNodeNameStreambuf.
     */
    bool printFor_;
    /**
     * Whenever set to true it will copy the name in original operation
     * nodes into new operation nodes created in AD<CG>.
     */
    bool adcgName_;
public:

    inline Evaluator(CodeHandler<ScalarIn>& handler) :
        Super(handler),
        printFor_(false),
        adcgName_(true) {
    }

    inline virtual ~Evaluator() {
    }

    /**
     * Whenever set to true it will add a CppAD::PrintFor(0, "", var, name)
     * to every variable with a name so that names can be recovered using
     * a OperationNodeNameStreambuf.
     */
    inline void setPrintFor(bool printFor) {
        printFor_ = printFor;
    }

    /**
     * true if a CppAD::PrintFor(0, "", var, name) will be added
     * to every variable with a name so that names can be recovered using
     * a OperationNodeNameStreambuf.
     * The default value is false.
     */
    inline bool isPrintFor() const {
        return printFor_;
    }

    /**
     * Whenever set to true it will copy the name in original operation
     * nodes into new operation nodes created in AD<CG>.
     */
    inline void setCopyAdCgName(bool adcgName) {
        adcgName_ = adcgName;
    }

    /**
     * Whenever set to true it will copy the name in original operation
     * nodes into new operation nodes created in AD<CG>.
     * The default value is true.
     */
    inline bool isCopyAdCgName() const {
        return adcgName_;
    }

protected:

    /**
     * @note overrides the default processActiveOut() even though this method
     *        is not virtual (hides a method in EvaluatorOperations)
     */
    void processActiveOut(const OperationNode<ScalarIn>& node,
                          ActiveOut& a) {
        if (node.getName() != nullptr) {
            if(adcgName_ && CppAD::Variable(a)) {
                ScalarOut a2(CppAD::Value(CppAD::Var2Par(a)));
                if (a2.getOperationNode() != nullptr) {
                    a2.getOperationNode()->setName(*node.getName());
                }
            }

            if(printFor_) {
                CppAD::PrintFor(ActiveOut(0), "", a, node.getName()->c_str());
            }
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif