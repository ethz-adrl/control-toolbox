#ifndef CPPAD_CG_ATOMIC_FUN_INCLUDED
#define CPPAD_CG_ATOMIC_FUN_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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
 * An atomic function for source code generation
 * 
 * @author Joao Leal
 */
template <class Base>
class CGAtomicFun : public CGAbstractAtomicFun<Base> {
protected:
    atomic_base<Base>& atomicFun_;
public:

    /**
     * Creates a new atomic function wrapper that is responsible for 
     * defining the dependencies to calls of a user atomic function.
     * 
     * @param atomicFun The atomic function to the called by the compiled
     *                  source.
     * @param standAlone Whether or not forward and reverse function calls
     *                   do not require the Taylor coefficients for the 
     *                   dependent variables (ty) and the previous
     *                   evaluation of other forward/reverse modes.
     */
    CGAtomicFun(atomic_base<Base>& atomicFun, bool standAlone = false) :
        CGAbstractAtomicFun<Base>(atomicFun.afun_name(), standAlone),
        atomicFun_(atomicFun) {

    }

    template <class ADVector>
    void operator()(const ADVector& ax, ADVector& ay, size_t id = 0) {
        this->CGAbstractAtomicFun<Base>::operator()(ax, ay, id);
    }

    virtual bool for_sparse_jac(size_t q,
                                const CppAD::vector< std::set<size_t> >& r,
                                CppAD::vector< std::set<size_t> >& s) override {
        return atomicFun_.for_sparse_jac(q, r, s);
    }

    virtual bool for_sparse_jac(size_t q,
                                const CppAD::vector<bool>& r,
                                CppAD::vector<bool>& s) override {
        return atomicFun_.for_sparse_jac(q, r, s);
    }

    virtual bool rev_sparse_jac(size_t q,
                                const CppAD::vector< std::set<size_t> >& rt,
                                CppAD::vector< std::set<size_t> >& st) override {
        return atomicFun_.rev_sparse_jac(q, rt, st);
    }

    virtual bool rev_sparse_jac(size_t q,
                                const CppAD::vector<bool>& rt,
                                CppAD::vector<bool>& st) override {
        return atomicFun_.rev_sparse_jac(q, rt, st);
    }

    virtual bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                                const CppAD::vector<bool>& s,
                                CppAD::vector<bool>& t,
                                size_t q,
                                const CppAD::vector< std::set<size_t> >& r,
                                const CppAD::vector< std::set<size_t> >& u,
                                CppAD::vector< std::set<size_t> >& v) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v);
    }

    virtual bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                                const CppAD::vector<bool>& s,
                                CppAD::vector<bool>& t,
                                size_t q,
                                const CppAD::vector<bool>& r,
                                const CppAD::vector<bool>& u,
                                CppAD::vector<bool>& v) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v);
    }

    virtual ~CGAtomicFun() {
    }

protected:

    virtual void zeroOrderDependency(const CppAD::vector<bool>& vx,
                                     CppAD::vector<bool>& vy) override {
        using CppAD::vector;

        size_t m = vy.size();
        size_t n = vx.size();

        vector<std::set<size_t> > rt(m);
        for (size_t j = 0; j < m; j++) {
            rt[j].insert(j);
        }
        vector<std::set<size_t> > st(n);

        rev_sparse_jac(m, rt, st);

        for (size_t j = 0; j < n; j++) {
            for (size_t i : st[j]) {
                if (vx[j]) {
                    vy[i] = true;
                }
            }
        }
    }

    virtual bool atomicForward(size_t q,
                               size_t p,
                               const CppAD::vector<Base>& tx,
                               CppAD::vector<Base>& ty) override {
        CppAD::vector<bool> vx, vy;
        return atomicFun_.forward(q, p, vx, vy, tx, ty);
    }

    virtual bool atomicReverse(size_t p,
                               const CppAD::vector<Base>& tx,
                               const CppAD::vector<Base>& ty,
                               CppAD::vector<Base>& px,
                               const CppAD::vector<Base>& py) override {
        return atomicFun_.reverse(p, tx, ty, px, py);
    }
};

} // END cg namespace
} // END CppAD namespace

#endif