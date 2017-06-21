/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef _IIT_RBD__STATE_DEPENDENT_MATRIX_H_
#define _IIT_RBD__STATE_DEPENDENT_MATRIX_H_

#include <iostream>

#include "rbd.h"
#include "StateDependentBase.h"

namespace iit {
namespace rbd {


/**
 * A matrix that exposes a dependency on some kind of state variable.
 * The type of such a variable is the first template parameter.
 * The second and third parameters are the number of rows and columns.
 *
 * The last parameter must be a class inheriting from this class, i.e., we
 * are using here the curiously recurring template pattern.
 * Such a sub-class shall implement the update() function. See the documentation
 * of StateDependentBase.
 *
 * This class was created explicitly to support code generation
 * e.g. of coordinate transforms.
 */
template<class State, int Rows, int Cols, class ActualMatrix>
class StateDependentMatrix  :
        public StateDependentBase<State, ActualMatrix>,
        public PlainMatrix<typename State::Scalar, Rows, Cols>
{
    private:
        typedef PlainMatrix<typename State::Scalar, Rows, Cols> Base;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /** The type of the coefficients of this matrix */
        typedef typename Base::Scalar Scalar;
        /** The type of row/column indices of this matrix */
        typedef typename Base::Index  Index;
        /** The regular matrix type this class inherits from */
        typedef Base MatrixType;
    public:
        StateDependentMatrix() {};
        ~StateDependentMatrix() {}

        template<typename OtherDerived>
         StateDependentMatrix& operator= (const MatrixBase<OtherDerived>& other) {
             this->Base::operator=(other);
             return *this;
         }

        using StateDependentBase<State, ActualMatrix>::operator();
        using MatrixType::operator();
        using MatrixType::setZero;
};


}
}


#endif /* _IIT_RBD__STATE_DEPENDENT_MATRIX_H_ */
