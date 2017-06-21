/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef _IIT_RBD__STATE_DEPENDENT_BASE_H_
#define _IIT_RBD__STATE_DEPENDENT_BASE_H_

#include <iostream>
#include "rbd.h"

namespace iit {
namespace rbd {

/**
 * A sort of minimal interface to model the dependency on some kind of state
 * variable.
 * The type of such a state variable is the first template parameter.
 *
 * The second parameter must be a class inheriting from this one, i.e., this
 * class is supposed to be used according to the "curiously recurring template
 * pattern".
 * The sub-class shall implement the update() function with the actual,
 * class-specific logic to update the instance, because the implementation
 * of the operator(State const&) relies on update(State const&).
 */
template<class State, class Actual>
class StateDependentBase
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * Updates this object according to the given state.
         * The actual logic of the update must be implemented in the update()
         * function.
         */
        const Actual& operator()(State const& state) {
            return static_cast<Actual*>(this) -> update(state);
        }
        /**
         * Updates this object according to the given state variable.
         * Subclasses shall implement this method since the implementation of
         * operator()(State const&) relies on it.
         */
        const Actual& update(State const& state);
};

}
}


#endif
