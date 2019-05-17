/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */
#ifndef IIT_RBD_TYPES_H_
#define IIT_RBD_TYPES_H_

#include "rbd.h"
#include "InertiaMatrix.h"


namespace iit {
namespace rbd {

#define TPL template<typename S>

TPL using Velocity = typename Core<S>::VelocityVector;
TPL using Force    = typename Core<S>::ForceVector;
TPL using Mat33    = typename Core<S>::Matrix33;
TPL using Mat66    = typename Core<S>::Matrix66;
TPL using Vec3     = typename Core<S>::Vector3;
TPL using Vec6     = typename Core<S>::Vector6;

TPL using InertiaMat = tpl::InertiaMatrixDense<S>;

#undef TPL

}
}


#endif
