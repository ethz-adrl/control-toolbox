/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef IIT_RBD_TRANSFORMSBASE_H_
#define IIT_RBD_TRANSFORMSBASE_H_

#include "StateDependentMatrix.h"

namespace iit {
namespace rbd {


/**
 * A 3x3 specialization of StateDependentMatrix, to be used as a base class for
 * rotation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class RotationTransformBase : public StateDependentMatrix<State, 3, 3, ActualMatrix>
{
public:
	RotationTransformBase() : StateDependentMatrix<State, 3, 3, ActualMatrix>() {}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 4x4 specialization of StateDependentMatrix, to be used as a base class for
 * homogeneous transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class HomogeneousTransformBase : public StateDependentMatrix<State, 4, 4, ActualMatrix>
{
public:
	HomogeneousTransformBase() : StateDependentMatrix<State, 4, 4, ActualMatrix>() {}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6x6 specialization of StateDependentMatrix, to be used as a base class for
 * spatial transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class SpatialTransformBase : public StateDependentMatrix<State, 6, 6, ActualMatrix>
{
public:
	SpatialTransformBase() : StateDependentMatrix<State, 6, 6, ActualMatrix>() {}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6xCols specialization of StateDependentMatrix, to be used as a base class
 * for geometric Jacobians that depend on a state variable.
 */
template<class State, int Cols, class ActualMatrix>
class JacobianBase : public StateDependentMatrix<State, 6, Cols, ActualMatrix>
{
public:
	JacobianBase() : StateDependentMatrix<State, 6, Cols, ActualMatrix>() {}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




}
}



#endif /* IIT_RBD_TRANSFORMSBASE_H_ */
