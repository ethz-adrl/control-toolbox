/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef IIT_RBD_EIGEN_TRAITS_
#define IIT_RBD_EIGEN_TRAITS_


#include "StateDependentMatrix.h"
#include "TransformsBase.h"

/**
 * \file
 * This header file contains some instantiations of the \c traits template (in
 * the \c Eigen::internal namespace) for matrix types defined in \c iit::rbd
 */

namespace Eigen {
namespace internal {

/**
 * The Eigen traits for the iit::rbd::HomogeneousTransformBase type
 */
template<typename State, typename M>
struct traits< iit::rbd::HomogeneousTransformBase<State, M> >
{
        typedef typename iit::rbd::HomogeneousTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};

/**
 * The Eigen traits for the iit::rbd::RotationTransformBase type
 */
template<typename State, typename M>
struct traits< iit::rbd::RotationTransformBase<State, M> >
{
        typedef typename iit::rbd::RotationTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};


}
}

#endif
