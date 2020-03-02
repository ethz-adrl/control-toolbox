#pragma once

#include "DiscreteArray.h"
#include <manif/manif.h>

namespace ct {
namespace core {
namespace tpl {

template <typename SCALAR>
using SE2Array = DiscreteArray<manif::SE2<SCALAR>>;

template <typename SCALAR>
using SE3Array = DiscreteArray<manif::SE3<SCALAR>>;

template <typename SCALAR>
using SE2TangentArray = DiscreteArray<manif::SE2Tangent<SCALAR>>;

template <typename SCALAR>
using SE3TangentArray = DiscreteArray<manif::SE3Tangent<SCALAR>>;

}  // namespace tpl

using SE2Array = tpl::SE2Array<double>;
using SE3Array = tpl::SE3Array<double>;

using SE2TangentArray = tpl::SE2TangentArray<double>;
using SE3TangentArray = tpl::SE3TangentArray<double>;

}  // namespace core
}  // namespace ct