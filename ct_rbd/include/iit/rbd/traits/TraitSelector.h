#ifndef INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_
#define INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_

#include "FloatTrait.h"
#include "DoubleTrait.h"

namespace iit {
namespace rbd {
namespace tpl {

template <typename SCALAR>
struct TraitSelector
{

};

template <>
struct TraitSelector<double>
{
 	typedef DoubleTrait Trait;
};

template <>
struct TraitSelector<float>
{
 	typedef FloatTrait Trait;
};

} //namespace tpl
} // namespace rbd
} // namespace iit


#endif //INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_

