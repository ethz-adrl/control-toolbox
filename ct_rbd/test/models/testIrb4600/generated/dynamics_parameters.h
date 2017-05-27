#ifndef _TESTIRB4600_RUNTIME_INERTIA_PARAMETERS_
#define _TESTIRB4600_RUNTIME_INERTIA_PARAMETERS_

namespace iit {
namespace testirb4600 {
namespace dyn {
/**
 * \defgroup dynparams Dynamics-parameters
 * Facilities related to the parameters of the inertia properties of the
 * robot testirb4600.
 *
 * Inertia parameters are non-constants used in the robot model, where the
 * inertia properties (mass, center of mass, intertia tensor) of the links
 * are specified. Since the value of such parameters must be resolved
 * at runtime, we sometimes refer to them as "runtime parameters", "runtime
 * dynamics parameters", "runtime inertia parameters", etc.
 *
 * Do not confuse them with the "inertia properties" of links, which
 * unfortunately, in the literature, are commonly referred to as
 * "inertia parameters"... Here, the parameters are the non-constant
 * fields of the inertia properties.
 */

    /**
     * A container for the set of non-constant inertia parameters of the robot testirb4600
     * \ingroup dynparams
     */
    struct RuntimeInertiaParams {
    };

    /**
     * The interface for classes that can compute the actual value of the
     * non-constant inertia parameters of the robot testirb4600.
     * \ingroup dynparams
     */
    class RuntimeParamsGetter {
        public:
    };

}
}
}
#endif
