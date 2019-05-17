
#pragma once

#include "TermLinear.hpp"
#include "TermQuadratic.hpp"
#include "TermQuadMult.hpp"
#include "TermMixed.hpp"
#include "TermSmoothAbs.hpp"

#define CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TERM, TERMNAME)                      \
    if (termKind == TERMNAME)                                                      \
    {                                                                              \
        term = std::shared_ptr<TERM<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>>( \
            new TERM<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>());              \
        term->setName(TERMNAME);                                                   \
    }

#define CT_LOADABLE_TERMS(SCALAR_EVAL, SCALAR)                                      \
    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermLinear, "linear")                     \
    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermQuadratic, "quadratic")               \
    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermMixed, "mixed")                       \
    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermQuadMult, "quadratic-multiplicative") \
    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermSmoothAbs, "smooth-abs")
