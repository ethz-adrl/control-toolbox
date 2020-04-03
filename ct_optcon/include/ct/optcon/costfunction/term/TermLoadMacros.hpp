
#pragma once

// TODO: adapt this

#define CT_LOADABLE_TERM(MANIFOLD, CONTROL_DIM, TERM, TERMNAME)                                 \
    if (termKind == TERMNAME)                                                                   \
    {                                                                                           \
        term = std::shared_ptr<TERM<MANIFOLD, CONTROL_DIM>>(new TERM<MANIFOLD, CONTROL_DIM>()); \
        term->setName(TERMNAME);                                                                \
    }

#define CT_LOADABLE_TERMS(MANIFOLD, CONTROL_DIM) CT_LOADABLE_TERM(MANIFOLD, CONTROL_DIM, TermQuadratic, "quadratic")

//    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermLinear, "linear")                     \
//    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermMixed, "mixed")                       \
//    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermQuadMult, "quadratic-multiplicative") \
//    CT_LOADABLE_TERM(SCALAR_EVAL, SCALAR, TermSmoothAbs, "smooth-abs")
