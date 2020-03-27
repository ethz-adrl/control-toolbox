
#include <manif/manif.h>

using namespace manif;


// but why on earth so complicated?

template <typename Derived, typename T>
class Base : public T
{
public:
    Base() = default;
    virtual ~Base() = default;

    void foo() {}
    template <typename O>
    Base(const O& other) : T(other)
    {
    }
};


template <typename T, typename TAN>
class TestManifold : public Base<TestManifold<T, TAN>, T>
{
public:
    using Base_t = Base<TestManifold<T, TAN>, T>;
    using Tangent = TAN;

    TestManifold() = default;
    virtual ~TestManifold() = default;

    template <typename OTHER>
    TestManifold(const OTHER& other) : Base_t(other)
    {
    }
};


template <typename LieGroup>
class SomeAlgorithm
{
public:
    using Tangent = typename LieGroup::Tangent;

    LieGroup integrate(const LieGroup x, const Tangent v) { return x + v; }
};


int main()
{
    using State = TestManifold<SE3d, SE3Tangentd>;

    SomeAlgorithm<State> alg;

    State m;
    m.setIdentity();

    State::Tangent t;
    t.setZero();
    t.w()(0) = 1.0;

    auto res = alg.integrate(m, t);

    std::cout << res << std::endl;

    return 0;
}
