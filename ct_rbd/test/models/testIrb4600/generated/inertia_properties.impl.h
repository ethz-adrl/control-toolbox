template <typename TRAIT>
iit::testirb4600::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_link1 = iit::rbd::Vector3d(0.09,0.0,0.25).cast<SCALAR>();
    tensor_link1.fill(
        SCALAR(120.0),
        com_link1,
        rbd::Utils::buildInertiaTensor(
                SCALAR(10.02),
                SCALAR(10.992001),
                SCALAR(2.7719998),
                SCALAR(0.0),
                SCALAR(2.7),
                SCALAR(0.0)) );

    com_link2 = iit::rbd::Vector3d(0.47,0.04,-0.17).cast<SCALAR>();
    tensor_link2.fill(
        SCALAR(120.0),
        com_link2,
        rbd::Utils::buildInertiaTensor(
                SCALAR(4.36),
                SCALAR(41.225998),
                SCALAR(38.449997),
                SCALAR(2.256),
                SCALAR(-9.588),
                SCALAR(-0.816)) );

    com_link3 = iit::rbd::Vector3d(0.035,-0.133,0.044).cast<SCALAR>();
    tensor_link3.fill(
        SCALAR(120.0),
        com_link3,
        rbd::Utils::buildInertiaTensor(
                SCALAR(6.0550003),
                SCALAR(5.4093204),
                SCALAR(7.29968),
                SCALAR(-0.5586),
                SCALAR(0.1848),
                SCALAR(-0.70224)) );

    com_link4 = iit::rbd::Vector3d(0.0,0.0,0.5).cast<SCALAR>();
    tensor_link4.fill(
        SCALAR(40.0),
        com_link4,
        rbd::Utils::buildInertiaTensor(
                SCALAR(12.0),
                SCALAR(12.0),
                SCALAR(0.482),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

    com_link5 = iit::rbd::Vector3d(0.07,0.0,0.0).cast<SCALAR>();
    tensor_link5.fill(
        SCALAR(10.0),
        com_link5,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.04),
                SCALAR(0.089),
                SCALAR(0.094000004),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

    com_link6 = iit::rbd::Vector3d(0.081,-0.091,0.232).cast<SCALAR>();
    tensor_link6.fill(
        SCALAR(5.0),
        com_link6,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.014),
                SCALAR(0.014),
                SCALAR(0.028),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

}

