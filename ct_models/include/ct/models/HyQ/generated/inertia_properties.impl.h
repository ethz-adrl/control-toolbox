template <typename TRAIT>
iit::HyQ::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_trunk = iit::rbd::Vector3d(-0.0223,-1.0E-4,0.0387).cast<Scalar>();
    tensor_trunk.fill(
        Scalar(53.433),
        com_trunk,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.209488),
                Scalar(5.5837),
                Scalar(6.056973),
                Scalar(0.00571),
                Scalar(-0.190812),
                Scalar(-0.012668)) );

    com_LF_hipassembly = iit::rbd::Vector3d(0.04263,0.0,0.16931).cast<Scalar>();
    tensor_LF_hipassembly.fill(
        Scalar(3.44),
        com_LF_hipassembly,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.134705),
                Scalar(0.144171),
                Scalar(0.011033),
                Scalar(3.6E-5),
                Scalar(0.022734),
                Scalar(5.1E-5)) );

    com_LF_upperleg = iit::rbd::Vector3d(0.15074,-0.02625,-0.0).cast<Scalar>();
    tensor_LF_upperleg.fill(
        Scalar(3.146),
        com_LF_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.005495),
                Scalar(0.087136),
                Scalar(0.089871),
                Scalar(-0.007418),
                Scalar(-1.02E-4),
                Scalar(-2.1E-5)) );

    com_LF_lowerleg = iit::rbd::Vector3d(0.1254,5.0E-4,-1.0E-4).cast<Scalar>();
    tensor_LF_lowerleg.fill(
        Scalar(0.881),
        com_LF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.68E-4),
                Scalar(0.026409),
                Scalar(0.026181),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_RF_hipassembly = iit::rbd::Vector3d(0.04263,-0.0,-0.16931).cast<Scalar>();
    tensor_RF_hipassembly.fill(
        Scalar(3.44),
        com_RF_hipassembly,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.134705),
                Scalar(0.144171),
                Scalar(0.011033),
                Scalar(-3.6E-5),
                Scalar(-0.022734),
                Scalar(5.1E-5)) );

    com_RF_upperleg = iit::rbd::Vector3d(0.15074,-0.02625,-0.0).cast<Scalar>();
    tensor_RF_upperleg.fill(
        Scalar(3.146),
        com_RF_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.005495),
                Scalar(0.087136),
                Scalar(0.089871),
                Scalar(-0.007418),
                Scalar(-1.02E-4),
                Scalar(-2.1E-5)) );

    com_RF_lowerleg = iit::rbd::Vector3d(0.1254,5.0E-4,-1.0E-4).cast<Scalar>();
    tensor_RF_lowerleg.fill(
        Scalar(0.881),
        com_RF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.68E-4),
                Scalar(0.026409),
                Scalar(0.026181),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_LH_hipassembly = iit::rbd::Vector3d(0.04263,-0.0,-0.16931).cast<Scalar>();
    tensor_LH_hipassembly.fill(
        Scalar(3.44),
        com_LH_hipassembly,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.134705),
                Scalar(0.144171),
                Scalar(0.011033),
                Scalar(-3.6E-5),
                Scalar(-0.022734),
                Scalar(5.1E-5)) );

    com_LH_upperleg = iit::rbd::Vector3d(0.15074,0.02625,0.0).cast<Scalar>();
    tensor_LH_upperleg.fill(
        Scalar(3.146),
        com_LH_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.005495),
                Scalar(0.087136),
                Scalar(0.089871),
                Scalar(0.007418),
                Scalar(1.02E-4),
                Scalar(-2.1E-5)) );

    com_LH_lowerleg = iit::rbd::Vector3d(0.125,-0.001,0.0).cast<Scalar>();
    tensor_LH_lowerleg.fill(
        Scalar(0.881),
        com_LH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.68E-4),
                Scalar(0.026409),
                Scalar(0.026181),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_RH_hipassembly = iit::rbd::Vector3d(0.04263,0.0,0.16931).cast<Scalar>();
    tensor_RH_hipassembly.fill(
        Scalar(3.44),
        com_RH_hipassembly,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.134705),
                Scalar(0.144171),
                Scalar(0.011033),
                Scalar(3.6E-5),
                Scalar(0.022734),
                Scalar(5.1E-5)) );

    com_RH_upperleg = iit::rbd::Vector3d(0.15074,0.02625,0.0).cast<Scalar>();
    tensor_RH_upperleg.fill(
        Scalar(3.146),
        com_RH_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.005495),
                Scalar(0.087136),
                Scalar(0.089871),
                Scalar(0.007418),
                Scalar(1.02E-4),
                Scalar(-2.1E-5)) );

    com_RH_lowerleg = iit::rbd::Vector3d(0.1254,-5.0E-4,1.0E-4).cast<Scalar>();
    tensor_RH_lowerleg.fill(
        Scalar(0.881),
        com_RH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.68E-4),
                Scalar(0.026409),
                Scalar(0.026181),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

