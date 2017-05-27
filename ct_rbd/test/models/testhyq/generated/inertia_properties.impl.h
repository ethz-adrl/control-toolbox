template <typename TRAIT>
iit::HyQ::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_trunk = iit::rbd::Vector3d(-0.0223,-0.0,0.0387).cast<SCALAR>();
    tensor_trunk.fill(
		SCALAR(47.376),
        com_trunk,
        rbd::Utils::buildInertiaTensor(
                SCALAR(1.209488),
                SCALAR(5.5837),
                SCALAR(6.056973),
                SCALAR(0.00571),
                SCALAR(-0.190812),
                SCALAR(-0.012668) ));

    com_LF_hipassembly = iit::rbd::Vector3d(0.043,0.0,0.169).cast<SCALAR>();
    tensor_LF_hipassembly.fill(
		SCALAR(2.93),
        com_LF_hipassembly,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.134705),
                SCALAR(0.144171),
                SCALAR(0.011033),
                SCALAR(3.6E-5),
                SCALAR(0.022734),
                SCALAR(5.1E-5) ));

    com_LF_upperleg = iit::rbd::Vector3d(0.151,-0.026,-0.0).cast<SCALAR>();
    tensor_LF_upperleg.fill(
		SCALAR(2.638),
        com_LF_upperleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.005495),
                SCALAR(0.087136),
                SCALAR(0.089871),
                SCALAR(-0.007418),
                SCALAR(-1.02E-4),
                SCALAR(-2.1E-5) ));

    com_LF_lowerleg = iit::rbd::Vector3d(0.125,0.0,-0.0).cast<SCALAR>();
    tensor_LF_lowerleg.fill(
		SCALAR(0.881),
        com_LF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(4.68E-4),
                SCALAR(0.026409),
                SCALAR(0.026181),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0) ));

    com_RF_hipassembly = iit::rbd::Vector3d(0.043,-0.0,-0.169).cast<SCALAR>();
    tensor_RF_hipassembly.fill(
		SCALAR(2.93),
        com_RF_hipassembly,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.134705),
                SCALAR(0.144171),
                SCALAR(0.011033),
                SCALAR(-3.6E-5),
                SCALAR(-0.022734),
                SCALAR(5.1E-5) ));

    com_RF_upperleg = iit::rbd::Vector3d(0.151,-0.026,-0.0).cast<SCALAR>();
    tensor_RF_upperleg.fill(
		SCALAR(2.638),
        com_RF_upperleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.005495),
                SCALAR(0.087136),
                SCALAR(0.089871),
                SCALAR(-0.007418),
                SCALAR(-1.02E-4),
                SCALAR(-2.1E-5) ));

    com_RF_lowerleg = iit::rbd::Vector3d(0.125,0.001,-0.0).cast<SCALAR>();
    tensor_RF_lowerleg.fill(
		SCALAR(0.881),
        com_RF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(4.68E-4),
                SCALAR(0.026409),
                SCALAR(0.026181),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0) ));

    com_LH_hipassembly = iit::rbd::Vector3d(0.043,-0.0,-0.169).cast<SCALAR>();
    tensor_LH_hipassembly.fill(
		SCALAR(2.93),
        com_LH_hipassembly,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.134705),
                SCALAR(0.144171),
                SCALAR(0.011033),
                SCALAR(-3.6E-5),
                SCALAR(-0.022734),
                SCALAR(5.1E-5) ));

    com_LH_upperleg = iit::rbd::Vector3d(0.151,0.026,0.0).cast<SCALAR>();
    tensor_LH_upperleg.fill(
		SCALAR(2.638),
        com_LH_upperleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.005495),
                SCALAR(0.087136),
                SCALAR(0.089871),
                SCALAR(0.007418),
                SCALAR(1.02E-4),
                SCALAR(-2.1E-5) ));

    com_LH_lowerleg = iit::rbd::Vector3d(0.125,-0.001,0.0).cast<SCALAR>();
    tensor_LH_lowerleg.fill(
		SCALAR(0.881),
        com_LH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(4.68E-4),
                SCALAR(0.026409),
                SCALAR(0.026181),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0) ));

    com_RH_hipassembly = iit::rbd::Vector3d(0.043,0.0,0.169).cast<SCALAR>();
    tensor_RH_hipassembly.fill(
		SCALAR(2.93),
        com_RH_hipassembly,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.134705),
                SCALAR(0.144171),
                SCALAR(0.011033),
                SCALAR(3.6E-5),
                SCALAR(0.022734),
                SCALAR(5.1E-5) ));

    com_RH_upperleg = iit::rbd::Vector3d(0.151,0.026,0.0).cast<SCALAR>();
    tensor_RH_upperleg.fill(
		SCALAR(2.638),
        com_RH_upperleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.005495),
                SCALAR(0.087136),
                SCALAR(0.089871),
                SCALAR(0.007418),
                SCALAR(1.02E-4),
                SCALAR(-2.1E-5) ));

    com_RH_lowerleg = iit::rbd::Vector3d(0.125,-0.001,0.0).cast<SCALAR>();
    tensor_RH_lowerleg.fill(
    	SCALAR(0.881),
        com_RH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                SCALAR(4.68E-4),
                SCALAR(0.026409),
                SCALAR(0.026181),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0) ));

}

