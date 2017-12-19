template <typename TRAIT>
iit::ct_DoubleInvertedPendulum::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_Link1 = iit::rbd::Vector3d(0.5,0.0,0.0).cast<Scalar>();
    tensor_Link1.fill(
        Scalar(1.0),
        com_Link1,
        rbd::Utils::buildInertiaTensor(
                Scalar(5.0E-4),
                Scalar(0.251),
                Scalar(0.2505),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(-2.1855406E-11)) );

    com_Link2 = iit::rbd::Vector3d(0.5,0.0,0.0).cast<Scalar>();
    tensor_Link2.fill(
        Scalar(1.0),
        com_Link2,
        rbd::Utils::buildInertiaTensor(
                Scalar(5.0E-4),
                Scalar(0.2505),
                Scalar(0.251),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

