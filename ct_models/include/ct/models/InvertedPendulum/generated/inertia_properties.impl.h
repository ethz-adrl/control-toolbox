template <typename TRAIT>
iit::ct_InvertedPendulum::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_Link1 = iit::rbd::Vector3d(0.137,0.0,0.0).cast<Scalar>();
    tensor_Link1.fill(
        Scalar(1.811),
        com_Link1,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.7E-4),
                Scalar(0.034460656),
                Scalar(0.043045655),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

