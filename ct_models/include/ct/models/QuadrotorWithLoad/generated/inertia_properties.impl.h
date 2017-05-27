template <typename TRAIT>
iit::ct_quadrotor::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_body = iit::rbd::Vector3d(0.0,0.0,0.0).cast<SCALAR>();
    tensor_body.fill(
        SCALAR(0.5),
        com_body,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.03),
                SCALAR(0.05),
                SCALAR(0.03),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

    com_link1 = iit::rbd::Vector3d(0.0,0.0,-0.05).cast<SCALAR>();
    tensor_link1.fill(
        SCALAR(0.025),
        com_link1,
        rbd::Utils::buildInertiaTensor(
                SCALAR(2.075E-5),
                SCALAR(1.0E-6),
                SCALAR(2.075E-5),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

    com_link2 = iit::rbd::Vector3d(0.35,0.0,0.0).cast<SCALAR>();
    tensor_link2.fill(
        SCALAR(0.1),
        com_link2,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.004084),
                SCALAR(0.004084),
                SCALAR(2.5E-6),
                SCALAR(0.0),
                SCALAR(0.0),
                SCALAR(0.0)) );

}

