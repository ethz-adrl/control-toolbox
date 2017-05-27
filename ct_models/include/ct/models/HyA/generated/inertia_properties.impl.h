template <typename TRAIT>
iit::ct_HyA::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_Shoulder_AA = iit::rbd::Vector3d(-1.96E-4,-0.003091,0.17763901).cast<SCALAR>();
    tensor_Shoulder_AA.fill(
        SCALAR(2.688),
        com_Shoulder_AA,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.08980614),
                SCALAR(0.08953714),
                SCALAR(0.007629),
                SCALAR(6.13E-4),
                SCALAR(4.91221E-4),
                SCALAR(-5.849322E-4)) );

    com_Shoulder_FE = iit::rbd::Vector3d(0.113166206,-0.023382,0.003832).cast<SCALAR>();
    tensor_Shoulder_FE.fill(
        SCALAR(2.5924191),
        com_Shoulder_FE,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.039002996),
                SCALAR(0.08585553),
                SCALAR(0.051393528),
                SCALAR(-0.0055279857),
                SCALAR(0.022851627),
                SCALAR(0.002752)) );

    com_Humerus_R = iit::rbd::Vector3d(-0.0023899989,0.00618,0.278456).cast<SCALAR>();
    tensor_Humerus_R.fill(
        SCALAR(2.327),
        com_Humerus_R,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.20236395),
                SCALAR(0.20986895),
                SCALAR(0.010329999),
                SCALAR(-5.399998E-5),
                SCALAR(0.011715295),
                SCALAR(0.0037997814)) );

    com_Elbow_FE = iit::rbd::Vector3d(0.09200001,-0.005,8.7422736E-10).cast<SCALAR>();
    tensor_Elbow_FE.fill(
        SCALAR(1.7423722),
        com_Elbow_FE,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.081661),
                SCALAR(0.01828449),
                SCALAR(0.09570849),
                SCALAR(-0.0036857284),
                SCALAR(2.1200017E-4),
                SCALAR(0.0022920002)) );

    com_Wrist_R = iit::rbd::Vector3d(0.0,0.0,0.0275).cast<SCALAR>();
    tensor_Wrist_R.fill(
        SCALAR(2.1032),
        com_Wrist_R,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.019432),
                SCALAR(0.029115),
                SCALAR(0.011046),
                SCALAR(0.001387),
                SCALAR(0.012652),
                SCALAR(0.001243)) );

    com_Wrist_FE = iit::rbd::Vector3d(0.025,0.0,0.0).cast<SCALAR>();
    tensor_Wrist_FE.fill(
        SCALAR(1.547475),
        com_Wrist_FE,
        rbd::Utils::buildInertiaTensor(
                SCALAR(0.010737),
                SCALAR(0.003227),
                SCALAR(0.009277),
                SCALAR(-1.98E-4),
                SCALAR(-1.5E-5),
                SCALAR(8.29E-4)) );

}

