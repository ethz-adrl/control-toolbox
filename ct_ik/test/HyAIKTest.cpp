#include <ct_ik/HyAInverseKinematics.h>
#include <ct/rbd/rbd.h>
#include <ct/rbd/state/JointState.h>
#include <ct/models/HyA/HyA.h>

#include <gtest/gtest.h>

TEST(HyaIKTest, IKTest)
{
    ct::rbd::HyAInverseKinematics<6> hya_ik;
    ct::rbd::HyA::Kinematics kin;
    typename ct::rbd::tpl::JointState<6, double>::Position pos;
    pos << 0, 1, 0, -1, 1, -1;

    auto ret = kin.getEEPoseInBase(0, pos);

    std::cout << ret.position() << std::endl;

    std::vector<double> joints{0, 1, 0, -1, 1, -1}, eetrans(3), eerot(9);
    hya_ik::ComputeFk(joints.data(), eetrans.data(), eerot.data());

    for (auto i : eetrans)
    {
        std::cout << i << std::endl;
    }

    for (auto i : hya_ik.computeInverseKinematics(ret))
        std::cerr << i << std::endl << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
