namespace ct {
namespace rbd {

template <class ROB, typename SCALAR>
PinocchioRBD<ROB, SCALAR>::PinocchioRBD() : model_(), data_(model_)
{
}

template <class ROB, typename SCALAR>
PinocchioRBD<ROB, SCALAR>::PinocchioRBD(const PinocchioRBD& other)
    : BASE(other), model_(other.model_), data_(model_), w_pose_base_(other.w_pose_base_)
{
}

template <class ROB, typename SCALAR>
PinocchioRBD<ROB, SCALAR>* PinocchioRBD<ROB, SCALAR>::clone() const
{
    return new PinocchioRBD(*this);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadModelFromString(const char* xml_string, const bool verb)
{
    std::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml_string);

    return loadFromURDF(urdf_model.get(), verb);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadFromURDF(const urdf::ModelInterface* urdf_mdl, const bool verb)
{
    std::shared_ptr<urdf::ModelInterface> urdf_model(new urdf::ModelInterface(*urdf_mdl));

    pinocchio::ModelTpl<double> dmodel;
    pinocchio::urdf::buildModel(urdf_model, dmodel, verb);
    model_ = dmodel.template cast<SCALAR>();

    return completeModelSetup(verb);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadModelFromFile(const std::string& fileName, const bool verb)
{
    pinocchio::ModelTpl<double> dmodel;
    pinocchio::urdf::buildModel(fileName, dmodel, verb);
    model_ = dmodel.template cast<SCALAR>();

    return completeModelSetup(verb);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadModelFromString(const char* xml_string,
    const std::vector<std::string>& move_joints,
    const bool verb)
{
    std::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml_string);

    return loadFromURDF(urdf_model.get(), move_joints, verb);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadFromURDF(const urdf::ModelInterface* urdf_mdl,
    const std::vector<std::string>& move_joints,
    const bool verb)
{
    std::shared_ptr<urdf::ModelInterface> urdf_model(new urdf::ModelInterface(*urdf_mdl));

    pinocchio::ModelTpl<double> dmodel;
    pinocchio::urdf::buildModel(urdf_model, /*move_joints,*/ dmodel, verb);  // TODO:
    model_ = dmodel.template cast<SCALAR>();

    return completeModelSetup(verb);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::loadModelFromFile(const std::string& fileName,
    const std::vector<std::string>& move_joints,
    const bool verb)
{
    pinocchio::ModelTpl<double> dmodel;
    pinocchio::urdf::buildModel(fileName, /*move_joints,*/ dmodel, verb);
    model_ = dmodel.template cast<SCALAR>();

    return completeModelSetup(verb);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeGravityCompensation(const JointPosition_t& p, JointTorque_t& tau)
{
    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::computeGeneralizedGravity(model_, data_, jointConfig);
    tau = data_.g;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeInverseDynamics(const JointPosition_t& p,
    const JointVelocity_t& v,
    const JointAcceleration_t& a,
    JointTorque_t& tau)
{
    // evaluate the inverse dynamics
    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::rnea(model_, data_, jointConfig, v, a);
    tau = data_.tau;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeInverseDynamics(const JointPosition_t& p,
    const JointVelocity_t& v,
    const JointAcceleration_t& a,
    const Wrench_t& extWrench_in_tool_frame,
    JointTorque_t& tau)
{
    auto extWrench_in_joint_frame = projectToolWrenchToLastJoint(p, extWrench_in_tool_frame);

    // evaluate the inverse dynamics with external forces
    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::rnea(model_, data_, jointConfig, v, a, extWrench_in_joint_frame);
    tau = data_.tau;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeForwardDynamics(const JointPosition_t& p,
    const JointVelocity_t& v,
    const JointTorque_t& tau,
    JointAcceleration_t& a,
    const bool update_kin)
{
    auto jointConfig = toPinocchioConfigVector(p);
    a = pinocchio::aba(model_, data_, jointConfig, v, tau);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeFramePoseInBaseCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Pose_t& result,
    const bool update_kin)
{
    // get the frame name from the frame
    std::string frameName;
    if (!ct::rbd::extractFrameName<ROB>(frame, frameName))
        throw std::runtime_error("Pinocchio: frame not found.");

    if (model_.existFrame(frameName))
    {
        size_t frame_id = model_.getFrameId(frameName);

        if (update_kin)
        {
            auto jointConfig = toPinocchioConfigVector(p);
            pinocchio::forwardKinematics(model_, data_, jointConfig);
            pinocchio::updateFramePlacement(model_, data_, frame_id);
        }

        // get pose of desired frame in world
        Pose_t w_pose_frame;
        w_pose_frame.setFromRotationMatrix(kindr::RotationMatrix<SCALAR>(data_.oMf[frame_id].rotation_impl()));
        w_pose_frame.position().toImplementation() = data_.oMf[frame_id].translation_impl();

        // express desired pose frame in base coordinates
        result = w_pose_frame.inReferenceFrame(w_pose_base_);
    }
    else
    {
        throw std::runtime_error("Pinocchio: requested frame with name " + frameName + " does not exist.");
    }
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeBasePoseInFrameCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Pose_t& result,
    const bool update_kin)
{
    // first compute the pose of the end-effector frame in base coordinates
    Pose_t b_pose_ee;
    computeFramePoseInBaseCoordinates(p, frame, b_pose_ee, update_kin);

    // then invert the transformation (base is "identity")
    result = b_pose_ee.inverted();
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeFramePoseInWorldCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Pose_t& result,
    const bool update_kin)
{
    // get the frame name from the frame
    std::string frameName;
    if (!extractFrameName<ROB>(frame, frameName))
        throw std::runtime_error("Pinocchio: frame not found.");

    if (model_.existFrame(frameName))
    {
        size_t frame_id = model_.getFrameId(frameName);

        if (update_kin)
        {
            auto jointConfig = toPinocchioConfigVector(p);
            pinocchio::forwardKinematics(model_, data_, jointConfig);
            pinocchio::updateFramePlacement(model_, data_, frame_id);
        }

        // get pose of desired frame in world
        result.setFromRotationMatrix(kindr::RotationMatrix<SCALAR>(data_.oMf[frame_id].rotation_impl()));
        result.position().toImplementation() = data_.oMf[frame_id].translation_impl();
    }
    else
    {
        throw std::runtime_error("Pinocchio: requested frame with name " + frameName + " does not exist.");
    }
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeWorldPoseInFrameCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Pose_t& result,
    const bool update_kin)
{
    // first compute the pose of the end-effector frame in base coordinates
    Pose_t w_pose_ee;
    computeFramePoseInWorldCoordinates(p, frame, w_pose_ee, update_kin);

    // then invert the transformation (base is "identity")
    Pose_t identity_pose;
    identity_pose.setIdentity();
    result = identity_pose.inReferenceFrame(w_pose_ee);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianInBaseCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Jacobian_t& jacobian,
    const bool update_kin)
{
    // first compute jacobian in frame coordinates
    computeSpatialJacobianInFrameCoordinates(p, frame, jacobian, update_kin);

    // then rotate it into base coordinates
    jacobian = this->fromFrameToBaseCoordinates(jacobian, p, frame, false);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianInWorldCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Jacobian_t& jacobian,
    const bool update_kin)
{
    // compute jacobian in frame coordinates
    computeSpatialJacobianInFrameCoordinates(p, frame, jacobian, update_kin);

    // and rotate it into the world frame
    jacobian = this->fromFrameToWorldCoordinates(jacobian, p, frame, false);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianInFrameCoordinates(const JointPosition_t& p,
    const typename ROB::Frames frame,
    Jacobian_t& jacobian,
    const bool update_kin)
{
    // get the frame name from the frame
    std::string frameName;
    if (!extractFrameName<ROB>(frame, frameName))
        throw std::runtime_error("Pinocchio: frame not found.");

    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::computeJointJacobians(model_, data_, jointConfig);
    pinocchio::framesForwardKinematics(model_, data_, jointConfig);  // mandatory after computing jacobians (!)

    typename Data_t::Matrix6x J(6, model_.nv);  // this dynamic size matrix is required for pinocchio API-reasons
    J.setZero();                                // required!

    pinocchio::getFrameJacobian(model_, data_, model_.getFrameId(frameName), pinocchio::ReferenceFrame::LOCAL, J);

    jacobian = J;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeTwistInFrameCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Twist_t& twist,
    const bool update_kin)
{
    // get the frame name from the frame
    std::string frameName;
    if (!extractFrameName<ROB>(frame, frameName))
        throw std::runtime_error("Pinocchio: frame not found.");

    if (update_kin)
    {
        size_t frame_id = model_.getFrameId(frameName);
        auto jointConfig = toPinocchioConfigVector(p);
        pinocchio::forwardKinematics(model_, data_, jointConfig, v);
        pinocchio::updateFramePlacement(model_, data_, frame_id);
    }
    // compute frame velocity in local coordinates
    Motion_t frame_v = getFrameVelocity(model_, data_, model_.getFrameId(frameName));

    twist.template head<3>() = frame_v.linear();   // assign translational part
    twist.template tail<3>() = frame_v.angular();  // assign rotational part
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeTwistInBaseCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Twist_t& twist,
    const bool update_kin)
{
    // get twist for frame coordinates
    computeTwistInFrameCoordinates(p, v, frame, twist, update_kin);

    // and rotate into base coordinates
    twist = this->fromFrameToBaseCoordinates(twist, p, frame, false);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeTwistInWorldCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Twist_t& twist,
    const bool update_kin)
{
    // get twist in frame coordinates ...
    computeTwistInFrameCoordinates(p, v, frame, twist, update_kin);

    // .. and rotate it to world coordinates
    twist = this->fromFrameToWorldCoordinates(twist, p, frame, false);
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::checkModel(const Model_t& mdl)
{
    // compare degrees of freedom (here velocity degrees of freedom) with NJOINTS from description
    if (mdl.nv != NJOINTS)
    {
        std::cerr << "Pinocchio: the loaded model_ has a different number of DoF than specified in ROB." << std::endl;
        std::cerr << "#DoF according to model: " << mdl.nv << std::endl;
        std::cerr << "#DoF (NJOINTS) according to ROB: " << NJOINTS << std::endl;
        return false;
    }
    return true;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::printModelInfo(const Model_t& mdl) const
{
    // looking at the model properties
    std::cout << "Loaded URDF model_ into Pinocchio:" << std::endl;
    std::cout << " -the model_ has " << mdl.nv << " DoF " << std::endl;
    std::cout << " -the model_ has " << mdl.njoints << " joints " << std::endl;
    std::cout << " -the joint names are: ";
    for (size_t i = 0; i < mdl.names.size(); i++)
    {
        std::cout << "  " << mdl.names[i];
    }
    std::cout << std::endl;
    std::cout << " -the model_ has " << mdl.nframes << " frames " << std::endl;
    std::cout << " -the model_ has " << mdl.nbodies << " bodies " << std::endl;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeForwardDynamicsDerivatives(const JointPosition_t& p,
    const JointVelocity_t& v,
    const JointTorque_t& tau,
    DerivativeMatrix_t& partial_dq,
    DerivativeMatrix_t& partial_dv,
    DerivativeMatrix_t& partial_dtau)
{
    auto jointConfig = toPinocchioConfigVector(p);

    // compute the derivatives
    pinocchio::computeABADerivatives(model_, data_, jointConfig, v, tau);

    partial_dq = data_.ddq_dq;
    partial_dv = data_.ddq_dv;
    partial_dtau = data_.Minv;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeMinverse(const JointPosition_t& p,
    JointSpaceInertiaMatrix_t& minv,
    const bool run_computations)
{
    if (run_computations)
    {
        auto jointConfig = toPinocchioConfigVector(p);
        pinocchio::computeMinverse(model_, data_, jointConfig);
    }

    data_.Minv.template triangularView<Eigen::StrictlyLower>() =
        data_.Minv.transpose().template triangularView<Eigen::StrictlyLower>();

    minv = data_.Minv;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeForwardDynamicsDerivatives(const JointPosition_t& p,
    const JointVelocity_t& v,
    const JointTorque_t& tau,
    const Wrench_t& extToolWrench,
    DerivativeMatrix_t& partial_dq,
    DerivativeMatrix_t& partial_dv,
    DerivativeMatrix_t& partial_dtau)
{
    auto fext = projectToolWrenchToLastJoint(p, extToolWrench);

    // compute the derivatives
    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::computeABADerivatives(model_, data_, jointConfig, v, tau, fext);

    partial_dq = data_.ddq_dq;
    partial_dv = data_.ddq_dv;
    partial_dtau = data_.Minv;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianTimeDerivativeInFrameCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Jacobian_t& jacTimeDerivative)
{
    // get the frame name from the frame
    std::string frameName;
    if (!extractFrameName<ROB>(frame, frameName))
        throw std::runtime_error("Pinocchio: frame not found.");

    typename Data_t::Matrix6x dJdt(6, model_.nv);  // this dynamic size matrix is required for pinocchio API-reasons
    dJdt.setZero();                                // required!

    size_t frame_idx = model_.getFrameId(frameName);
    auto jointConfig = toPinocchioConfigVector(p);
    pinocchio::computeJointJacobiansTimeVariation(model_, data_, jointConfig, v);
    pinocchio::getFrameJacobianTimeVariation(model_, data_, frame_idx, pinocchio::ReferenceFrame::LOCAL, dJdt);

    jacTimeDerivative = dJdt;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianTimeDerivativeInBaseCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Jacobian_t& dJdt)
{
    // first compute jacobian derivative in frame coordinates
    computeSpatialJacobianTimeDerivativeInFrameCoordinates(p, v, frame, dJdt);

    // then rotate it into base coordinates
    dJdt = this->fromFrameToBaseCoordinates(dJdt, p, frame, false);
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::computeSpatialJacobianTimeDerivativeInWorldCoordinates(const JointPosition_t& p,
    const JointVelocity_t& v,
    const typename ROB::Frames frame,
    Jacobian_t& dJdt)
{
    // compute jacobian derivative in frame coordinates
    computeSpatialJacobianTimeDerivativeInFrameCoordinates(p, v, frame, dJdt);

    // and rotate it into the world frame
    dJdt = this->fromFrameToWorldCoordinates(dJdt, p, frame, false);
}

template <class ROB, typename SCALAR>
typename PinocchioRBD<ROB, SCALAR>::PinocchioForceVector_t PinocchioRBD<ROB, SCALAR>::projectToolWrenchToLastJoint(
    const JointPosition_t& p,
    const Wrench_t& extToolWrench,
    const bool update_kin)
{
    // initialize new external force vector (zero everywhere)
    PinocchioForceVector_t fext(model_.njoints, Force_t::Zero());

    Pose_t b_p_tool;  // tool pose in base frame
    computeFramePoseInBaseCoordinates(p, ROB::TOOL, b_p_tool);


    size_t frame_id = model_.getFrameId(ROB::ControlledJoints.back());
    if (update_kin)
    {
        auto jointConfig = toPinocchioConfigVector(p);
        pinocchio::forwardKinematics(model_, data_, jointConfig);
    }
    pinocchio::updateFramePlacement(model_, data_, frame_id);

    // get pose of desired frame in world
    Pose_t w_pose_frame;
    w_pose_frame.setFromRotationMatrix(kindr::RotationMatrix<SCALAR>(data_.oMf[frame_id].rotation_impl()));
    w_pose_frame.position().toImplementation() = data_.oMf[frame_id].translation_impl();

    // express desired pose frame in base coordinates
    Pose_t b_p_lastJoint = w_pose_frame.inReferenceFrame(w_pose_base_);

    Pose_t tool_pose_in_last_jointframe = b_p_tool.inReferenceFrame(b_p_lastJoint);

    Eigen::Matrix<SCALAR, 6, 6> T = tool_pose_in_last_jointframe.get6DMotionTransform();

    Wrench_t w_trans = T * extToolWrench;

    Force_t eeWrench_aba = Force_t::Zero();

    eeWrench_aba.toVector() = w_trans;

    // we assign the external wrench to the last joint
    fext.back() = eeWrench_aba;

    return fext;
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::completeModelSetup(const bool verb)
{
    data_ = Data_t(model_);

    // assign joint limits
    assignJointLimits(model_, verb);

    // assign velocity limits
    assignVelocityLimits(model_, verb);

    // assign effort limits
    assignEffortLimits(model_, verb);

    if (verb)
        printModelInfo(model_);

    if (!checkModel(model_))
        return false;

    if (!initializeBaseFrame(verb))
        return false;

    return true;
}

template <class ROB, typename SCALAR>
bool PinocchioRBD<ROB, SCALAR>::initializeBaseFrame(const bool verb)
{
    std::string baseFrameName;
    size_t base_frame_id;

    // get the frame name of the robot base frame
    if (!extractFrameName<ROB>(ROB::Frames::BASE, baseFrameName))
        return false;

    if (!model_.existFrame(baseFrameName))
    {
        std::cout << "Pinocchio: base frame is defined in ROB, but not in URDF." << std::endl;
        return false;
    }

    base_frame_id = model_.getFrameId(baseFrameName);

    // update the kinematics
    pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
    pinocchio::updateFramePlacement(model_, data_, base_frame_id);

    // get pose of base frame in world
    w_pose_base_.setFromRotationMatrix(kindr::RotationMatrix<SCALAR>(data_.oMf[base_frame_id].rotation_impl()));
    w_pose_base_.position().toImplementation() = data_.oMf[base_frame_id].translation_impl();

    if (verb)
    {
        std::cout << "Pinocchio interface: pose of the robot base in the world is: " << std::endl;
        w_pose_base_.print();
    }

    return true;
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::assignJointLimits(const Model_t& mdl, const bool verb)
{
    double lower = std::numeric_limits<double>::lowest();
    double upper = std::numeric_limits<double>::max();

    for (int i = 1; i < mdl.njoints; i++)  // first joint is universe -> ignore
    {
        JointModel_t joint = mdl.joints[i];
        std::string s_name = joint.shortname();

        if ((s_name == "JointModelRX") || (s_name == "JointModelRY") || (s_name == "JointModelRZ") ||
            (s_name == "JointModelPX") || (s_name == "JointModelPY") || (s_name == "JointModelPZ"))
        {
            // Bounded 1-DOF joint
            this->p_lim_lower_(joint.idx_v()) = mdl.lowerPositionLimit(joint.idx_q());
            this->p_lim_upper_(joint.idx_v()) = mdl.upperPositionLimit(joint.idx_q());
        }
        else if ((s_name == "JointModelRUBX") || (s_name == "JointModelRUBY") || (s_name == "JointModelRUBZ"))
        {
            // Unbounded 1-DOF joint
            this->p_lim_lower_(joint.idx_v()) = lower;
            this->p_lim_upper_(joint.idx_v()) = upper;
        }
    }

    if (verb)
    {
        std::cerr << "Read lower joint limit as " << this->p_lim_lower_.transpose() << std::endl;
        std::cerr << "Read upper joint limit as " << this->p_lim_upper_.transpose() << std::endl;
    }
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::assignEffortLimits(const Model_t& mdl, const bool verb)
{
    this->tau_lim_ = mdl.effortLimit;

    if (verb)
    {
        std::cout << "Read effort limit as " << this->tau_lim_.transpose() << std::endl;
    }
}

template <class ROB, typename SCALAR>
void PinocchioRBD<ROB, SCALAR>::assignVelocityLimits(const Model_t& mdl, const bool verb)
{
    this->v_lim_ = mdl.velocityLimit;

    if (verb)
    {
        std::cout << "Read velocity limit as " << this->v_lim_.transpose() << std::endl;
    }
}

template <class ROB, typename SCALAR>
auto PinocchioRBD<ROB, SCALAR>::toPinocchioConfigVector(const JointPosition_t& p)
{
    return pinocchio::integrate(model_, pinocchio::neutral(model_), p);
}

}  // namespace rbd
}  // namespace ct