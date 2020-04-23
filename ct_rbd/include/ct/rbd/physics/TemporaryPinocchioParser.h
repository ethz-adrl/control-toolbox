#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/parsers/urdf/types.hpp"
#include "pinocchio/parsers/urdf.hpp"

/**
 * @brief TODO: replace this by the pinocchio-internal reduced joints parsing
 * 
 */
namespace pinocchio {
namespace urdf {

using namespace details;

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void parseTree(::urdf::LinkConstSharedPtr link,
    ModelTpl<Scalar, Options, JointCollectionTpl>& model,
    const std::vector<std::string>& movable_joints,
    bool verbose)
{
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointCollection JointCollection;
    typedef typename Model::SE3 SE3;
    typedef typename Model::FrameIndex FrameIndex;

    // Parent joint of the current body
    ::urdf::JointConstSharedPtr joint = ::urdf::const_pointer_cast<::urdf::Joint>(link->parent_joint);

    if (joint)  // if the link is not the root of the tree
    {
        assert(link->getParent());

        const std::string& joint_name = joint->name;
        const std::string& link_name = link->name;
        const std::string& parent_link_name = link->getParent()->name;
        std::ostringstream joint_info;

        FrameIndex parentFrameId = getParentJointFrame(link, model);

        // Transformation from the parent link to the joint origin
        const SE3 jointPlacement = convertFromUrdf(joint->parent_to_joint_origin_transform).template cast<Scalar>();

        const ::urdf::InertialSharedPtr Y = ::urdf::const_pointer_cast<::urdf::Inertial>(link->inertial);

        // check if the current joint is NOT listed as movable joint
        if (std::find(movable_joints.begin(), movable_joints.end(), joint_name) == movable_joints.end())
        {
            if (verbose)
                std::cout << "Now modifying joint " << joint_name << " to be fixed." << std::endl;

            // if it is not listed as movable joint, we modify it to be FIXED.
            joint_info << "fixed joint";
            details::addFixedJointAndBody(model, parentFrameId, jointPlacement, joint_name, Y, link_name);
        }
        else
        {
            switch (joint->type)
            {
                case ::urdf::Joint::FLOATING:
                    joint_info << "joint FreeFlyer";
                    addJointAndBody(model, typename JointCollection::JointModelFreeFlyer(), parentFrameId,
                        jointPlacement, joint->name, Y, link->name);

                    break;

                case ::urdf::Joint::REVOLUTE:
                {
                    joint_info << "joint REVOLUTE with axis";

                    typedef JointModelRX::ConfigVector_t ConfigVector_t;
                    typedef JointModelRX::TangentVector_t TangentVector_t;

                    TangentVector_t max_effort;
                    TangentVector_t max_velocity;
                    ConfigVector_t lower_position;
                    ConfigVector_t upper_position;

                    if (joint->limits)
                    {
                        max_effort << joint->limits->effort;
                        max_velocity << joint->limits->velocity;
                        lower_position << joint->limits->lower;
                        upper_position << joint->limits->upper;
                    }

                    CartesianAxis axis = extractCartesianAxis(joint->axis);

                    switch (axis)
                    {
                        case AXIS_X:
                            joint_info << " along X";
                            addJointAndBody(model, typename JointCollection::JointModelRX(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Y:
                            joint_info << " along Y";
                            addJointAndBody(model, typename JointCollection::JointModelRY(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Z:
                            joint_info << " along Z";
                            addJointAndBody(model, typename JointCollection::JointModelRZ(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_UNALIGNED:
                        {
                            typename SE3::Vector3 joint_axis(
                                (Scalar)joint->axis.x, (Scalar)joint->axis.y, (Scalar)joint->axis.z);
                            joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                            addJointAndBody(model,
                                typename JointCollection::JointModelRevoluteUnaligned(joint_axis.normalized()),
                                parentFrameId, jointPlacement, joint->name, Y, link->name, max_effort, max_velocity,
                                lower_position, upper_position);
                            break;
                        }
                        default:
                            assert(false && "The axis type of the revolute joint is of wrong type.");
                            break;
                    }
                    break;
                }

                case ::urdf::Joint::CONTINUOUS:  // Revolute joint with no joint limits
                {
                    joint_info << "joint CONTINUOUS with axis";

                    typedef JointModelRUBX::ConfigVector_t ConfigVector_t;
                    typedef JointModelRUBX::TangentVector_t TangentVector_t;

                    TangentVector_t max_effort;
                    TangentVector_t max_velocity;
                    const ConfigVector_t::Scalar u = 1.01;
                    ConfigVector_t lower_position(-u, -u);
                    ConfigVector_t upper_position(u, u);

                    if (joint->limits)
                    {
                        max_effort << joint->limits->effort;
                        max_velocity << joint->limits->velocity;
                    }

                    CartesianAxis axis = extractCartesianAxis(joint->axis);

                    switch (axis)
                    {
                        case AXIS_X:
                            joint_info << " along X";
                            addJointAndBody(model, typename JointCollection::JointModelRUBX(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Y:
                            joint_info << " along Y";
                            addJointAndBody(model, typename JointCollection::JointModelRUBY(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Z:
                            joint_info << " along Z";
                            addJointAndBody(model, typename JointCollection::JointModelRUBZ(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_UNALIGNED:
                        {
                            typename SE3::Vector3 joint_axis(
                                (Scalar)joint->axis.x, (Scalar)joint->axis.y, (Scalar)joint->axis.z);
                            joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                            typedef
                                typename JointCollection::JointModelRevoluteUnaligned::ConfigVector_t ConfigVector_t;

                            const Scalar infty = std::numeric_limits<Scalar>::infinity();
                            ConfigVector_t lower_position(ConfigVector_t::Constant(-infty));
                            ConfigVector_t upper_position(ConfigVector_t::Constant(infty));

                            addJointAndBody(model,
                                typename JointCollection::JointModelRevoluteUnaligned(joint_axis.normalized()),
                                parentFrameId, jointPlacement, joint->name, Y, link->name, max_effort, max_velocity,
                                lower_position, upper_position);
                            break;
                        }

                        default:
                            assert(false && "The axis type of the revolute joint is of wrong type.");
                            break;
                    }
                    break;
                }

                case ::urdf::Joint::PRISMATIC:
                {
                    joint_info << "joint PRISMATIC with axis";

                    typedef JointModelRX::ConfigVector_t ConfigVector_t;
                    typedef JointModelRX::TangentVector_t TangentVector_t;

                    TangentVector_t max_effort;
                    TangentVector_t max_velocity;
                    ConfigVector_t lower_position;
                    ConfigVector_t upper_position;

                    if (joint->limits)
                    {
                        max_effort << joint->limits->effort;
                        max_velocity << joint->limits->velocity;
                        lower_position << joint->limits->lower;
                        upper_position << joint->limits->upper;
                    }

                    CartesianAxis axis = extractCartesianAxis(joint->axis);
                    switch (axis)
                    {
                        case AXIS_X:
                            joint_info << " along X";
                            addJointAndBody(model, typename JointCollection::JointModelPX(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Y:

                            joint_info << " along Y";
                            addJointAndBody(model, typename JointCollection::JointModelPY(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_Z:
                            joint_info << " along Z";
                            addJointAndBody(model, typename JointCollection::JointModelPZ(), parentFrameId,
                                jointPlacement, joint->name, Y, link->name, max_effort, max_velocity, lower_position,
                                upper_position);
                            break;

                        case AXIS_UNALIGNED:
                        {
                            typename SE3::Vector3 joint_axis(
                                (Scalar)joint->axis.x, (Scalar)joint->axis.y, (Scalar)joint->axis.z);
                            joint_info << " unaligned along (" << joint_axis.transpose() << ")";

                            addJointAndBody(model,
                                typename JointCollection::JointModelPrismaticUnaligned(joint_axis.normalized()),
                                parentFrameId, jointPlacement, joint->name, Y, link->name, max_effort, max_velocity,
                                lower_position, upper_position);
                            break;
                        }

                        default:
                            assert(false && "The axis type of the prismatic joint is of wrong type.");
                            break;
                    }
                    break;
                }

                case ::urdf::Joint::PLANAR:
                {
                    joint_info << "joint PLANAR with normal axis along Z";

                    typedef JointModelPlanar::ConfigVector_t ConfigVector_t;
                    typedef JointModelPlanar::TangentVector_t TangentVector_t;

                    TangentVector_t max_effort;
                    TangentVector_t max_velocity;
                    ConfigVector_t lower_position;
                    ConfigVector_t upper_position;

                    if (joint->limits)
                    {
                        max_effort << joint->limits->effort;
                        max_velocity << joint->limits->velocity;
                        lower_position << joint->limits->lower;
                        upper_position << joint->limits->upper;
                    }

                    addJointAndBody(model, typename JointCollection::JointModelPlanar(), parentFrameId, jointPlacement,
                        joint->name, Y, link->name, max_effort, max_velocity, lower_position, upper_position);
                }
                break;


                case ::urdf::Joint::FIXED:
                    // In case of fixed joint, if link has inertial tag:
                    //    -add the inertia of the link to his parent in the model
                    // Otherwise do nothing.
                    // In all cases:
                    //    -let all the children become children of parent
                    //    -inform the parser of the offset to apply
                    //    -add fixed body in model to display it in gepetto-viewer

                    joint_info << "fixed joint";
                    details::addFixedJointAndBody(model, parentFrameId, jointPlacement, joint_name, Y, link_name);

                    break;

                default:
                {
                    const std::string exception_message("The type of joint " + joint_name + " is not supported.");
                    throw std::invalid_argument(exception_message);
                    break;
                }
            }
        }

        if (verbose)
        {
            const Inertia YY = (!Y) ? Inertia::Zero() : convertFromUrdf(*Y);
            std::cout << "Adding Body" << std::endl;
            std::cout << "\"" << link_name << "\" connected to "
                      << "\"" << parent_link_name << "\" throw joint "
                      << "\"" << joint_name << "\"" << std::endl;
            std::cout << "joint type: " << joint_info.str() << std::endl;
            std::cout << "joint placement:\n" << jointPlacement;
            std::cout << "body info: " << std::endl;
            std::cout << "  "
                      << "mass: " << YY.mass() << std::endl;
            std::cout << "  "
                      << "lever: " << YY.lever().transpose() << std::endl;
            std::cout << "  "
                      << "inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): " << YY.inertia().data().transpose() << std::endl
                      << std::endl;
        }
    }
    else if (link->getParent())
    {
        const std::string exception_message(link->name + " - joint information missing.");
        throw std::invalid_argument(exception_message);
    }

    BOOST_FOREACH (::urdf::LinkConstSharedPtr child, link->child_links)
    {
        parseTree(child, model, movable_joints, verbose);
    }
}


template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void parseRootTree(::urdf::LinkConstSharedPtr root_link,
    const std::vector<std::string>& movable_joints,
    ModelTpl<Scalar, Options, JointCollectionTpl>& model,
    const bool verbose)
{
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::SE3 SE3;

    details::addFixedJointAndBody(model, 0, SE3::Identity(), "root_joint", root_link->inertial, root_link->name);

    BOOST_FOREACH (::urdf::LinkConstSharedPtr child, root_link->child_links)
    {
        parseTree(child, model, movable_joints, verbose);
    }
}


template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
ModelTpl<Scalar, Options, JointCollectionTpl>& buildModel(const ::urdf::ModelInterfaceSharedPtr& urdfTree,
    const std::vector<std::string>& movable_joints,
    ModelTpl<Scalar, Options, JointCollectionTpl>& model,
    const bool verbose)
{
    assert(urdfTree);
    model.name = urdfTree->getName();
    parseRootTree(urdfTree->getRoot(), movable_joints, model, verbose);
    return model;
}


template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
ModelTpl<Scalar, Options, JointCollectionTpl>& buildModel(const std::string& filename,
    const std::vector<std::string>& movable_joints,
    ModelTpl<Scalar, Options, JointCollectionTpl>& model,
    const bool verbose)
{
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
    if (urdfTree)
    {
        return buildModel(urdfTree, movable_joints, model, verbose);
    }
    else
    {
        const std::string exception_message("The file " + filename + " does not contain a valid URDF model.");
        throw std::invalid_argument(exception_message);
    }

    return model;
}


}  // namespace urdf
}  // namespace pinocchio
