
/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

class ExampleRobot
{
public:
    /*!
     * @brief number of actuated robot arm joints 
     */
    static constexpr size_t NJOINTS = 7;

    /*!
     * @brief names of the actuated robot arm joints
     */
    static const std::vector<std::string> ControlledJoints;

    /*!
     * @brief enumeration of link frames
     */
    enum Frames
    {
        BASE = 0,
        LINK1,
        LINK2,
        LINK3,
        LINK4,
        LINK5,
        LINK6,
        LINK7,
        LINK8,
        TOOL
    };
    /*!
     * @brief map frame enumerations to frame names from the semantic robot description
    */
    static const std::map<Frames, std::string> frameMap;
};

const std::vector<std::string> ExampleRobot::ControlledJoints{"robot_joint_1", "robot_joint_2", "robot_joint_3",
    "robot_joint_4", "robot_joint_5", "robot_joint_6", "robot_joint_7"};

const std::map<ExampleRobot::Frames, std::string> ExampleRobot::frameMap{
    {std::make_pair(ExampleRobot::Frames::BASE, "robot_link_0"),
        std::make_pair(ExampleRobot::Frames::LINK1, "robot_link_1"),
        std::make_pair(ExampleRobot::Frames::LINK2, "robot_link_2"),
        std::make_pair(ExampleRobot::Frames::LINK3, "robot_link_3"),
        std::make_pair(ExampleRobot::Frames::LINK4, "robot_link_4"),
        std::make_pair(ExampleRobot::Frames::LINK5, "robot_link_5"),
        std::make_pair(ExampleRobot::Frames::LINK6, "robot_link_6"),
        std::make_pair(ExampleRobot::Frames::LINK7, "robot_link_7"),
        std::make_pair(ExampleRobot::Frames::LINK8, "robot_link_8"),
        std::make_pair(ExampleRobot::Frames::TOOL, "robot_tool")}};