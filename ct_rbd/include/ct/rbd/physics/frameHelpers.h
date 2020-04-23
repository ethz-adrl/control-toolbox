
#pragma once

namespace ct {
namespace rbd {

/*
 * This helper functions extracts the frame name from the frame-mapping in the robot description class.
 */
template <class RBD>
bool extractFrameName(const typename RBD::Frames fr, std::string& name)
{
    auto pos = RBD::frameMap.find(fr);
    if (pos == RBD::frameMap.end())
    {
        std::cerr << "Fatal, no frame with id " << fr << " to be found in robot's description class!" << std::endl;
        return false;
    }
    else
        name = pos->second;

    return true;
}

/*
 * This helper functions extracts the frame from the frame-mapping in the robot description class, given a frame name
 * @warning this method here uses linear search for getting the enum, and should be used carefully
 */
template <class RBD>
bool extractFrameEnum(const std::string& name, typename RBD::Frames& fr)
{
    if (!name.empty())
    {
        for (auto& i : RBD::frameMap)
        {
            if (i.second == name)
            {
                fr = i.first;
                return true;
            }
        }
    }
    return false;
}

}  // namespace rbd
}  // namespace ct
