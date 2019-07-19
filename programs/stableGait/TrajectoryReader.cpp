// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstring>
#include <istream>

#include <kdl/frames.hpp>

#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>

#include <kdl/utilities/error.h>
#include <kdl/utilities/error_stack.h>

namespace roboticslab
{

KDL::Trajectory * parseTrajectory(std::istream & is)
{
    KDL::IOTrace("Trajectory::Read");
    char storage[64];
    KDL::EatWord(is, "[", storage, sizeof(storage));
    KDL::Eat(is, '[');

    if (std::strcmp(storage, "SEGMENT") == 0)
    {
        KDL::IOTrace("SEGMENT");
        KDL::Path * geom = KDL::Path::Read(is);
        KDL::VelocityProfile * motprof = KDL::VelocityProfile::Read(is);
        KDL::EatEnd(is, ']');
        KDL::IOTracePop();
        KDL::IOTracePop();
        return new KDL::Trajectory_Segment(geom, motprof);
    }
    else if (std::strcmp(storage, "STATIONARY") == 0)
    {
        KDL::IOTrace("STATIONARY");
        double duration;
        KDL::Frame pos;
        is >> duration;
        is >> pos;
        KDL::EatEnd(is, ']');
        KDL::IOTracePop();
        KDL::IOTracePop();
        return new KDL::Trajectory_Stationary(duration, pos);
    }
    else if (std::strcmp(storage, "COMPOSITE") == 0)
    {
        KDL::IOTrace("COMPOSITE");
        KDL::Trajectory_Composite * tr = new KDL::Trajectory_Composite();
        int size;
        is >> size;
        for (int i = 0; i < size; i++)
        {
            tr->Add(parseTrajectory(is));
        }
        KDL::EatEnd(is, ']');
        KDL::IOTracePop();
        KDL::IOTracePop();
        return tr;
    }
    else
    {
        throw KDL::Error_MotionIO_Unexpected_Traj();
    }

    return NULL;
}

} // namespace roboticslab

