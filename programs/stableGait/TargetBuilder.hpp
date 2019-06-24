// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TARGET_BUILDER_HPP__
#define __TARGET_BUILDER_HPP__

#include <vector>

#include <kdl/trajectory.hpp>

#include <ICartesianControl.h>

namespace rl = roboticslab;

class TargetBuilder
{
public:
    typedef std::vector< std::vector<double> > Targets;
    TargetBuilder(rl::ICartesianControl * iCartLeft, rl::ICartesianControl * iCartRight);
    void configure(KDL::Trajectory * tCom, KDL::Trajectory * tLeft, KDL::Trajectory * tRight);
    void build(double period, Targets & vLeft, Targets & vRight);
    bool validate(Targets & vLeft, Targets & vRight);

private:
    rl::ICartesianControl * iCartLeft;
    rl::ICartesianControl * iCartRight;
    KDL::Trajectory * tCom;
    KDL::Trajectory * tLeft;
    KDL::Trajectory * tRight;
    double maxTime;
};

#endif // __TARGET_BUILDER_HPP__
