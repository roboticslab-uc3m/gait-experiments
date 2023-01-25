// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TARGET_BUILDER_HPP__
#define __TARGET_BUILDER_HPP__

#include <vector>

#include <kdl/trajectory.hpp>

#include <ICartesianSolver.h>

namespace rl = roboticslab;

class TargetBuilder
{
public:
    using Targets = std::vector<std::vector<double>>;

    TargetBuilder(rl::ICartesianSolver * solverLeft, rl::ICartesianSolver * solverRight,
                  const std::vector<double> & initialJointLeft, const std::vector<double> & initialJointRight);

    void configure(KDL::Trajectory * tCom, KDL::Trajectory * tLeft, KDL::Trajectory * tRight);
    void build(double period, Targets & vLeft, Targets & vRight);
    bool validate(const Targets & vLeft, const Targets & vRight);

private:
    rl::ICartesianSolver * solverLeft;
    rl::ICartesianSolver * solverRight;

    const std::vector<double> & initialJointLeft;
    const std::vector<double> & initialJointRight;

    KDL::Trajectory * tCom;
    KDL::Trajectory * tLeft;
    KDL::Trajectory * tRight;

    double maxTime;
};

#endif // __TARGET_BUILDER_HPP__
