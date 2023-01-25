// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LIMIT_CHECKER_HPP__
#define __LIMIT_CHECKER_HPP__

#include <vector>

#include <ICartesianSolver.h>

#include "GaitSpecs.hpp"

namespace rl = roboticslab;

/**
 * @ingroup gait-experiments-programs
 */
class LimitChecker
{
public:
    LimitChecker(FootSpec footSpec, double tolerance,
                 rl::ICartesianSolver * leftLeg, rl::ICartesianSolver * rightLeg,
                 const std::vector<double> & initialJointLeft, const std::vector<double> & initialJointRight,
                 const std::vector<double> & initialCartLeft, const std::vector<double> & initialCartRight);

    void estimateParameters(GaitSpec & gaitSpec);
    void setReference(GaitSpec gaitSpec);
    bool updateSpecs(GaitSpec & gaitSpec);

private:
    double iterateSquat();
    double iterateStep(GaitSpec gaitSpec);

    rl::ICartesianSolver * leftLeg;
    rl::ICartesianSolver * rightLeg;

    const std::vector<double> & initialJointLeft;
    const std::vector<double> & initialJointRight;

    const std::vector<double> & initialCartLeft;
    const std::vector<double> & initialCartRight;

    FootSpec footSpec;
    GaitSpec referenceGaitSpec;
    double tolerance; // iteration magnitude
};

#endif // __LIMIT_CHECKER_HPP__
