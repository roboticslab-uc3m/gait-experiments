// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LIMIT_CHECKER_HPP__
#define __LIMIT_CHECKER_HPP__

#include <vector>

#include <ICartesianControl.h>

#include "GaitSpecs.hpp"

namespace rl = roboticslab;

/**
 * @ingroup gait-experiments-programs
 */
class LimitChecker
{
public:
    LimitChecker(FootSpec footSpec, double tolerance, rl::ICartesianControl * leftLeg, rl::ICartesianControl * rightLeg);
    void estimateParameters(GaitSpec & gaitSpec);
    void setReference(GaitSpec gaitSpec);
    bool updateSpecs(GaitSpec & gaitSpec);

private:
    double iterateSquat();
    double iterateStep(GaitSpec gaitSpec);

    rl::ICartesianControl * leftLeg;
    rl::ICartesianControl * rightLeg;

    std::vector<double> initialLeft;
    std::vector<double> initialRight;

    FootSpec footSpec;
    GaitSpec referenceGaitSpec;
    double tolerance; // iteration magnitude
};

#endif // __LIMIT_CHECKER_HPP__
