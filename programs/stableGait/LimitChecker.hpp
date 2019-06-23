// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LIMIT_CHECKER_HPP__
#define __LIMIT_CHECKER_HPP__

#include <vector>

#include <ICartesianControl.h>

#include "FootSpec.hpp"

#define DEFAULT_TOLERANCE 0.005 // [m]

namespace rl = roboticslab;

/**
 * @ingroup gait-experiments-programs
 */
class LimitChecker
{
public:
    LimitChecker(rl::ICartesianControl * leftLeg, rl::ICartesianControl * rightLeg);

    void configure(FootSpec footSpec, double tolerance = DEFAULT_TOLERANCE);
    void offsetSquat(double offset);
    void estimateParameters(double * squat, double * step);

private:
    void iterateSquat();
    void iterateStep();

    rl::ICartesianControl * leftLeg;
    rl::ICartesianControl * rightLeg;

    std::vector<double> initialLeft;
    std::vector<double> initialRight;

    double squat;
    double step;

    FootSpec footSpec;
    double tolerance; // iteration magnitude

    bool preset;
};

#endif // __LIMIT_CHECKER_HPP__
