// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STEP_GENERATOR_HPP__
#define __STEP_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>

#include "FootSpec.hpp"

class StepGenerator
{
public:
    StepGenerator(FootSpec footSpec);
    void configure(double step, double initialSep);
    void generate(double distance, std::vector<KDL::Frame> & stepsLeft, std::vector<KDL::Frame> & stepsRight);

private:
    FootSpec footSpec;
    double step;
    double initialSep;
};

#endif // __STEP_GENERATOR_HPP__
