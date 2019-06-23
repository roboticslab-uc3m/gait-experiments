// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STEP_GENERATOR_HPP__
#define __STEP_GENERATOR_HPP__

#include "FootSpec.hpp"

class StepGenerator
{
public:
    StepGenerator(FootSpec footSpec);

private:
    FootSpec footSpec;
};

#endif // __STEP_GENERATOR_HPP__
