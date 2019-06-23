// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StepGenerator.hpp"

#include <cmath>

StepGenerator::StepGenerator(FootSpec _footSpec)
    : footSpec(_footSpec),
      step(0.0),
      initialSep(0.0)
{}

void StepGenerator::configure(double _step, double _initialSep)
{
    step = _step;
    initialSep = _initialSep;
}

void StepGenerator::generate(double distance, std::vector<KDL::Frame> & stepsLeft, std::vector<KDL::Frame> & stepsRight)
{
    stepsRight.empty();
    stepsLeft.empty();

    stepsRight.push_back(KDL::Frame(KDL::Vector(0, initialSep, 0)));
    stepsLeft.push_back(KDL::Frame(KDL::Vector(0, -initialSep, 0)));

    if (distance <= step)
    {
        stepsRight.push_back(KDL::Frame(KDL::Vector(distance, -initialSep, 0)));
        stepsLeft.push_back(KDL::Frame(KDL::Vector(distance, initialSep, 0)));
        return;
    }

    double travelled = 0.0;
    bool movingRightFoot = true;
    double sep = footSpec.sep + footSpec.width / 2.0;

    do
    {
        travelled += step;
        bool isLastStep = travelled >= distance;
        travelled = std::min(travelled, distance);

        if (isLastStep)
        {
            stepsRight.push_back(KDL::Frame(KDL::Vector(distance, -initialSep, 0)));
            stepsLeft.push_back(KDL::Frame(KDL::Vector(distance, initialSep, 0)));
            break;
        }

        if (movingRightFoot)
        {
            stepsRight.push_back(KDL::Frame(KDL::Vector(travelled, -sep, 0)));
        }
        else
        {
            stepsLeft.push_back(KDL::Frame(KDL::Vector(travelled, sep, 0)));
        }

        movingRightFoot = !movingRightFoot;
    }
    while (true);
}
