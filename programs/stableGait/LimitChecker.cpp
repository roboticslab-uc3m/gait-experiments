// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LimitChecker.hpp"

LimitChecker::LimitChecker(FootSpec _footSpec, double _tolerance, rl::ICartesianControl * _leftLeg, rl::ICartesianControl * _rightLeg)
    : leftLeg(_leftLeg),
      rightLeg(_rightLeg),
      footSpec(_footSpec),
      tolerance(_tolerance)
{
    leftLeg->stat(initialLeft);
    rightLeg->stat(initialRight);
}

void LimitChecker::estimateParameters(GaitSpec & gaitSpec)
{
    gaitSpec.squat = iterateSquat();
    gaitSpec.step = iterateStep(gaitSpec);
}

void LimitChecker::setReference(GaitSpec gaitSpec)
{
    referenceGaitSpec = gaitSpec;
}

bool LimitChecker::updateSpecs(GaitSpec & gaitSpec)
{
    gaitSpec.sep += tolerance;

    if (gaitSpec.sep <= initialLeft[1] * 2)
    {
        return true;
    }

    gaitSpec.sep = referenceGaitSpec.sep;
    gaitSpec.hop += tolerance;

    if (gaitSpec.hop < gaitSpec.squat)
    {
        return true;
    }

    gaitSpec.hop = referenceGaitSpec.hop;
    gaitSpec.squat -= tolerance;

    if (gaitSpec.squat > 0.0)
    {
        return true;
    }

    gaitSpec.squat = referenceGaitSpec.squat;
    gaitSpec.step -= tolerance;

    return gaitSpec.step > 0.0;
}

double LimitChecker::iterateSquat()
{
    std::vector<double> x = initialLeft;
    std::vector<double> q;

    do
    {
        x[2] += tolerance;
    }
    while (leftLeg->inv(x, q));

    return x[2] - initialLeft[2] - tolerance;
}

double LimitChecker::iterateStep(GaitSpec gaitSpec)
{
    std::vector<double> x = initialRight;
    x[1] = -(footSpec.margin + gaitSpec.sep + (footSpec.width / 2.0));
    std::vector<double> q;

    do
    {
        x[0] += tolerance;
    }
    while (rightLeg->inv(x, q));

    return (x[0] - tolerance) + footSpec.length - footSpec.margin - (footSpec.width / 2.0);
}
