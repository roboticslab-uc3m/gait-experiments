// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LimitChecker.hpp"

LimitChecker::LimitChecker(rl::ICartesianControl * _leftLeg, rl::ICartesianControl * _rightLeg)
    : leftLeg(_leftLeg),
      rightLeg(_rightLeg),
      squat(0.0),
      step(0.0),
      length(0.0),
      width(0.0),
      margin(0.0),
      sep(0.0),
      tolerance(DEFAULT_TOLERANCE),
      preset(false)
{}

void LimitChecker::configure(double _length, double _width, double _margin, double _sep, double _tolerance)
{
    length = _length;
    width = _width;
    margin = _margin;
    sep = _sep;
    tolerance = _tolerance;

    // TODO: validate geometric parameters

    leftLeg->stat(initialLeft);
    rightLeg->stat(initialRight);
}

void LimitChecker::offsetSquat(double offset)
{
    if (preset)
    {
        squat -= offset;
    }
}

void LimitChecker::estimateParameters(double * _squat, double * _step)
{
    if (!preset)
    {
        iterateSquat();
        preset = true;
    }

    iterateStep();

    *_squat = squat;
    *_step = step;
}

void LimitChecker::iterateSquat()
{
    std::vector<double> x = initialLeft;
    std::vector<double> q;

    do
    {
        x[2] += tolerance;
    }
    while (leftLeg->inv(x, q));

    squat = x[2] - initialLeft[2] - tolerance;
}

void LimitChecker::iterateStep()
{
    std::vector<double> x = initialRight;
    x[1] = -(margin + sep + (width / 2.0));
    std::vector<double> q;

    do
    {
        x[0] += tolerance;
    }
    while (rightLeg->inv(x, q));

    step = (x[0] - tolerance) + length - margin - (width / 2.0);
}
