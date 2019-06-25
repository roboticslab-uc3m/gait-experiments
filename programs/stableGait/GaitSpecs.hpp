// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GAIT_SPECS_HPP__
#define __GAIT_SPECS_HPP__

struct GaitSpec
{
    double squat;
    double step;
    double sep;
    double hop;
};

struct FootSpec
{
    double length;
    double width;
    double margin;
    double stable;
    double lift;
};

#endif // __GAIT_SPECS_HPP__
