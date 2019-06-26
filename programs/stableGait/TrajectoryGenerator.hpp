// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>

#include "GaitSpecs.hpp"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(FootSpec footSpec, double distance, double vel, double acc);
    void configure(const std::vector<KDL::Frame> & steps, const std::vector<KDL::Frame> & com);
    void generate(KDL::Trajectory_Composite & comTraj, KDL::Trajectory_Composite & leftTraj, KDL::Trajectory_Composite & rightTraj);

private:
    double getDuration(const KDL::Vector & v);
    KDL::Frame makeHop(const KDL::Frame & H1, const KDL::Frame & H2);

    FootSpec footSpec;
    double distance;
    double vel;
    double acc;
    double radius;
    double eqradius;
    std::vector<KDL::Frame> steps;
    std::vector<KDL::Frame> com;
};

#endif // __TRAJECTORY_GENERATOR_HPP__
