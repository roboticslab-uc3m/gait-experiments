// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryGenerator.hpp"

#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>

#define DEFAULT_SQUAT_DURATION 5.0 // [s]
#define DEFAULT_RADIUS 0.005 // [m]
#define DEFAULT_EQ_RADIUS 1.0 // [m]

TrajectoryGenerator::TrajectoryGenerator(FootSpec _footSpec, double _distance, double _vel, double _acc)
    : footSpec(_footSpec),
      distance(_distance),
      vel(_vel),
      acc(_acc),
      radius(DEFAULT_RADIUS),
      eqradius(DEFAULT_EQ_RADIUS)
{}

void TrajectoryGenerator::configure(const std::vector<KDL::Frame> & _steps, const std::vector<KDL::Frame> & _com)
{
    steps = _steps;
    com = _com;
}

void TrajectoryGenerator::generate(KDL::Trajectory_Composite & comTraj, KDL::Trajectory_Composite & leftTraj, KDL::Trajectory_Composite & rightTraj)
{
    KDL::RotationalInterpolation_SingleAxis orient;

    KDL::Path_Composite * pathSquatDown = new KDL::Path_Composite();
    KDL::Path_Composite * pathSquatUp = new KDL::Path_Composite();

    pathSquatDown->Add(new KDL::Path_Line(com[0], com[1], orient.Clone(), eqradius));
    pathSquatDown->Add(new KDL::Path_Line(com[1], com[2], orient.Clone(), eqradius));

    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 3], com[com.size() - 2], orient.Clone(), eqradius));
    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 2], com[com.size() - 1], orient.Clone(), eqradius));

    // FIXME: hardcoded, check behavior on simulator
    KDL::VelocityProfile * profSquatDown = new KDL::VelocityProfile_TrapHalf(0.05, 0.2, true);
    KDL::VelocityProfile * profSquatUp = new KDL::VelocityProfile_TrapHalf(0.05, 0.2, false);

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));

    rightTraj.Add(new KDL::Trajectory_Stationary(profSquatDown->Duration(), steps[0]));
    leftTraj.Add(new KDL::Trajectory_Stationary(profSquatDown->Duration(), steps[1]));

    KDL::VelocityProfile_Rectangular profRect(vel);

    bool movingRightFoot = true;
    int i, stepN;
    double duration0;

    for (i = 2; i < com.size() - 3; i += 3)
    {
        duration0 = getDuration(com[i].p - com[i - 1].p);

        KDL::Path_Composite * pathCom1 = new KDL::Path_Composite();
        pathCom1->Add(new KDL::Path_Line(com[i], com[i + 1], orient.Clone(), eqradius));

        double duration1 = getDuration(com[i + 1].p - com[i].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profRect.Clone(), duration1));

        KDL::Path_Composite * pathCom2 = new KDL::Path_Composite();
        pathCom2->Add(new KDL::Path_Line(com[i + 1], com[i + 2], orient.Clone(), eqradius));

        double duration2 = getDuration(com[i + 2].p - com[i + 1].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom2, profRect.Clone(), duration2));

        KDL::Path_Composite * pathCom3 = new KDL::Path_Composite();
        pathCom3->Add(new KDL::Path_Line(com[i + 2], com[i + 3], orient.Clone(), eqradius));

        double duration3 = getDuration(com[i + 3].p - com[i + 2].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom3, profRect.Clone(), duration3));

        stepN = (i - 2) / 3;

        KDL::Path_RoundedComposite * pathStep = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathStep->Add(steps[stepN]);
        pathStep->Add(makeHop(steps[stepN], steps[stepN + 2]));
        pathStep->Add(steps[stepN + 2]);
        pathStep->Finish();

        if (i == 2)
        {
            duration0 = 0.0;
        }

        double durationMovingUp = duration0 + duration1;
        double durationMovingDown = duration2;
        double durationStationary = duration0 + duration1 + duration2;

        if (movingRightFoot)
        {
            rightTraj.Add(new KDL::Trajectory_Segment(pathStep, profRect.Clone(), durationMovingUp));
            rightTraj.Add(new KDL::Trajectory_Stationary(durationMovingDown, steps[stepN + 2]));
            leftTraj.Add(new KDL::Trajectory_Stationary(durationStationary, steps[stepN + 1]));
        }
        else
        {
            leftTraj.Add(new KDL::Trajectory_Segment(pathStep, profRect.Clone(), durationMovingUp));
            leftTraj.Add(new KDL::Trajectory_Stationary(durationMovingDown, steps[stepN + 2]));
            rightTraj.Add(new KDL::Trajectory_Stationary(durationStationary, steps[stepN + 1]));
        }

        movingRightFoot = !movingRightFoot;
    }

    duration0 = getDuration(com[i].p - com[i - 1].p);
    stepN = (i - 2) / 3;

    KDL::Path_RoundedComposite * pathLastStep = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
    pathLastStep->Add(steps[stepN]);
    pathLastStep->Add(makeHop(steps[stepN], steps[stepN + 2]));
    pathLastStep->Add(steps[stepN + 2]);
    pathLastStep->Finish();

    if (movingRightFoot)
    {
        rightTraj.Add(new KDL::Trajectory_Segment(pathLastStep, profRect.Clone(), duration0));
        leftTraj.Add(new KDL::Trajectory_Stationary(duration0, steps[stepN + 1]));
    }
    else
    {
        leftTraj.Add(new KDL::Trajectory_Segment(pathLastStep, profRect.Clone(), duration0));
        rightTraj.Add(new KDL::Trajectory_Stationary(duration0, steps[stepN + 1]));
    }

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp, DEFAULT_SQUAT_DURATION));

    rightTraj.Add(new KDL::Trajectory_Stationary(profSquatUp->Duration(), steps[steps.size() - 2]));
    leftTraj.Add(new KDL::Trajectory_Stationary(profSquatUp->Duration(), steps[steps.size() - 1]));
}

double TrajectoryGenerator::getDuration(const KDL::Vector & v)
{
    return v.x() / vel;
}

KDL::Frame TrajectoryGenerator::makeHop(const KDL::Frame & H1, const KDL::Frame & H2)
{
    KDL::Vector diff = H2.p - H1.p;
    return H1 * KDL::Frame(H1.M, 0.5 * diff) * KDL::Frame(H1.M, KDL::Vector(0, 0, footSpec.lift));
}
