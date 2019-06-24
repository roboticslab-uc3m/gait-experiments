// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryGenerator.hpp"

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

    KDL::Path_RoundedComposite * pathSquatDown = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
    KDL::Path_RoundedComposite * pathSquatUp = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());

    pathSquatDown->Add(com[0]);
    pathSquatDown->Add(com[1]);
    pathSquatDown->Finish();

    pathSquatUp->Add(com[com.size() - 2]);
    pathSquatUp->Add(com[com.size() - 1]);
    pathSquatUp->Finish();

    KDL::VelocityProfile * profSquatDown = new KDL::VelocityProfile_TrapHalf(vel, acc, true);
    KDL::VelocityProfile * profSquatUp = new KDL::VelocityProfile_TrapHalf(vel, acc, false);

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));

    rightTraj.Add(new KDL::Trajectory_Stationary(profSquatDown->Duration(), steps[0]));
    leftTraj.Add(new KDL::Trajectory_Stationary(profSquatDown->Duration(), steps[1]));

    KDL::VelocityProfile_Rectangular profCom(vel);
    KDL::VelocityProfile_Trap profStep(vel, acc);

    bool movingRightFoot = true;
    int i, stepN;
    double duration0;

    for (i = 1, stepN = (i - 1) / 3, duration0 = getDuration(com[i - 1].p - com[i].p); i < com.size() - 2; i += 3)
    {
        KDL::Path_RoundedComposite * pathCom1 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom1->Add(com[i]);
        pathCom1->Add(com[i + 1]);
        pathCom1->Finish();

        double duration1 = getDuration(com[i + 1].p - com[i].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profCom.Clone(), duration1));

        KDL::Path_RoundedComposite * pathCom2 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom2->Add(com[i + 1]);
        pathCom2->Add(com[i + 2]);
        pathCom2->Finish();

        double duration2 = getDuration(com[i + 2].p - com[i + 1].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom2, profCom.Clone(), duration2));

        KDL::Path_RoundedComposite * pathCom3 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom3->Add(com[i + 2]);
        pathCom3->Add(com[i + 3]);
        pathCom3->Finish();

        double duration3 = getDuration(com[i + 3].p - com[i + 2].p);
        comTraj.Add(new KDL::Trajectory_Segment(pathCom3, profCom.Clone(), duration3));

        KDL::Path_RoundedComposite * pathStep = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathStep->Add(steps[stepN]);
        pathStep->Add(makeHop(steps[stepN], steps[stepN + 2]));
        pathStep->Add(steps[stepN + 2]);
        pathStep->Finish();

        if (i == 1)
        {
            duration0 = 0.0;
        }

        double durationMovingUp = duration0 + duration1;
        double durationMovingDown = duration2;
        double durationStationary = duration0 + duration1 + duration2;

        if (movingRightFoot)
        {
            rightTraj.Add(new KDL::Trajectory_Segment(pathStep, profStep.Clone(), durationMovingUp));
            rightTraj.Add(new KDL::Trajectory_Stationary(durationMovingDown, steps[stepN + 2]));
            leftTraj.Add(new KDL::Trajectory_Stationary(durationStationary, steps[stepN + 1]));
        }
        else
        {
            leftTraj.Add(new KDL::Trajectory_Segment(pathStep, profStep.Clone(), durationMovingUp));
            leftTraj.Add(new KDL::Trajectory_Stationary(durationMovingDown, steps[stepN + 2]));
            rightTraj.Add(new KDL::Trajectory_Stationary(durationStationary, steps[stepN + 1]));
        }

        movingRightFoot = !movingRightFoot;
    }

    KDL::Path_RoundedComposite * pathLastStep = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());

    if (movingRightFoot)
    {
        rightTraj.Add(new KDL::Trajectory_Segment(pathLastStep, profStep.Clone(), duration0));
        leftTraj.Add(new KDL::Trajectory_Stationary(duration0, steps[stepN + 1]));
    }
    else
    {
        leftTraj.Add(new KDL::Trajectory_Segment(pathLastStep, profStep.Clone(), duration0));
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
    return H1 * KDL::Frame(H1.M, H2.p - H1.p) * KDL::Frame(H1.M, KDL::Vector(0, 0, footSpec.hop));
}
