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

    KDL::VelocityProfile_Rectangular profStep(vel);

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));

    for (int i = 1; i < com.size() - 2; i = i + 3)
    {
        KDL::Path_RoundedComposite * pathCom1 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom1->Add(com[i]);
        pathCom1->Add(com[i + 1]);
        pathCom1->Finish();

        comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profStep.Clone(), getDuration(com[i + 1].p - com[i].p)));

        KDL::Path_RoundedComposite * pathCom2 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom2->Add(com[i + 1]);
        pathCom2->Add(com[i + 2]);
        pathCom2->Finish();

        comTraj.Add(new KDL::Trajectory_Segment(pathCom2, profStep.Clone(), getDuration(com[i + 2].p - com[i + 1].p)));

        KDL::Path_RoundedComposite * pathCom3 = new KDL::Path_RoundedComposite(radius, eqradius, orient.Clone());
        pathCom3->Add(com[i + 2]);
        pathCom3->Add(com[i + 3]);
        pathCom3->Finish();

        comTraj.Add(new KDL::Trajectory_Segment(pathCom3, profStep.Clone(), getDuration(com[i + 3].p - com[i + 2].p)));
    }

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp, DEFAULT_SQUAT_DURATION));
}

double TrajectoryGenerator::getDuration(const KDL::Vector & v)
{
    return v.x() / vel;
}
