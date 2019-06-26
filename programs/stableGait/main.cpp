// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * \defgroup stableGait stableGait
 *
 * <b>Building</b>
 *
\verbatim
mkdir build; cd build; cmake ..
make -j$(nproc)
\endverbatim
 *
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
 *
\verbatim
[on terminal 1] yarp server
\endverbatim
 *
 * The following is an example for the simulated robot's legs:
 *
\verbatim
[on terminal 2] teoSim
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/leftLeg-kinematics.ini --local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/rightLeg-kinematics.ini --local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] stableGait --distance 1.0 --dry
\endverbatim
 */

#include <cmath>

#include <string>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>
#include <ColorDebug.h>

#include "GaitSpecs.hpp"
#include "LimitChecker.hpp"
#include "StepGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "TargetBuilder.hpp"

#define DEFAULT_TRAJ_VEL 0.175 // [m/s]
#define DEFAULT_TRAJ_ACC 5.0 // [m/s^2]
#define DEFAULT_CMD_PERIOD 0.05 // [s]
#define DEFAULT_FOOT_LENGTH 0.245 // [m]
#define DEFAULT_FOOT_WIDTH 0.14 // [m]
#define DEFAULT_FOOT_MARGIN 0.02 // [m]
#define DEFAULT_FOOT_STABLE 0.04 // [m]
#define DEFAULT_FOOT_LIFT 0.005 // [m]
#define DEFAULT_GAIT_SEP 0.06 // [m]
#define DEFAULT_GAIT_HOP 0.01 // [m]
#define DEFAULT_TOLERANCE 0.001 // [m]

namespace rl = roboticslab;

int main(int argc, char *argv[])
{
    // Find YARP network.

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    // Parse arguments.

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    CD_DEBUG("%s\n", rf.toString().c_str());

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();

    double distance = rf.check("distance", yarp::os::Value(1.0), "distance to travel [m]").asFloat64();
    double trajVel = rf.check("vel", yarp::os::Value(DEFAULT_TRAJ_VEL), "velocity [m/s]").asFloat64();
    double trajAcc = rf.check("acc", yarp::os::Value(DEFAULT_TRAJ_ACC), "acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(DEFAULT_CMD_PERIOD), "command period [s]").asFloat64();
    double footLength = rf.check("length", yarp::os::Value(DEFAULT_FOOT_LENGTH), "foot length [m]").asFloat64();
    double footWidth = rf.check("width", yarp::os::Value(DEFAULT_FOOT_WIDTH), "foot width [m]").asFloat64();
    double footMargin = rf.check("margin", yarp::os::Value(DEFAULT_FOOT_MARGIN), "foot stability outer margin [m]").asFloat64();
    double footStable = rf.check("stable", yarp::os::Value(DEFAULT_FOOT_STABLE), "foot stability inner margin [m]").asFloat64();
    double footLift = rf.check("lift", yarp::os::Value(DEFAULT_FOOT_LIFT), "lift [m]").asFloat64();
    double gaitSep = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SEP), "foot separation [m]").asFloat64();
    double gaitHop = rf.check("hop", yarp::os::Value(DEFAULT_GAIT_HOP), "hop [m]").asFloat64();
    double tolerance = rf.check("tolerance", yarp::os::Value(DEFAULT_TOLERANCE), "tolerance [m]").asFloat64();

    bool dryRun = rf.check("dry", "dry run");
    bool once = rf.check("once", "run once");

    if (distance <= 0.0)
    {
        CD_ERROR("Illegal argument: '--distance' must be greater than '0' (was '%f').\n", distance);
        return 1;
    }

    if (trajVel <= 0.0)
    {
        CD_ERROR("Illegal argument: '--vel' must be greater than '0' (was '%f').\n", trajVel);
        return 1;
    }

    if (trajAcc <= 0.0)
    {
        CD_ERROR("Illegal argument: '--acc' must be greater than '0' (was '%f').\n", trajAcc);
        return 1;
    }

    if (period <= 0.0)
    {
        CD_ERROR("Illegal argument: '--period' must be greater than '0' (was '%f').\n", period);
        return 1;
    }

    if (footLength <= 0.0)
    {
        CD_ERROR("Illegal argument: '--length' must be greater than '0' (was '%f').\n", footLength);
        return 1;
    }

    if (footWidth <= 0.0)
    {
        CD_ERROR("Illegal argument: '--width' must be greater than '0' (was '%f').\n", footWidth);
        return 1;
    }

    if (footMargin <= 0.0)
    {
        CD_ERROR("Illegal argument: '--margin' must be greater than '0' (was '%f').\n", footMargin);
        return 1;
    }

    if (footStable <= 0.0)
    {
        CD_ERROR("Illegal argument: '--stable' must be greater than '0' (was '%f').\n", footStable);
        return 1;
    }

    if (footLift < 0.0)
    {
        CD_ERROR("Illegal argument: '--lift' must be greater than or equal to '0' (was '%f').\n", footLift);
        return 1;
    }

    if (gaitSep <= 0.0)
    {
        CD_ERROR("Illegal argument: '--sep' must be greater than '0' (was '%f').\n", gaitSep);
        return 1;
    }

    if (gaitHop < 0.0)
    {
        CD_ERROR("Illegal argument: '--hop' must be greater than or equal to '0' (was '%f').\n", gaitHop);
        return 1;
    }

    if (tolerance < 0.0)
    {
        CD_ERROR("Illegal argument: '--tolerance' must be greater than '0' (was '%f').\n", tolerance);
        return 1;
    }

    // Create devices (left leg).

    yarp::os::Property leftLegDeviceOptions;
    leftLegDeviceOptions.put("device", "CartesianControlClient");
    leftLegDeviceOptions.put("cartesianRemote", robotPrefix + "/leftLeg/CartesianControl");
    leftLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/leftLeg");

    yarp::dev::PolyDriver leftLegDevice(leftLegDeviceOptions);

    if (!leftLegDevice.isValid())
    {
        CD_ERROR("Cartesian device (left leg) not available.\n");
        return 1;
    }

    rl::ICartesianControl * iCartesianControlLeftLeg;

    if (!leftLegDevice.view(iCartesianControlLeftLeg))
    {
        CD_ERROR("Cannot view iCartesianControlLeftLeg.\n");
        return 1;
    }

    if (!iCartesianControlLeftLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        CD_ERROR("Cannot preset streaming command (left leg).\n");
        return 1;
    }

    // Create devices (right leg).

    yarp::os::Property rightLegDeviceOptions;
    rightLegDeviceOptions.put("device", "CartesianControlClient");
    rightLegDeviceOptions.put("cartesianRemote", robotPrefix + "/rightLeg/CartesianControl");
    rightLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/rightLeg");

    yarp::dev::PolyDriver rightLegDevice(rightLegDeviceOptions);

    if (!rightLegDevice.isValid())
    {
        CD_ERROR("Cartesian device (right leg) not available.\n");
        return 1;
    }

    rl::ICartesianControl * iCartesianControlRightLeg;

    if (!rightLegDevice.view(iCartesianControlRightLeg))
    {
        CD_ERROR("Cannot view iCartesianControlRightLeg.\n");
        return 1;
    }

    if (!iCartesianControlRightLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        CD_ERROR("Cannot preset streaming command (right leg).\n");
        return 1;
    }

    // Initialize specs and components.

    std::vector<double> x_leftInitial;

    if (!iCartesianControlLeftLeg->stat(x_leftInitial))
    {
        CD_ERROR("Cannot stat left leg.\n");
        return 1;
    }

    x_leftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    KDL::Frame H_leftInitial = rl::KdlVectorConverter::vectorToFrame(x_leftInitial);

    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.margin = footMargin;
    footSpec.stable = footStable;
    footSpec.lift = footLift;

    GaitSpec gaitSpec;
    gaitSpec.sep = gaitSep;
    gaitSpec.hop = gaitHop;

    LimitChecker limitChecker(footSpec, tolerance, iCartesianControlLeftLeg, iCartesianControlRightLeg);
    limitChecker.estimateParameters(gaitSpec);
    limitChecker.setReference(gaitSpec);

    StepGenerator stepGenerator(footSpec, H_leftInitial);
    TrajectoryGenerator trajectoryGenerator(footSpec, distance, trajVel, trajAcc);
    TargetBuilder targetBuilder(iCartesianControlLeftLeg, iCartesianControlRightLeg);

    TargetBuilder::Targets pointsLeft, pointsRight;

    bool hasSolution = false;
    double maxDuration;

    do
    {
        CD_INFO("step: %f, squat: %f, hop: %f, sep: %f\n", gaitSpec.step, gaitSpec.squat, gaitSpec.hop, gaitSpec.sep);

        // Generate steps.

        stepGenerator.configure(gaitSpec);

        std::vector<KDL::Frame> steps, com;
        stepGenerator.generate(distance, steps, com);

        CD_INFO("Steps (%d, [x, y]):", steps.size());

        for (int i = 0; i < steps.size(); i++)
        {
            const KDL::Vector & p = steps[i].p;
            CD_INFO_NO_HEADER(" [%f %f]", p.x(), p.y());
        }

        CD_INFO_NO_HEADER("\n");

        CD_INFO("CoM (%d, [x, y, z]):", com.size());

        for (int i = 0; i < com.size(); i++)
        {
            const KDL::Vector & p = com[i].p;
            CD_INFO_NO_HEADER(" [%f %f %f]", p.x(), p.y(), p.z());
        }

        CD_INFO_NO_HEADER("\n");

        // Generate trajectories.

        trajectoryGenerator.configure(steps, com);

        KDL::Trajectory_Composite comTraj, leftTraj, rightTraj;

        try
        {
            trajectoryGenerator.generate(comTraj, leftTraj, rightTraj);
        }
        catch (const KDL::Error_MotionPlanning & e)
        {
            CD_WARNING("Error: %s.\n", e.Description());
            continue;
        }

        CD_INFO("CoM: %f [s], left: %f [s], right: %f [s]\n", comTraj.Duration(), leftTraj.Duration(), rightTraj.Duration());

        double minDuration = std::min(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
        maxDuration = std::max(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());

        if (maxDuration - minDuration > 1.0)
        {
            CD_WARNING("Duration difference exceeds 1.0 seconds: %f.\n", maxDuration - minDuration);
            continue;
        }

        // Build target points.

        targetBuilder.configure(&comTraj, &leftTraj, &rightTraj);
        targetBuilder.build(period, pointsLeft, pointsRight);

        if (!targetBuilder.validate(pointsLeft, pointsRight))
        {
            CD_WARNING("IK failed.\n");
            continue;
        }
        else
        {
            hasSolution = true;
            break;
        }
    }
    while (!once && limitChecker.updateSpecs(gaitSpec));

    if (!hasSolution)
    {
        CD_ERROR("No valid solution found.\n");
        return 1;
    }

    // Configure worker.

    if (!dryRun)
    {
        yarp::os::TimerSettings timerSettings(period, maxDuration / period, maxDuration);

        yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
        {
            iCartesianControlLeftLeg->movi(pointsLeft[event.runCount]);
            iCartesianControlRightLeg->movi(pointsRight[event.runCount]);

            return true;
        };

        yarp::os::Timer timer(timerSettings, callback, true);

        // Execute trajectory.

        if (timer.start())
        {
            yarp::os::Time::delay(maxDuration);
            timer.stop();
        }
    }

    return 0;
}
