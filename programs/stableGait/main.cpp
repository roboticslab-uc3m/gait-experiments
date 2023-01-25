// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup stableGait stableGait
 * @brief An attempt to perform gait on a humanoid robot with simple, non-dynamic stability assumptions.
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
[on terminal 3] stableGait --distance 1.0 --dry
\endverbatim
 */

#include <cmath>

#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>

#include <ICartesianSolver.h>
#include <KdlVectorConverter.hpp>

#include "GaitSpecs.hpp"
#include "LimitChecker.hpp"
#include "StepGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "TargetBuilder.hpp"

constexpr auto DEFAULT_TRAJ_VEL = 0.175; // [m/s]
constexpr auto DEFAULT_TRAJ_ACC = 5.0; // [m/s^2]
constexpr auto DEFAULT_CMD_PERIOD = 0.05; // [s]
constexpr auto DEFAULT_FOOT_LENGTH = 0.245; // [m]
constexpr auto DEFAULT_FOOT_WIDTH = 0.14; // [m]
constexpr auto DEFAULT_FOOT_MARGIN = 0.02; // [m]
constexpr auto DEFAULT_FOOT_STABLE = 0.04; // [m]
constexpr auto DEFAULT_FOOT_LIFT = 0.005; // [m]
constexpr auto DEFAULT_GAIT_SEP = 0.06; // [m]
constexpr auto DEFAULT_GAIT_HOP = 0.01; // [m]
constexpr auto DEFAULT_TOLERANCE = 0.001; // [m]

namespace rl = roboticslab;

int main(int argc, char * argv[])
{
    // Find YARP network.

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    // Parse arguments.

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    yDebug() << rf.toString();

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();
    std::string kinematicsLeftLeg = rf.check("kinematicsLeftLeg", yarp::os::Value("teo-leftLeg.ini")).asString();
    std::string kinematicsRightLeg = rf.check("kinematicsRightLeg", yarp::os::Value("teo-rightLeg.ini")).asString();

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
        yError() << "Illegal argument: '--distance' must be greater than '0', was:" << distance;
        return 1;
    }

    if (trajVel <= 0.0)
    {
        yError() << "Illegal argument: '--vel' must be greater than '0', was:" << trajVel;
        return 1;
    }

    if (trajAcc <= 0.0)
    {
        yError() << "Illegal argument: '--acc' must be greater than '0', was:" << trajAcc;
        return 1;
    }

    if (period <= 0.0)
    {
        yError() << "Illegal argument: '--period' must be greater than '0', was:" << period;
        return 1;
    }

    if (footLength <= 0.0)
    {
        yError() << "Illegal argument: '--length' must be greater than '0', was:" << footLength;
        return 1;
    }

    if (footWidth <= 0.0)
    {
        yError() << "Illegal argument: '--width' must be greater than '0', was:" << footWidth;
        return 1;
    }

    if (footMargin <= 0.0)
    {
        yError() << "Illegal argument: '--margin' must be greater than '0', was:" << footMargin;
        return 1;
    }

    if (footStable <= 0.0)
    {
        yError() << "Illegal argument: '--stable' must be greater than '0', was:" << footStable;
        return 1;
    }

    if (footLift < 0.0)
    {
        yError() << "Illegal argument: '--lift' must be greater than or equal to '0', was:" << footLift;
        return 1;
    }

    if (gaitSep <= 0.0)
    {
        yError() << "Illegal argument: '--sep' must be greater than '0', was:" << gaitSep;
        return 1;
    }

    if (gaitHop < 0.0)
    {
        yError() << "Illegal argument: '--hop' must be greater than or equal to '0', was:" << gaitHop;
        return 1;
    }

    if (tolerance < 0.0)
    {
        yError() << "Illegal argument: '--tolerance' must be greater than '0', was:" << tolerance;
        return 1;
    }

    // Create robot device (left leg).

    yarp::os::Property leftLegDeviceOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotPrefix + "/leftLeg")},
        {"local", yarp::os::Value("/stableGait/leftLeg")}
    };

    yarp::dev::PolyDriver leftLegDevice(leftLegDeviceOptions);

    if (!leftLegDevice.isValid())
    {
        yError() << "Robot device (left leg) not available";
        return 1;
    }

    yarp::dev::IControlLimits * limitsLeftLeg;
    yarp::dev::IControlMode * modeLeftLeg;
    yarp::dev::IEncoders * encLeftLeg;
    yarp::dev::IPositionDirect * posdLeftLeg;

    if (!leftLegDevice.view(limitsLeftLeg) || !leftLegDevice.view(modeLeftLeg)
        || !leftLegDevice.view(encLeftLeg) || !leftLegDevice.view(posdLeftLeg))
    {
        yError() << "Cannot view interfaces (left leg)";
        return 1;
    }

    int numJointsLeftLeg;
    encLeftLeg->getAxes(&numJointsLeftLeg);

    std::vector<double> qLeftInitial(numJointsLeftLeg);
    int retry = 0;

    while (!encLeftLeg->getEncoders(qLeftInitial.data()))
    {
        if (++retry == 10)
        {
            yError() << "Failed to get initial joint pose (left leg)";
            return 1;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

    yInfo() << "Initial joint pose (left leg):" << qLeftInitial;

    yarp::os::Bottle bMinLeftLeg;
    yarp::os::Bottle bMaxLeftLeg;

    for (int joint = 0; joint < numJointsLeftLeg; joint++)
    {
        double qMin, qMax;

        if (!limitsLeftLeg->getLimits(joint, &qMin, &qMax))
        {
            yError() << "Unable to retrieve position limits for joint" << joint << "(left leg)";
            return 1;
        }

        bMinLeftLeg.addFloat64(qMin);
        bMaxLeftLeg.addFloat64(qMax);
    }

    yInfo() << "Joint position limits (left leg):" << bMinLeftLeg.toString() << bMaxLeftLeg.toString();

    if (!modeLeftLeg->setControlModes(std::vector(numJointsLeftLeg, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yError() << "Failed to set posd control mode (left leg)";
        return 1;
    }

    // Create robot device (right leg).

    yarp::os::Property rightLegDeviceOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotPrefix + "/rightLeg")},
        {"local", yarp::os::Value("/stableGait/rightLeg")}
    };

    yarp::dev::PolyDriver rightLegDevice(rightLegDeviceOptions);

    if (!rightLegDevice.isValid())
    {
        yError() << "Robot device (right leg) not available";
        return 1;
    }

    yarp::dev::IControlLimits * limitsRightLeg;
    yarp::dev::IControlMode * modeRightLeg;
    yarp::dev::IEncoders * encRightLeg;
    yarp::dev::IPositionDirect * posdRightLeg;

    if (!rightLegDevice.view(limitsRightLeg) || !rightLegDevice.view(modeRightLeg)
        || !rightLegDevice.view(encRightLeg) || !rightLegDevice.view(posdRightLeg))
    {
        yError() << "Cannot view interfaces (right leg)";
        return 1;
    }

    int numJointsRightLeg;
    encRightLeg->getAxes(&numJointsRightLeg);

    std::vector<double> qRightInitial(numJointsRightLeg);
    retry = 0;

    while (!encRightLeg->getEncoders(qRightInitial.data()))
    {
        if (++retry == 10)
        {
            yError() << "Failed to get initial joint pose (right leg)";
            return 1;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

    yInfo() << "Initial joint pose (right leg):" << qRightInitial;

    yarp::os::Bottle bMinRightLeg;
    yarp::os::Bottle bMaxRightLeg;

    for (int joint = 0; joint < numJointsRightLeg; joint++)
    {
        double qMin, qMax;

        if (!limitsRightLeg->getLimits(joint, &qMin, &qMax))
        {
            yError() << "Unable to retrieve position limits for joint" << joint << "(right leg)";
            return 1;
        }

        bMinRightLeg.addFloat64(qMin);
        bMaxRightLeg.addFloat64(qMax);
    }

    yInfo() << "Joint position limits (right leg):" << bMinRightLeg.toString() << bMaxRightLeg.toString();

    if (!modeRightLeg->setControlModes(std::vector(numJointsRightLeg, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yError() << "Failed to set posd control mode (right leg)";
        return 1;
    }

    // Create solver device (left leg).

    yarp::os::Property leftLegSolverOptions {
        {"device", yarp::os::Value("KdlSolver")},
        {"kinematics", yarp::os::Value(kinematicsLeftLeg)},
        {"ikPos", yarp::os::Value("st")},
        {"invKinStrategy", yarp::os::Value("humanoidGait")},
        {"quiet", yarp::os::Value::getNullValue()}
    };

    leftLegSolverOptions.put("mins", yarp::os::Value::makeList(bMinLeftLeg.toString().c_str()));
    leftLegSolverOptions.put("maxs", yarp::os::Value::makeList(bMaxLeftLeg.toString().c_str()));

    yarp::dev::PolyDriver leftLegSolverDevice(leftLegSolverOptions);

    if (!leftLegSolverDevice.isValid())
    {
        yError() << "Solver device (left leg) not available";
        return 1;
    }

    rl::ICartesianSolver * iCartesianSolverLeftLeg;

    if (!leftLegSolverDevice.view(iCartesianSolverLeftLeg))
    {
        yError() << "Cannot view solver interface (left leg)";
        return 1;
    }

    // Create solver device (right leg).

    yarp::os::Property rightLegSolverOptions {
        {"device", yarp::os::Value("KdlSolver")},
        {"kinematics", yarp::os::Value(kinematicsRightLeg)},
        {"ikPos", yarp::os::Value("st")},
        {"invKinStrategy", yarp::os::Value("humanoidGait")},
        {"quiet", yarp::os::Value::getNullValue()}
    };

    rightLegSolverOptions.put("mins", yarp::os::Value::makeList(bMinRightLeg.toString().c_str()));
    rightLegSolverOptions.put("maxs", yarp::os::Value::makeList(bMaxRightLeg.toString().c_str()));

    yarp::dev::PolyDriver rightLegSolverDevice(rightLegSolverOptions);

    if (!rightLegSolverDevice.isValid())
    {
        yError() << "Solver device (right leg) not available";
        return 1;
    }

    rl::ICartesianSolver * iCartesianSolverRightLeg;

    if (!rightLegSolverDevice.view(iCartesianSolverRightLeg))
    {
        yError() << "Cannot view solver interface (right leg)";
        return 1;
    }

    // Initialize specs and components.

    std::vector<double> xLeftInitial;

    if (!iCartesianSolverLeftLeg->fwdKin(qLeftInitial, xLeftInitial))
    {
        yError() << "Cannot stat left leg";
        return 1;
    }

    std::vector<double> xRightInitial;

    if (!iCartesianSolverRightLeg->fwdKin(qRightInitial, xRightInitial))
    {
        yError() << "Cannot stat right leg";
        return 1;
    }

    xLeftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    xRightInitial[2] += KDL::epsilon; // initial pose is hard to attain

    KDL::Frame H_leftInitial = rl::KdlVectorConverter::vectorToFrame(xLeftInitial);

    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.margin = footMargin;
    footSpec.stable = footStable;
    footSpec.lift = footLift;

    GaitSpec gaitSpec;
    gaitSpec.sep = gaitSep;
    gaitSpec.hop = gaitHop;

    LimitChecker limitChecker(footSpec, tolerance,
                              iCartesianSolverLeftLeg, iCartesianSolverRightLeg,
                              qLeftInitial, qRightInitial,
                              xLeftInitial, xRightInitial);

    limitChecker.estimateParameters(gaitSpec);
    limitChecker.setReference(gaitSpec);

    StepGenerator stepGenerator(footSpec, H_leftInitial);
    TrajectoryGenerator trajectoryGenerator(footSpec, distance, trajVel, trajAcc);
    TargetBuilder targetBuilder(iCartesianSolverLeftLeg, iCartesianSolverRightLeg, qLeftInitial, qRightInitial);

    TargetBuilder::Targets pointsLeft, pointsRight;

    bool hasSolution = false;
    double maxDuration;

    do
    {
        yInfo("step: %f, squat: %f, hop: %f, sep: %f", gaitSpec.step, gaitSpec.squat, gaitSpec.hop, gaitSpec.sep);

        // Generate steps.

        stepGenerator.configure(gaitSpec);

        std::vector<KDL::Frame> steps, com;
        stepGenerator.generate(distance, steps, com);

        {
            auto && log = yInfo();
            log << steps.size() << "steps ([x, y]):";

            for (int i = 0; i < steps.size(); i++)
            {
                const KDL::Vector & p = steps[i].p;
                std::ostringstream oss;
                oss << "[" << p.x() << " " << p.y() << "]";
                log << oss.str();
            }
        }

        {
            auto && log = yInfo();
            log << com.size() << "CoM points ([x, y, z]):";

            for (int i = 0; i < com.size(); i++)
            {
                const KDL::Vector & p = com[i].p;
                std::ostringstream oss;
                oss << "[" << p.x() << " " << p.y() << " " << p.z() << "]";
                log << oss.str();
            }
        }

        // Generate trajectories.

        trajectoryGenerator.configure(steps, com);

        KDL::Trajectory_Composite comTraj, leftTraj, rightTraj;

        try
        {
            trajectoryGenerator.generate(comTraj, leftTraj, rightTraj);
        }
        catch (const KDL::Error_MotionPlanning & e)
        {
            yWarning() << "Error:" << e.Description();
            continue;
        }

        yInfo() << "CoM:" << comTraj.Duration() << "[s], left:" << leftTraj.Duration() << "[s], right:" << rightTraj.Duration() << "[s]";

        double minDuration = std::min(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
        maxDuration = std::max(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());

        if (maxDuration - minDuration > 1.0)
        {
            yWarning() << "Duration difference exceeds 1.0 seconds:" << maxDuration - minDuration;
            continue;
        }

        // Build target points.

        targetBuilder.configure(&comTraj, &leftTraj, &rightTraj);
        targetBuilder.build(period, pointsLeft, pointsRight);

        if (!targetBuilder.validate(pointsLeft, pointsRight))
        {
            yWarning() << "IK failed";
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
        yError() << "No valid solution found";
        return 1;
    }

    // Configure worker.

    if (!dryRun)
    {
        yarp::os::TimerSettings timerSettings(period, maxDuration / period, maxDuration);

        std::vector<double> qLeft = qLeftInitial;
        std::vector<double> qRight = qRightInitial;

        yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
        {
            iCartesianSolverLeftLeg->invKin(pointsLeft[event.runCount], qLeft, qLeft);
            iCartesianSolverRightLeg->invKin(pointsRight[event.runCount], qRight, qRight);

            posdLeftLeg->setPositions(qLeft.data());
            posdRightLeg->setPositions(qRight.data());

            return true;
        };

        yarp::os::Timer timer(timerSettings, callback, true);

        // Execute trajectory.

        if (timer.start())
        {
            yarp::os::SystemClock::delaySystem(maxDuration);
            timer.stop();
        }
    }

    return 0;
}
