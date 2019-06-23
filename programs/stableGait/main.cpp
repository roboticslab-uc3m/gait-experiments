// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * \defgroup stableGait stableGait
 *
 * <b>Building</b>
 *
\verbatim
mkdir build; cd build; cmake ..
make -j$(nproc) && sudo make install
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
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/leftLeg-Kinematics.ini--local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/rightLeg-Kinematics.ini--local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] stableGait # WIP
\endverbatim
 */

#include <string>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>
#include <ColorDebug.h>

#include "FootSpec.hpp"
#include "LimitChecker.hpp"
#include "StepGenerator.hpp"

#define DEFAULT_TRAJ_VEL 0.1 // [m/s]
#define DEFAULT_TRAJ_ACC 0.2 // [m/s^2]
#define DEFAULT_CMD_PERIOD 0.05 // [s]
#define DEFAULT_SQUAT_DIFF 0.01 // [m]
#define DEFAULT_FOOT_LENGTH 0.245 // [m]
#define DEFAULT_FOOT_WIDTH 0.14 // [m]
#define DEFAULT_FOOT_MARGIN 0.01 // [m]
#define DEFAULT_FOOT_SEP 0.06 // [m]

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

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();

    double distance = rf.check("distance", yarp::os::Value(1.0), "distance to travel [m]").asFloat64();
    double trajVel = rf.check("vel", yarp::os::Value(DEFAULT_TRAJ_VEL), "velocity [m/s]").asFloat64();
    double trajAcc = rf.check("acc", yarp::os::Value(DEFAULT_TRAJ_ACC), "acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(DEFAULT_CMD_PERIOD), "command period [s]").asFloat64();
    double squatDiff = rf.check("squat", yarp::os::Value(DEFAULT_SQUAT_DIFF), "squat distance [m]").asFloat64();
    double footLength = rf.check("length", yarp::os::Value(DEFAULT_FOOT_LENGTH), "foot length [m]").asFloat64();
    double footWidth = rf.check("width", yarp::os::Value(DEFAULT_FOOT_WIDTH), "foot width [m]").asFloat64();
    double footMargin = rf.check("margin", yarp::os::Value(DEFAULT_FOOT_MARGIN), "foot stability margin [m]").asFloat64();
    double footSep = rf.check("sep", yarp::os::Value(DEFAULT_FOOT_SEP), "foot separation [m]").asFloat64();

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

    if (squatDiff < 0.0)
    {
        CD_ERROR("Illegal argument: '--squat' must be greater than or equal to '0' (was '%f').\n", squatDiff);
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

    if (footSep <= 0.0)
    {
        CD_ERROR("Illegal argument: '--sep' must be greater than '0' (was '%f').\n", footSep);
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

    // Analyze motion limits.

    std::vector<double> x_leftInitial;

    if (!iCartesianControlLeftLeg->stat(x_leftInitial))
    {
        CD_ERROR("Cannot stat left leg.\n");
        return 1;
    }

    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.margin = footMargin;
    footSpec.sep = footSep;

    LimitChecker limitChecker(iCartesianControlLeftLeg, iCartesianControlRightLeg);
    limitChecker.configure(footSpec);

    double squat, step;
    limitChecker.estimateParameters(&squat, &step);

    CD_INFO("squat: %f, step: %f\n", squat, step);

    // Generate steps.

    StepGenerator stepGenerator(footSpec);
    stepGenerator.configure(step, x_leftInitial[1]);

    std::vector<KDL::Frame> stepsLeft, stepsRight;
    stepGenerator.generate(distance, stepsLeft, stepsRight);

    CD_INFO("Left leg: %d steps:", stepsLeft.size());

    for (int i = 0; i < stepsLeft.size(); i++)
    {
        CD_INFO_NO_HEADER(" %f", stepsLeft[i].p.x());
    }

    CD_INFO_NO_HEADER("\n");

    CD_INFO("Right leg: %d steps:", stepsRight.size());

    for (int i = 0; i < stepsRight.size(); i++)
    {
        CD_INFO_NO_HEADER(" %f", stepsRight[i].p.x());
    }

    CD_INFO_NO_HEADER("\n");

    leftLegDevice.close();
    rightLegDevice.close();

    return 0;
}
