// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * \defgroup squatAndBalance squatAndBalance
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
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --kinematics teo-leftLeg.ini --local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --kinematics teo-rightLeg.ini --local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] ./squatAndBalance --z 0.045 # move robot's CoM 4.5 cm down
[on terminal 5] ./squatAndBalance --y 0.1 # move CoM 10 cm to its left
[on terminal 5] ./squatAndBalance --y -0.2 # move CoM 20 cm to its right
[on terminal 5] ./squatAndBalance --y 0.1 # move CoM 10 cm back to its left, now centered
[on terminal 5] ./squatAndBalance --z -0.045 # return to initial pose
\endverbatim
 */

#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <ICartesianControl.h>
#include <KdlTrajectory.hpp>

#define TRAJ_DURATION 10.0
#define TRAJ_MAX_VEL 0.05
#define TRAJ_PERIOD_MS 50.0

namespace rl = roboticslab;

namespace
{
    struct Worker
    {
        bool doWork(const yarp::os::YarpTimerEvent & timerEvent)
        {
            std::vector<double> position;
            iCartesianTrajectory->getPosition(timerEvent.runCount * period, position);
            iCartesianControl->movi(position);
            return true;
        }

        rl::ICartesianControl * iCartesianControl;
        rl::ICartesianTrajectory * iCartesianTrajectory;
        double period;
    };
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();

    double y = rf.check("y", yarp::os::Value(0.0), "y offset (COG)").asFloat64();
    double z = rf.check("z", yarp::os::Value(0.0), "z offset (COG)").asFloat64();

    double duration = rf.check("duration", yarp::os::Value(TRAJ_DURATION), "trajectory duration [s]").asFloat64();
    double maxVel = rf.check("maxvel", yarp::os::Value(TRAJ_MAX_VEL), "trajectory max velocity [m/s]").asFloat64();
    double period = rf.check("period", yarp::os::Value(TRAJ_PERIOD_MS * 0.001), "trajectory period [s]").asFloat64();

    // Create devices.

    yarp::os::Property leftLegDeviceOptions;
    leftLegDeviceOptions.put("device", "CartesianControlClient");
    leftLegDeviceOptions.put("cartesianRemote", robotPrefix + "/leftLeg/CartesianControl");
    leftLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/leftLeg");

    yarp::dev::PolyDriver leftLegDevice(leftLegDeviceOptions);

    if (!leftLegDevice.isValid())
    {
        yError() << "Cartesian device (left leg) not available";
        return 1;
    }

    rl::ICartesianControl * iCartesianControlLeftLeg;

    if (!leftLegDevice.view(iCartesianControlLeftLeg))
    {
        yError() << "Cannot view iCartesianControlLeftLeg";
        return 1;
    }

    if (!iCartesianControlLeftLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yError() << "Cannot preset streaming command (left leg)";
        return 1;
    }

    yarp::os::Property rightLegDeviceOptions;
    rightLegDeviceOptions.put("device", "CartesianControlClient");
    rightLegDeviceOptions.put("cartesianRemote", robotPrefix + "/rightLeg/CartesianControl");
    rightLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/rightLeg");

    yarp::dev::PolyDriver rightLegDevice(rightLegDeviceOptions);

    if (!rightLegDevice.isValid())
    {
        yError() << "Cartesian device (right leg) not available";
        return 1;
    }

    rl::ICartesianControl * iCartesianControlRightLeg;

    if (!rightLegDevice.view(iCartesianControlRightLeg))
    {
        yError() << "Cannot view iCartesianControlRightLeg";
        return 1;
    }

    if (!iCartesianControlRightLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yError() << "Cannot preset streaming command (right leg)";
        return 1;
    }

    // Configure trajectories.

    std::vector<double> x_leftLeg;

    if (!iCartesianControlLeftLeg->stat(x_leftLeg))
    {
        yError() << "stat() failed (left leg)";
        return 1;
    }

    std::vector<double> xd_leftLeg(x_leftLeg);
    xd_leftLeg[1] -= y;
    xd_leftLeg[2] += z;

    yInfo() << "Current (left):" <<  x_leftLeg[0] << x_leftLeg[1] << x_leftLeg[2];
    yInfo() << "Desired (left):" << xd_leftLeg[0] << xd_leftLeg[1] << xd_leftLeg[2];

    rl::KdlTrajectory trajectoryLeftLeg;

    trajectoryLeftLeg.setDuration(duration);
    trajectoryLeftLeg.setMaxVelocity(maxVel);
    trajectoryLeftLeg.addWaypoint(x_leftLeg);
    trajectoryLeftLeg.addWaypoint(xd_leftLeg);
    trajectoryLeftLeg.configurePath(rl::ICartesianTrajectory::LINE);
    trajectoryLeftLeg.configureVelocityProfile(rl::ICartesianTrajectory::TRAPEZOIDAL);

    if (!trajectoryLeftLeg.create())
    {
        yError() << "Problem creating cartesian trajectory (left leg)";
        return 1;
    }

    std::vector<double> x_rightLeg;

    if (!iCartesianControlRightLeg->stat(x_rightLeg))
    {
        yError() << "stat() failed (right leg)";
        return 1;
    }

    std::vector<double> xd_rightLeg(x_rightLeg);
    xd_rightLeg[1] -= y;
    xd_rightLeg[2] += z;

    yInfo() << "Current (right):" << x_rightLeg[0] << x_rightLeg[1] << x_rightLeg[2];
    yInfo() << "Desired (right):" << xd_rightLeg[0] << xd_rightLeg[1] << xd_rightLeg[2];

    rl::KdlTrajectory trajectoryRightLeg;

    trajectoryRightLeg.setDuration(duration);
    trajectoryRightLeg.setMaxVelocity(maxVel);
    trajectoryRightLeg.addWaypoint(x_rightLeg);
    trajectoryRightLeg.addWaypoint(xd_rightLeg);
    trajectoryRightLeg.configurePath(rl::ICartesianTrajectory::LINE);
    trajectoryRightLeg.configureVelocityProfile(rl::ICartesianTrajectory::TRAPEZOIDAL);

    if (!trajectoryRightLeg.create())
    {
        yError() << "Problem creating cartesian trajectory (right leg)";
        return 1;
    }

    // Configure workers.

    Worker leftLegWorker, rightLegWorker;

    leftLegWorker.iCartesianControl = iCartesianControlLeftLeg;
    leftLegWorker.iCartesianTrajectory = &trajectoryLeftLeg;

    rightLegWorker.iCartesianControl = iCartesianControlRightLeg;
    rightLegWorker.iCartesianTrajectory = &trajectoryRightLeg;

    leftLegWorker.period = rightLegWorker.period = period;

    yarp::os::TimerSettings timerSettings(period, duration / period, duration);

    yarp::os::Timer leftLegTimer(timerSettings, &Worker::doWork, &leftLegWorker, true);
    yarp::os::Timer rightLegTimer(timerSettings, &Worker::doWork, &rightLegWorker, true);

    // Perform actions.

    if (leftLegTimer.start() && rightLegTimer.start())
    {
        yarp::os::Time::delay(TRAJ_DURATION);
        leftLegTimer.stop();
        rightLegTimer.stop();
    }

    leftLegDevice.close();
    rightLegDevice.close();

    return 0;
}
