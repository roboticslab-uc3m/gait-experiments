// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup squatAndBalance squatAndBalance
 * @brief Perform squat and balancing on a humanoid robot.
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

#include <memory>
#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>

constexpr auto TRAJ_DURATION = 10.0;
constexpr auto TRAJ_MAX_VEL = 0.05;
constexpr auto TRAJ_MAX_ACC = 0.05;
constexpr auto TRAJ_PERIOD_MS = 50.0;

namespace rl = roboticslab;

namespace
{
    struct Worker
    {
        bool doWork(const yarp::os::YarpTimerEvent & timerEvent)
        {
            auto H = trajectory->Pos(timerEvent.runCount * period);
            auto position = rl::KdlVectorConverter::frameToVector(H);
            iCartesianControl->pose(position);
            return true;
        }

        rl::ICartesianControl * iCartesianControl;
        KDL::Trajectory * trajectory;
        double period;
    };
}

int main(int argc, char * argv[])
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

    double x = rf.check("x", yarp::os::Value(0.0), "x offset (COG)").asFloat64();
    double y = rf.check("y", yarp::os::Value(0.0), "y offset (COG)").asFloat64();
    double z = rf.check("z", yarp::os::Value(0.0), "z offset (COG)").asFloat64();

    double duration = rf.check("duration", yarp::os::Value(TRAJ_DURATION), "trajectory duration [s]").asFloat64();
    double maxVel = rf.check("maxVel", yarp::os::Value(TRAJ_MAX_VEL), "trajectory max velocity [m/s]").asFloat64();
    double maxAcc = rf.check("maxAcc", yarp::os::Value(TRAJ_MAX_VEL), "trajectory max acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(TRAJ_PERIOD_MS * 0.001), "trajectory period [s]").asFloat64();

    // Create devices.

    yarp::os::Property leftLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"remote", yarp::os::Value(robotPrefix + "/leftLeg/CartesianControl")},
        {"local", yarp::os::Value("/squatAndBalance/leftLeg")}
    };

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

    if (!iCartesianControlLeftLeg->setParameter(rl::ICartesianControl::Config::STREAMING_CMD, static_cast<yarp::conf::vocab32_t>(rl::ICartesianControl::Streaming::POSE)))
    {
        yError() << "Cannot preset streaming command (left leg)";
        return 1;
    }

    yarp::os::Property rightLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"remote", yarp::os::Value(robotPrefix + "/rightLeg/CartesianControl")},
        {"local", yarp::os::Value("/squatAndBalance/rightLeg")}
    };

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

    if (!iCartesianControlRightLeg->setParameter(rl::ICartesianControl::Config::STREAMING_CMD, static_cast<yarp::conf::vocab32_t>(rl::ICartesianControl::Streaming::POSE)))
    {
        yError() << "Cannot preset streaming command (right leg)";
        return 1;
    }

    // Configure trajectories.

    rl::ICartesianControl::ControllerState stateLeftLeg;

    if (!iCartesianControlLeftLeg->getState(stateLeftLeg))
    {
        yError() << "getState() failed (left leg)";
        return 1;
    }

    std::vector<double> xd_leftLeg(stateLeftLeg.x);
    xd_leftLeg[0] += x;
    xd_leftLeg[1] -= y;
    xd_leftLeg[2] += z;


    yInfo() << "Current (left):" << stateLeftLeg.x[0] << stateLeftLeg.x[1] << stateLeftLeg.x[2];
    yInfo() << "Desired (left):" << xd_leftLeg[0] << xd_leftLeg[1] << xd_leftLeg[2];

    std::unique_ptr<KDL::Trajectory> trajectoryLeftLeg;

    {
        auto H_base_start = rl::KdlVectorConverter::vectorToFrame(stateLeftLeg.x);
        auto H_base_end = rl::KdlVectorConverter::vectorToFrame(xd_leftLeg);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, H_base_end, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);

        trajectoryLeftLeg = std::make_unique<KDL::Trajectory_Segment>(path, profile, duration);
    }

    rl::ICartesianControl::ControllerState stateRightLeg;

    if (!iCartesianControlRightLeg->getState(stateRightLeg))
    {
        yError() << "stat() failed (right leg)";
        return 1;
    }

    std::vector<double> xd_rightLeg(stateRightLeg.x);
    xd_rightLeg[0] += x;
    xd_rightLeg[1] -= y;
    xd_rightLeg[2] += z;


    yInfo() << "Current (right):" << stateRightLeg.x[0] << stateRightLeg.x[1] << stateRightLeg.x[2];
    yInfo() << "Desired (right):" << xd_rightLeg[0] << xd_rightLeg[1] << xd_rightLeg[2];

    std::unique_ptr<KDL::Trajectory> trajectoryRightLeg;

    {
        auto H_base_start = rl::KdlVectorConverter::vectorToFrame(stateRightLeg.x);
        auto H_base_end = rl::KdlVectorConverter::vectorToFrame(xd_rightLeg);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, H_base_end, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);

        trajectoryRightLeg = std::make_unique<KDL::Trajectory_Segment>(path, profile, duration);
    }

    // Configure workers.

    Worker leftLegWorker, rightLegWorker;

    leftLegWorker.iCartesianControl = iCartesianControlLeftLeg;
    leftLegWorker.trajectory = trajectoryLeftLeg.get();

    rightLegWorker.iCartesianControl = iCartesianControlRightLeg;
    rightLegWorker.trajectory = trajectoryRightLeg.get();

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
