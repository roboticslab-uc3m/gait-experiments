// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup squatLooped squatLooped
 * @brief Perform a series of squats on a humanoid robot.
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
[on terminal 5] squatLooped --z 0.045 --squats 1
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

#include <kdl/frames.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>

constexpr auto TRAJ_DURATION = 10.0;
constexpr auto TRAJ_MAX_VEL = 0.05;
constexpr auto TRAJ_MAX_ACC = 0.2;
constexpr auto TRAJ_PERIOD_MS = 50.0;

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

    // Parse parameters.

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();
    double z = rf.check("z", yarp::os::Value(0.0), "z offset (COG)").asFloat64();
    int squats = rf.check("squats", yarp::os::Value(1), "how many squats?").asInt32();

    double duration = rf.check("duration", yarp::os::Value(TRAJ_DURATION), "trajectory duration [s]").asFloat64();
    double maxVel = rf.check("maxVel", yarp::os::Value(TRAJ_MAX_VEL), "trajectory max velocity [m/s]").asFloat64();
    double maxAcc = rf.check("maxAcc", yarp::os::Value(TRAJ_MAX_ACC), "trajectory max acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(TRAJ_PERIOD_MS * 0.001), "trajectory period [s]").asFloat64();

    if (z <= 0.0)
    {
        yError() << "Illegal argument: '--z' must be greater than '0', was:" << z;
        return 1;
    }

    if (squats < 1)
    {
        yError() << "Illegal argument: '--squats' must be greater than '0', was:" << squats;
        return 1;
    }

    // Create devices (left leg).

    yarp::os::Property leftLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value(robotPrefix + "/leftLeg/CartesianControl")},
        {"cartesianLocal", yarp::os::Value("/squatLooped/leftLeg")}
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

    if (!iCartesianControlLeftLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_POSE))
    {
        yError() << "Cannot preset streaming command (left leg)";
        return 1;
    }

    // Create devices (right leg).

    yarp::os::Property rightLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value(robotPrefix + "/rightLeg/CartesianControl")},
        {"cartesianLocal", yarp::os::Value("/squatLooped/rightLeg")}
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

    if (!iCartesianControlRightLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_POSE))
    {
        yError() << "Cannot preset streaming command (right leg)";
        return 1;
    }

    // Configure trajectories (common parameters).

    KDL::RotationalInterpolation_SingleAxis orient;
    const double eqradius = 1.0;

    KDL::VelocityProfile_Trap profile(maxVel, maxAcc);

    // Configure trajectories (left leg).

    std::vector<double> x_leftLeg;

    if (!iCartesianControlLeftLeg->stat(x_leftLeg))
    {
        yError() << "stat() failed (left leg)";
        return 1;
    }

    KDL::Frame H_leftLeg_start = rl::KdlVectorConverter::vectorToFrame(x_leftLeg);
    KDL::Frame H_leftLeg_end = H_leftLeg_start;
    H_leftLeg_end.p.z(H_leftLeg_end.p.z() + z);

    KDL::Path_Line pathDown_leftLeg(H_leftLeg_start, H_leftLeg_end, &orient, eqradius, false);
    KDL::Path_Line pathUp_leftLeg(H_leftLeg_end, H_leftLeg_start, &orient, eqradius, false);

    KDL::Trajectory_Segment trajectorySquatDownLeftLeg(&pathDown_leftLeg, &profile, duration, false);
    KDL::Trajectory_Segment trajectorySquatUpLeftLeg(&pathUp_leftLeg, &profile, duration, false);

    // Configure trajectories (right leg).

    std::vector<double> x_rightLeg;

    if (!iCartesianControlRightLeg->stat(x_rightLeg))
    {
        yError() << "stat() failed (right leg)";
        return 1;
    }

    KDL::Frame H_rightLeg_start = rl::KdlVectorConverter::vectorToFrame(x_rightLeg);
    KDL::Frame H_rightLeg_end = H_rightLeg_start;
    H_rightLeg_end.p.z(H_rightLeg_end.p.z() + z);

    KDL::Path_Line pathDown_rightLeg(H_rightLeg_start, H_rightLeg_end, &orient, eqradius, false);
    KDL::Path_Line pathUp_rightLeg(H_rightLeg_end, H_rightLeg_start, &orient, eqradius, false);

    KDL::Trajectory_Segment trajectorySquatDownRightLeg(&pathDown_rightLeg, &profile, duration, false);
    KDL::Trajectory_Segment trajectorySquatUpRightLeg(&pathUp_rightLeg, &profile, duration, false);

    // Configure trajectories (composites).

    KDL::Trajectory_Composite trajectoryLeftLeg;
    KDL::Trajectory_Composite trajectoryRightLeg;

    for (int i = 0; i < squats; i++)
    {
        trajectoryLeftLeg.Add(trajectorySquatDownLeftLeg.Clone());
        trajectoryLeftLeg.Add(trajectorySquatUpLeftLeg.Clone());

        trajectoryRightLeg.Add(trajectorySquatDownRightLeg.Clone());
        trajectoryRightLeg.Add(trajectorySquatUpRightLeg.Clone());
    }

    // Configure workers.

    double totalDuration = duration * 2 * squats;

    yarp::os::TimerSettings timerSettings(period, totalDuration / period, totalDuration);

    yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
    {
        KDL::Frame H_left = trajectoryLeftLeg.Pos(event.runCount * period);
        KDL::Frame H_right = trajectoryRightLeg.Pos(event.runCount * period);

        iCartesianControlLeftLeg->pose(rl::KdlVectorConverter::frameToVector(H_left));
        iCartesianControlRightLeg->pose(rl::KdlVectorConverter::frameToVector(H_right));

        return true;
    };

    yarp::os::Timer timer(timerSettings, callback, true);

    // Perform actions.

    yInfo() << "Performing" << squats << "squat(s) with z =" << z << "in" << totalDuration << "seconds...";

    if (timer.start())
    {
        yarp::os::Time::delay(totalDuration);
        timer.stop();
    }

    leftLegDevice.close();
    rightLegDevice.close();

    return 0;
}
