// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * \defgroup squatLooped squatLooped
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
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --kinematics leftLeg.ini --local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --kinematics rightLeg.ini --local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] squatLooped --z 0.045 --squats 1
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

#define TRAJ_DURATION 10.0
#define TRAJ_MAX_VEL 0.05
#define TRAJ_MAX_ACC 0.2
#define TRAJ_PERIOD_MS 50.0

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
        CD_ERROR("Illegal argument: '--z' must be greater than '0' (was '%f').\n", z);
        return 1;
    }

    if (squats < 1)
    {
        CD_ERROR("Illegal argument: '--squats' must be greater than '0' (was '%d').\n", squats);
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

    // Configure trajectories (common parameters).

    KDL::RotationalInterpolation_SingleAxis orient;
    const double eqradius = 1.0;

    KDL::VelocityProfile_Trap profile(maxVel, maxAcc);

    // Configure trajectories (left leg).

    std::vector<double> x_leftLeg;

    if (!iCartesianControlLeftLeg->stat(x_leftLeg))
    {
        CD_ERROR("stat() failed (left leg).\n");
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
        CD_ERROR("stat() failed (right leg).\n");
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

        iCartesianControlLeftLeg->movi(rl::KdlVectorConverter::frameToVector(H_left));
        iCartesianControlRightLeg->movi(rl::KdlVectorConverter::frameToVector(H_right));

        return true;
    };

    yarp::os::Timer timer(timerSettings, callback, true);

    // Perform actions.

    CD_INFO("Performing %d squat(s) with z=%f in %f seconds...\n", squats, z, totalDuration);

    if (timer.start())
    {
        yarp::os::Time::delay(totalDuration);
        timer.stop();
    }

    leftLegDevice.close();
    rightLegDevice.close();

    return 0;
}
