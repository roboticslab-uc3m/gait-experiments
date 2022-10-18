// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneFootStand.hpp"

#include <cmath> // std::abs, std::copysign
#include <algorithm> // std::copy, std::max, std::min

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <kdl/utilities/utility.h> // KDL::deg2rad

#include <KdlVectorConverter.hpp> // TODO: unused

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(OFS, "rl.OneFootStand")
}

constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";
constexpr auto DEFAULT_IK_STEP = 0.001; // [m]
constexpr auto DEFAULT_MAX_SPEED = 0.01; // [m/s]
constexpr auto DEFAULT_MAX_ACCELERATION = 0.1; // [m/s^2]

bool OneFootStand::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(OFS) << "Config:" << rf.toString();

    period = rf.check("periodMs", yarp::os::Value(0), "period [ms]").asInt32() * 0.001;

    if (period <= 0)
    {
        yCError(OFS) << "Missing or invalid period parameter:" << static_cast<int>(period * 1000) << "(ms)";
        return false;
    }

    ikStep = rf.check("ikStep", yarp::os::Value(DEFAULT_IK_STEP), "IK step [m]").asFloat64();

    if (ikStep <= 0.0)
    {
        yCError(OFS) << "Invalid IK step parameter:" << ikStep << "(m)";
        return false;
    }

    maxSpeed = rf.check("maxSpeed", yarp::os::Value(DEFAULT_MAX_SPEED), "max speed [m/s]").asFloat64();

    if (maxSpeed <= 0.0)
    {
        yCError(OFS) << "Invalid max speed parameter:" << maxSpeed << "(m/s)";
        return false;
    }

    maxAcceleration = rf.check("maxAcceleration", yarp::os::Value(DEFAULT_MAX_ACCELERATION), "max acceleration [m/s^2]").asFloat64();

    if (maxAcceleration <= 0.0)
    {
        yCError(OFS) << "Invalid max acceleration parameter:" << maxAcceleration << "(m/s^2)";
        return false;
    }

    dryRun = rf.check("dryRun", "process sensor loops, but don't send motion command");

    if (dryRun)
    {
        yCInfo(OFS) << "Dry run mode enabled, robot will perform no motion";
    }

    auto sensorFrameRPY = rf.check("sensorFrameRPY", yarp::os::Value::getNullValue(), "sensor frame RPY rotation regarding TCP frame [deg]");

    if (!sensorFrameRPY.isNull())
    {
        if (!sensorFrameRPY.isList() || sensorFrameRPY.asList()->size() != 3)
        {
            yCError(OFS) << "sensorFrameRPY must be a list of 3 doubles";
            return false;
        }

        yCInfo(OFS) << "Sensor frame RPY [deg]:" << sensorFrameRPY.toString();

        auto roll = sensorFrameRPY.asList()->get(0).asFloat64() * KDL::deg2rad;
        auto pitch = sensorFrameRPY.asList()->get(1).asFloat64() * KDL::deg2rad;
        auto yaw = sensorFrameRPY.asList()->get(2).asFloat64() * KDL::deg2rad;

        // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
        R_N_sensor = KDL::Rotation::RPY(roll, pitch, yaw);
    }
    else
    {
        yCInfo(OFS) << "Using no sensor frame";
        R_N_sensor = KDL::Rotation::Identity();
    }

    soleNormal = R_N_sensor * KDL::Vector(0.0, 0.0, 1.0); // assume sole normal is the sensor's Z axis

    auto localPrefix = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port prefix").asString();

    // ----- sensor device -----

    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
    {
        yCError(OFS) << "Missing parameter: sensorName";
        return false;
    }

    auto sensorName = rf.find("sensorName").asString();

    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(OFS) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();

    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(localPrefix + sensorRemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(OFS) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(OFS) << "Failed to view sensor interface";
        return false;
    }

    sensorIndex = -1;

    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
    {
        std::string temp;

        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
        {
            sensorIndex = i;
            break;
        }
    }

    if (sensorIndex == -1)
    {
        yCError(OFS) << "Failed to find sensor with name" << sensorName;
        return false;
    }

    int retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK && retry++ < 10)
    {
        yCDebug(OFS) << "Waiting for sensor to be ready... retry" << retry;
        yarp::os::SystemClock::delaySystem(0.1);
    }

    if (retry == 10)
    {
        yCError(OFS) << "Failed to get first read, max number of retries exceeded";
        return false;
    }

    // ----- cartesian device -----

    if (!rf.check("cartesianRemote", "remote cartesian port to connect to"))
    {
        yCError(OFS) << "Missing parameter: cartesianRemote";
        return false;
    }

    auto cartesianRemote = rf.find("cartesianRemote").asString();

    yarp::os::Property cartesianOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value(cartesianRemote)},
        {"cartesianLocal", yarp::os::Value(localPrefix + cartesianRemote)}
    };

    if (!cartesianDevice.open(cartesianOptions))
    {
        yCError(OFS) << "Failed to open cartesian device";
        return false;
    }

    if (!cartesianDevice.view(iCartesianControl))
    {
        yCError(OFS) << "Failed to view cartesian control interface";
        return false;
    }

    if (!dryRun)
    {
        std::map<int, double> params;

        if (!iCartesianControl->getParameters(params))
        {
            yCError(OFS) << "Unable to retrieve cartesian configuration parameters";
            return false;
        }

        bool usingStreamingPreset = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

        if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
        {
            yCWarning(OFS) << "Unable to preset streaming command";
            return false;
        }
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
    {
        yCWarning(OFS) << "Unable to set TCP frame";
        return false;
    }

    // ----- ZMP publisher port -----

    if (!zmpPort.open(localPrefix + "/zmp:o"))
    {
        yCError(OFS) << "Failed to open ZMP publisher port" << zmpPort.getName();
        return false;
    }

    zmpPort.setWriteOnly();

    return yarp::os::PeriodicThread::setPeriod(period) && yarp::os::PeriodicThread::start();
}

bool OneFootStand::readSensor(KDL::Wrench & wrench_N) const
{
    yarp::sig::Vector outSensor;

    if (double timestamp; !sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, outSensor, timestamp))
    {
        yCWarning(OFS) << "Failed to retrieve current sensor measurements";
        return false;
    }

    KDL::Wrench currentWrench_sensor (
        KDL::Vector(outSensor[0], outSensor[1], outSensor[2]), // force
        KDL::Vector(outSensor[3], outSensor[4], outSensor[5]) // torque
    );

    wrench_N = R_N_sensor * currentWrench_sensor;
    return true;
}

bool OneFootStand::selectZmp(const KDL::Vector & axis, KDL::Vector & zmp) const
{
    std::vector<double> x(6, 0.0);
    std::vector<double> q;

    // ignore orientation (last 3 elements)
    std::copy(zmp.data, zmp.data + 3, x.begin());

    const auto initialIkSucceded = iCartesianControl->inv(x, q);
    const auto step = initialIkSucceded ? -ikStep : ikStep;

    static const auto maxIterations = 20;
    auto i = 0;

    do
    {
        zmp += ikStep * axis; // axis must be normalized
        std::copy(zmp.data, zmp.data + 3, x.begin());
        i++;
    }
    while (i < maxIterations && !(initialIkSucceded ^ iCartesianControl->inv(x, q)));

    if (initialIkSucceded)
    {
        zmp -= ikStep * axis; // undo last failed attempt
    }
    else if (i >= maxIterations)
    {
        return false; // we never found a valid configuration in `maxIterations` attempts
    }

    return true;
}

void OneFootStand::publishProjection(const KDL::Vector & zmp)
{
    KDL::Vector projection = zmp - (zmp * soleNormal) * soleNormal;

    zmpPort.prepare() = {
        yarp::os::Value(projection.x()),
        yarp::os::Value(projection.y()),
    };

    zmpPort.write();
}

std::vector<double> OneFootStand::computeStep(const KDL::Vector & p)
{
    std::vector<double> x(6, 0.0);

    KDL::Vector direction = p;
    double step = direction.Normalize();

    double acceleration = std::abs(step - previousStep) / period;
    acceleration = std::min(acceleration, maxAcceleration);
    acceleration = std::copysign(acceleration, step - previousStep);

    step = std::min(step, previousStep + acceleration * period);
    step = std::min(step, maxSpeed);
    step = std::max(step, 0.0); // just in case

    KDL::Vector pp = direction * step;
    std::copy(pp.data, pp.data + 3, x.begin());

    previousStep = step;
    return x;
}

void OneFootStand::run()
{
    KDL::Wrench wrench_N; // expressed in TCP frame

    if (!readSensor(wrench_N))
    {
        yarp::os::PeriodicThread::askToStop();
        return;
    }

    auto forceNorm = wrench_N.force.Norm();

    if (KDL::Equal(forceNorm, 0.0))
    {
        yCWarning(OFS) << "Zero force detected, skipping this iteration";
        return;
    }

    // shortest distance vector from TCP to ZMP axis expressed in TCP frame
    KDL::Vector p_N_zmp = (wrench_N.force * wrench_N.torque) / (forceNorm * forceNorm);

    if (!selectZmp(wrench_N.force / forceNorm, p_N_zmp))
    {
        yCWarning(OFS) << "Unable to find a valid configuration for the ZMP in this iteration";
        return;
    }

    if (zmpPort.getOutputCount() > 0)
    {
        publishProjection(p_N_zmp);
    }

    auto xd = computeStep(p_N_zmp);
    yCDebug(OFS) << xd;
    if (!dryRun) iCartesianControl->movi(xd);
}

bool OneFootStand::updateModule()
{
    return yarp::os::PeriodicThread::isRunning();
}

bool OneFootStand::interruptModule()
{
    yarp::os::PeriodicThread::stop();
    return dryRun || iCartesianControl->stopControl();
}

double OneFootStand::getPeriod()
{
    return 1.0; // [s]
}

bool OneFootStand::close()
{
    zmpPort.interrupt();
    zmpPort.close();
    cartesianDevice.close();
    sensorDevice.close();
    return true;
}
