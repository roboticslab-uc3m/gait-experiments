// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneFootStand.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <kdl/utilities/utility.h> // KDL::deg2rad

#include "KdlVectorConverter.hpp"

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(OFS, "rl.OneFootStand")
}

constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";
constexpr auto DEFAULT_PERIOD = 0.02;
constexpr auto DEFAULT_LIN_GAIN = 0.01;
constexpr auto DEFAULT_ROT_GAIN = 0.02;
constexpr auto DEFAULT_FORCE_DEADBAND = 1.0;
constexpr auto DEFAULT_TORQUE_DEADBAND = 1.0;

bool OneFootStand::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(OFS) << "Config:" << rf.toString();

    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "period [s]").asFloat64();

    dryRun = rf.check("dryRun", "process sensor loops, but don't send motion command");
    linGain = rf.check("linGain", yarp::os::Value(DEFAULT_LIN_GAIN), "linear gain").asFloat64();
    rotGain = rf.check("rotGain", yarp::os::Value(DEFAULT_ROT_GAIN), "rotational gain").asFloat64();
    forceDeadband = rf.check("forceDeadband", yarp::os::Value(DEFAULT_FORCE_DEADBAND), "force deadband [N]").asFloat64();
    torqueDeadband = rf.check("torqueDeadband", yarp::os::Value(DEFAULT_TORQUE_DEADBAND), "torque deadband [Nm]").asFloat64();

    yCInfo(OFS) << "Using linear gain:" << linGain;
    yCInfo(OFS) << "Using rotational gain:" << rotGain;
    yCInfo(OFS) << "Using force deadband [N]:" << forceDeadband;
    yCInfo(OFS) << "Using torque deadband [Nm]:" << torqueDeadband;

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

    if (!readSensor(initialOffset))
    {
        yCError(OFS) << "Failed to read sensor";
        return false;
    }

    // ----- cartesian device -----

    if (!dryRun)
    {
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

            if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_TWIST))
            {
                yCWarning(OFS) << "Unable to preset streaming command";
                return false;
            }

            if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
            {
                yCWarning(OFS) << "Unable to set TCP frame";
                return false;
            }
        }
    }
    else if (dryRun)
    {
        yCInfo(OFS) << "Dry run mode enabled, robot will perform no motion";
    }

    yarp::os::PeriodicThread::setPeriod(period);

    return yarp::os::PeriodicThread::start();
}

bool OneFootStand::readSensor(KDL::Wrench & wrench) const
{
    yarp::sig::Vector outSensor;

    if (double timestamp; !sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, outSensor, timestamp))
    {
        yCWarning(OFS) << "Failed to retrieve current sensor measurements";
        return false;
    }

    KDL::Wrench currentWrench_sensor;
    currentWrench_sensor.force = KDL::Vector(outSensor[0], outSensor[1], outSensor[2]);
    currentWrench_sensor.torque = KDL::Vector(outSensor[3], outSensor[4], outSensor[5]);

    wrench = R_N_sensor * currentWrench_sensor;
    return true;
}

void OneFootStand::run()
{
    KDL::Wrench wrench;

    if (!readSensor(wrench))
    {
        yarp::os::PeriodicThread::askToStop();
        return;
    }

    wrench -= initialOffset;

    auto forceNorm = wrench.force.Norm();
    auto torqueNorm = wrench.torque.Norm();

    if (forceNorm <= forceDeadband && torqueNorm <= torqueDeadband)
    {
        if (!dryRun) iCartesianControl->twist(std::vector(6, 0.0));
        return;
    }

    if (forceNorm > forceDeadband)
    {
        auto forceThreshold = (wrench.force / forceNorm) * forceDeadband;
        wrench.force = (wrench.force - forceThreshold) * linGain;
    }
    else
    {
        wrench.force = KDL::Vector::Zero();
    }

    if (torqueNorm > torqueDeadband)
    {
        auto torqueThreshold = (wrench.torque / torqueNorm) * torqueDeadband;
        wrench.torque = (wrench.torque - torqueThreshold) * rotGain;
    }
    else
    {
        wrench.torque = KDL::Vector::Zero();
    }

    auto v = KdlVectorConverter::wrenchToVector(wrench);
    yCDebug(OFS) << v;
    if (!dryRun) iCartesianControl->twist(v);
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
    cartesianDevice.close();
    sensorDevice.close();
    return true;
}
