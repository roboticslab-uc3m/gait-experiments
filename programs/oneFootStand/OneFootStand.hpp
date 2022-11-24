// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_FOOT_STAND_HPP__
#define __ONE_FOOT_STAND_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include <ICartesianSolver.h>

namespace roboticslab
{

/**
 * @ingroup oneFootStand
 *
 * @brief Uses ZMP to maintain stability while standing on one foot.
 */
class OneFootStand : public yarp::os::RFModule,
                     public yarp::os::PeriodicThread
{
public:
    OneFootStand()
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute)
    {}

    ~OneFootStand() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;

protected:
    void run() override;

private:
    bool readSensor(KDL::Wrench & wrench_N);
    void publishProjection(const KDL::Vector & p_N_zmp);
    double computeStepDistance(const KDL::Vector & p_N_zmp);
    bool computeStepDirection(const KDL::Vector & p_N_zmp, double distance, KDL::Vector & dir);

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IPositionDirect * posd;

    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver * solver;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Rotation R_N_sole;

    bool dryRun;

    double period;
    double ikStep;
    double maxSpeed;
    double maxAcceleration;

    double previousStep {0.0};
    std::vector<double> previousJointPose;

    yarp::os::BufferedPort<yarp::os::Bottle> zmpPort;
};

} // namespace roboticslab

#endif // __ONE_FOOT_STAND_HPP__
