// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_FOOT_STAND_HPP__
#define __ONE_FOOT_STAND_HPP__

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include <ICartesianControl.h>

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
    bool readSensor(KDL::Wrench & wrench) const;

    yarp::dev::PolyDriver cartesianDevice;
    roboticslab::ICartesianControl * iCartesianControl;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Wrench initialOffset;

    bool dryRun;
    double linGain;
    double rotGain;
    double forceDeadband;
    double torqueDeadband;
};

} // namespace roboticslab

#endif // __ONE_FOOT_STAND_HPP__
