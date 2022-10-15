// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "OneFootStand.hpp"

/**
 * @ingroup gait-experiments-programs
 *
 * @defgroup oneFootStand oneFootStand
 *
 * @brief Creates an instance of roboticslab::OneFootStand.
 */

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("oneFootStand");
    rf.setDefaultConfigFile("oneFootStand.ini");
    rf.configure(argc, argv);

    roboticslab::OneFootStand mod;

    yInfo() << "oneFootStand checking for YARP network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "oneFootStand found no YARP network (try running \"yarpserver &\")";
        return 1;
    }

    return mod.runModule(rf);
}
