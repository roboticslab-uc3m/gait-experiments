// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup zmpViewer zmpViewer
 * @brief Creates an instance of roboticslab::ZmpViewer.
 *
 * This app connects to a remote /zmp:o port that streams ZMP coordinates
 * relative to a TCP frame. Then, it is drawn and published as a YARP image
 * representing the robot's foot sole.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "ZmpViewer.hpp"

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("zmpViewer");
    rf.setDefaultConfigFile("zmpViewer.ini");
    rf.configure(argc, argv);

    yarp::os::Network yarp;
    yInfo() << "Checking for yarp network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "YARP network not found";
        return 1;
    }

    roboticslab::ZmpViewer mod;
    return mod.runModule(rf);
}
