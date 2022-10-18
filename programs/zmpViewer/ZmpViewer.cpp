// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ZmpViewer.hpp"

#include <algorithm> // std::min

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>

#include <yarp/sig/ImageDraw.h>

using namespace roboticslab;

constexpr auto DEFAULT_LOCAL_PORT = "/zmpViewer";

namespace
{
    YARP_LOG_COMPONENT(ZMPV, "rl.ZmpViewer")
}

bool ZmpViewer::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(ZMPV) << "Config:" << rf.toString();

    width = rf.check("width", yarp::os::Value(0.0), "sole width [m]").asFloat64() * 1000;
    height = rf.check("height", yarp::os::Value(0.0), "sole height [m]").asFloat64() * 1000;
    cx = rf.check("cx", yarp::os::Value(0.0), "x coordinate of TCP from bottom left sole corner [m]").asFloat64() * 1000;
    cy = rf.check("cy", yarp::os::Value(0.0), "y coordinate of TCP from bottom left sole corner [m]").asFloat64() * 1000;

    if (width <= 0 || height <= 0)
    {
        yCError(ZMPV) << "Invalid sole dimensions:" << width << "x" << height << "[mm]";
        return false;
    }

    if (!rf.check("remote", "remote port prefix"))
    {
        yCError(ZMPV) << "Missing remote port prefix";
        return false;
    }

    auto local = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT), "local port prefix").asString();
    auto remote = rf.find("remote").asString();

    if (!imagePort.open(local + "/sole:o"))
    {
        yCError(ZMPV) << "Unable not open local image port" << imagePort.getName();
        return false;
    }

    imagePort.setWriteOnly();

    if (!zmpPort.open(local + "/zmp:i"))
    {
        yCError(ZMPV) << "Unable to open local ZMP port" << zmpPort.getName();
        return false;
    }

    zmpPort.setReadOnly();

    yarp::os::ContactStyle style;
    style.carrier = "fast_tcp";
    style.expectReply = false;
    style.persistent = true;
    style.persistenceType = yarp::os::ContactStyle::END_WITH_TO_PORT;

    if (!yarp::os::Network::connect(remote + "/zmp:o", zmpPort.getName(), style))
    {
        yCError(ZMPV) << "Unable to connect to remote port";
        return false;
    }

    zmpPortReader.attach(zmpPort);
    zmpPortReader.useCallback(*this);

    return true;
}

bool ZmpViewer::close()
{
    imagePort.interrupt();
    imagePort.close();

    zmpPortReader.interrupt();
    zmpPortReader.disableCallback();
    zmpPort.close();

    return true;
}

bool ZmpViewer::updateModule()
{
    static const double throttle = 5.0;
    const double now = yarp::os::SystemClock::nowSystem();

    mutex.lock();
    double stamp = lastStamp;
    int x = lastZmpX;
    int y = lastZmpY;
    mutex.unlock();

    if (now - stamp > getPeriod())
    {
        yCWarningThrottle(ZMPV, throttle) << "No ZMP data received in the last" << getPeriod() << "seconds";
        drawAndPublishImage(false, x, y);
    }

    return true;
}

double ZmpViewer::getPeriod()
{
    return 0.1; // [s]
}

void ZmpViewer::onRead(yarp::os::Bottle & b)
{
    yCDebug(ZMPV) << b.toString();

    if (b.size() != 2)
    {
        yCWarning(ZMPV) << "Invalid bottle size:" << b.size();
        return;
    }

    int x = b.get(0).asFloat64() * 1000;
    int y = b.get(1).asFloat64() * 1000;

    {
        std::lock_guard lock(mutex);
        lastStamp = yarp::os::SystemClock::nowSystem();
        lastZmpX = x;
        lastZmpY = y;
    }

    drawAndPublishImage(true, x, y);
}

void ZmpViewer::drawAndPublishImage(bool isActive, int zmpX, int zmpY)
{
    static const yarp::sig::PixelRgb BLACK(0, 0, 0);
    static const yarp::sig::PixelRgb WHITE(255, 255, 255);
    static const yarp::sig::PixelRgb GREY(100, 100, 100);
    static const yarp::sig::PixelRgb RED(255, 0, 0);
    static const yarp::sig::PixelRgb BLUE(0, 0, 255);

    static const int margin = std::min(width, height) * 0.5;

    static yarp::sig::ImageOf<yarp::sig::PixelRgb> blankImage;

    if (blankImage.getRowSize() == 0)
    {
        blankImage.resize(width + margin * 2, height + margin * 2);

        for (auto i = 0; i < blankImage.width(); i++)
        {
            for (auto j = 0; j < blankImage.height(); j++)
            {
                blankImage.pixel(i, j) = WHITE;
            }
        }

        // footprint
        yarp::sig::draw::addRectangleOutline(blankImage, BLACK, margin + width * 0.5, margin + height * 0.5, width * 0.5, height * 0.5);

        // TCP
        yarp::sig::draw::addCircle(blankImage, BLUE, margin + cx, margin + cy, 8);
    }

    auto & image = imagePort.prepare();
    image = blankImage;

    // ZMP
    yarp::sig::draw::addCircle(image, isActive ? RED : GREY, margin + cx + zmpX, margin + cy + zmpY, 5);

    imagePort.write();
}
