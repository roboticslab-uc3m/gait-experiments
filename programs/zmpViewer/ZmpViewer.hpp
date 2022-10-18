// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ZMP_VIEWER_HPP__
#define __ZMP_VIEWER_HPP__

#include <mutex>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/sig/Image.h>

namespace roboticslab
{

/**
 * @ingroup zmpViewer
 * @brief Connects to a remote ZMP publisher port and generates an image.
 */
class ZmpViewer : public yarp::os::RFModule,
                  public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    ~ZmpViewer() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;

    bool updateModule() override;

    double getPeriod() override;

    bool close() override;

    void onRead(yarp::os::Bottle & b) override;

private:
    void drawAndPublishImage(bool isActive, int zmpX, int zmpY);

    yarp::os::Port zmpPort;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> zmpPortReader;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;

    int width;
    int height;
    int cx;
    int cy;

    double lastStamp {0.0};
    int lastZmpX {0};
    int lastZmpY {0};
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __ZMP_VIEWER_HPP__
