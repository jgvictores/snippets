#include <yarp/os/all.h>
#include <yarp/sig/all.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelFloat >> port;
    port.open("/cpp/ecroSim/depthImage:i");
    yarp::os::Network::connect("/ecroSim/depthImage:o","/cpp/ecroSim/depthImage:i");

    yarp::sig::ImageOf< yarp::sig::PixelFloat>* yarpImage = port.read(true);  // true makes blocking

    printf("Got image: height %d, width %d\n",yarpImage->height(),yarpImage->width());

    printf("Pixel (0,0): %f [mm] depth\n",yarpImage->pixel(0,0));

    // For OpenCV example see:
    // https://github.com/robotology/yarp/blob/master/example/opencv/main.cpp

    port.close();

    return 0;
}
