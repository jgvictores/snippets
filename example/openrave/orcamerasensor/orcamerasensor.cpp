/** \example orcamerasensor.cpp
    \author Juan G. Victores, based on source code written by Rosen Diankov.
    Additional contributions by J. G. Quijano

    Loads a robot into the openrave environment, starts a viewer, and attempts
    to grab camera images.
    
    Usage:
    \verbatim
    orcamerasensor [--num n] [--scene filename] viewername
    \endverbatim

    - \b --num - Number of environments/viewers to create simultaneously
    - \b --scene - The filename of the scene to load.

    Example:
    \verbatim
    ./orcamerasensor --scene data/testwamcamera.env.xml qtcoin
    \endverbatim

    <b>Full Example Code:</b>
*/    
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
    
}

int main(int argc, char ** argv)
{
    //int num = 1;
    string scenefilename = "data/testwamcamera.env.xml";
    string viewername = "qtcoin";

    // parse the command line options
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            RAVELOG_INFO("orloadviewer [--num n] [--scene filename] viewername\n");
            return 0;
        }
        else if( strcmp(argv[i], "--scene") == 0 ) {
            scenefilename = argv[i+1];
            i += 2;
        }
        else
            break;
    }
    if( i < argc ) {
        viewername = argv[i++];
    }
    
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    RaveSetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
    penv->Load(scenefilename); // load the scene

    //-- Get Robot 0
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);
    std::cout << "Robot 0: " << robots.at(0)->GetName() << std::endl;  // default: BarrettWAM

    //-- Get Robot 0's camera ptr, description and name
    std::vector<RobotBase::AttachedSensorPtr> sensors;
    sensors = robots.at(0)->GetAttachedSensors();
    SensorBasePtr psensorbase = sensors.at(0)->GetSensor();
    if(psensorbase == NULL) printf("[error] Bad sensorbase, may break in future because of this.\n");
    else printf("Good sensorbase...\n");
    std::string tipo = psensorbase->GetDescription();
    printf("%s\n",tipo.c_str());
    tipo = psensorbase->GetName();
    printf("%s\n",tipo.c_str());

    // Get some camera parameter info
    boost::shared_ptr<SensorBase::CameraGeomData> pcamerageomdata = boost::dynamic_pointer_cast<SensorBase::CameraGeomData>(psensorbase->GetSensorGeometry(SensorBase::ST_Camera));
    int imgw = pcamerageomdata->width;
    int imgh = pcamerageomdata->height;
    printf("Camera width: %d, height: %d.\n",imgw,imgh);

    // Get a pointer to access the camera data stream
    boost::shared_ptr<SensorBase::CameraSensorData> pcamerasensordata = boost::dynamic_pointer_cast<SensorBase::CameraSensorData>(psensorbase->CreateSensorData(SensorBase::ST_Camera));
    // Activate the camera
    stringstream sin, sout;
    sin << "power 1";
    if (psensorbase->SendCommand(sout,sin))
      cout << "Sending power camera on command success, response: " << sout.str() << endl;
    else printf("[error] Could not power camera on.\n");

    psensorbase->SimulationStep(0.1);
    psensorbase->GetSensorData(pcamerasensordata);

    // from <stdint.h>: typedef signed char int8_t
    //std::vector<uint8_t> currentFrame = pcamerasensordata->vimagedata;
    printf("Vector size: %d",pcamerasensordata->vimagedata.size()); // = 480 * 640 * 3 = 921600;
    printf(" ( %d * %d * 3 = %d )\n", imgw, imgh, imgw*imgh*3);
    printf("A blocking [cin] waiting for input before dumping camera image...\n");
    int null;
    cin >> null;
    for (int i_x = 0; i_x < imgw; ++i_x) {
      for (int i_y = 0; i_y < imgh; ++i_y) {
        printf("%d",pcamerasensordata->vimagedata[3*(i_x+(i_y*imgw))]); // r
        printf("%d",pcamerasensordata->vimagedata[1+3*(i_x+(i_y*imgw))]); // g
        printf("%d",pcamerasensordata->vimagedata[2+3*(i_x+(i_y*imgw))]); // b
      }
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
