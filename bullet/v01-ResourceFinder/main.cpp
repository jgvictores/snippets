// thanks: bullet3/examples/RobotSimulator/RobotSimulatorMain.cpp

#include <RobotSimulator/b3RobotSimulatorClientAPI.h>

#include <yarp/os/Log.h>
#include <yarp/os/ResourceFinder.h>

#include <unistd.h>

int main(int argc, char* argv[])
{
    b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

	bool isConnected = sim->connect(eCONNECT_GUI);

    usleep(2000000);

    yarp::os::ResourceFinder rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string fullEnvString = rf.findFileByName("bullet/TEO.urdf");
    yInfo("%s", fullEnvString.c_str());

    if ( ! sim->loadURDF( fullEnvString.c_str()) )
    {
        yError("Could not load '%s' environment.\n",fullEnvString.c_str());
        return 1;
    }

    usleep(2000000);

    return 0;
}
