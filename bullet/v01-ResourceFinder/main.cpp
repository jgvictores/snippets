// thanks: bullet3/examples/RobotSimulator/RobotSimulatorMain.cpp

#include <RobotSimulator/b3RobotSimulatorClientAPI.h>

#include <yarp/os/Log.h>
#include <yarp/os/ResourceFinder.h>

#include <unistd.h>

int main(int argc, char* argv[])
{
    b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

    if(!sim->connect(eCONNECT_GUI))
    {
        printf("Fail: connect.\n");
        return 1;
    }
    //sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeStep(1./500);
    sim->setGravity(btVector3(0, 0, -9.8));

    int planeUid = sim->loadURDF("plane.urdf");
    printf("planeUid = %d\n", planeUid);
    if(-1 == planeUid)
    {
        printf("Fail: open plane.urdf\n");
        delete sim;
        return 1;
    }

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_startPosition.setValue(0, 0, 0.820932);
    args.m_startOrientation.setEulerZYX(0, 0, 0);
    args.m_forceOverrideFixedBase = true;
    args.m_useMultiBody = true;

    yarp::os::ResourceFinder rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string fullEnvString = rf.findFileByName("bullet/TEO.urdf");
    yInfo("%s", fullEnvString.c_str());

    int teoUid = sim->loadURDF(fullEnvString.c_str(), args);
    printf("teoUid = %d\n", teoUid);
    if(-1 == teoUid)
    {
        printf("Fail: open TEO.urdf\n");
        delete sim;
        return 1;
    }

    for(int i=0; i<100; i++)
    {
        usleep(100000);
        sim->stepSimulation();
        printf("Iter: %d\n", i);
    }

    delete sim;
    return 0;
}
