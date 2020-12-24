// thanks: bullet3/examples/RobotSimulator/RobotSimulatorMain.cpp

#include <RobotSimulator/b3RobotSimulatorClientAPI.h>

#include <unistd.h>

int main(int argc, char* argv[])
{
    b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

    if(!sim->connect(eCONNECT_GUI))
    {
        printf("Fail: connect.\n");
        return 1;
    }
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeStep(1./500);
    sim->setGravity(btVector3(0, 0, -9.8));

    int planeUid = sim->loadURDF("plane.urdf");
    printf("planeUid = %d\n", planeUid);
    if(-1 == planeUid)
    {
        printf("Fail: open TEO.urdf\n");
        delete sim;
        return 1;
    }

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_startPosition.setValue(0, 0, 0.820932);
    args.m_startOrientation.setEulerZYX(0, 0, 0);
    args.m_forceOverrideFixedBase = true;
    args.m_useMultiBody = true;

    int teoUid = sim->loadURDF("TEO.urdf", args);
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
