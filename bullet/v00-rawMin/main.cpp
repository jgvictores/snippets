// thanks: bullet3/examples/RobotSimulator/RobotSimulatorMain.cpp

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
//#include "b3RobotSimulatorClientAPI.h"

#include <unistd.h>

int main(int argc, char* argv[])
{
    b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

	bool isConnected = sim->connect(eCONNECT_GUI);

    usleep(2000000);
    return 0;
}
