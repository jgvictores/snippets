/** \example or2dplanning.cpp
    \author Juan G Victores
    \contrib Rosen Diankov (author of orplanning_module.cpp)

    Shows how to use a planner from a module to move the arm withut colliding into anything.
    The default values plan for Base of the robot.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningModuleExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {

        string scenefilename = "/home/yo/asibot/main/app/ravebot/models/asibot_kitchen_wheelchair.env.xml";
        penv->Load(scenefilename);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);

        KinBodyPtr objPtr = penv->GetKinBody("asibot");
        if(objPtr) {
            if(vrobots.at(1)->Grab(objPtr)) printf("[success] object asibot exists and grabbed.\n");
            else printf("[warning] object asibot exists but not grabbed.\n");
        } else printf("[fail] object asibot does not exist.\n");

        printf("begin wait\n");
        sleep(5);
        printf("end wait\n");
        RobotBasePtr probot = vrobots.at(1);

        vector<dReal> v(2);

        /* vector<int> vindices(probot->GetDOF());
        for(size_t i = 0; i < vindices.size(); ++i) {
            vindices[i] = i;
        }
        probot->SetActiveDOFs(vindices);*/

        vector<int> vindices;  // send it empty instead
        //probot->SetActiveDOFs(vindices,DOF_X|DOF_Y|DOF_RotationAxis,Vector(0,0,1));
        probot->SetActiveDOFs(vindices,DOF_X|DOF_Y,Vector(0,0,1));

        ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
        penv->Add(pbasemanip,true,probot->GetName()); // load the module

        int iteracion = 0;

        while(IsOk()) {
            {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

                // find a set of free joint values for the robot
                {
                    RobotBase::RobotStateSaver saver(probot); // save the state
                    while(1) {
                        Transform T = probot->GetTransform();
                        if(iteracion%2) {
                            v[0] = T.trans.x;
                            v[1] = T.trans.y+1;
                            //v[2] = T.trans.z;
                            iteracion++;
                        } else {
                            v[0] = T.trans.x;
                            v[1] = T.trans.y-0.5;
                            //v[2] = T.trans.z;
                            iteracion++;
                        }
                        if(iteracion==5) vrobots.at(1)->Release(objPtr);
                        probot->SetActiveDOFValues(v);
                        if( !penv->CheckCollision(probot) && !probot->CheckSelfCollision() ) {
                            break;
                        }
                    }
                    // robot state is restored
                }

                stringstream cmdin,cmdout;
                cmdin << "MoveActiveJoints goal ";
                for(size_t i = 0; i < v.size(); ++i) {
                    cmdin << v[i] << " ";
                }

                // start the planner and run the robot
                RAVELOG_INFO("%s\n",cmdin.str().c_str());
                if( !pbasemanip->SendCommand(cmdout,cmdin) ) {
                    continue;
                }
            }

            // unlock the environment and wait for the robot to finish
            while(!probot->GetController()->IsDone() && IsOk()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PlanningModuleExample example;
    return example.main(argc,argv);
}

