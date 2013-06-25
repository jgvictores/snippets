
#include "XnCppWrapper.h"

int main(int argc, char *argv[]) {

XnStatus nRetVal = XN_STATUS_OK;

xn::Context context;
nRetVal = context.Init();
if(nRetVal!=XN_STATUS_OK) printf("Worry1\n");

xn::DepthGenerator depth;
nRetVal = depth.Create(context);
if(nRetVal!=XN_STATUS_OK) printf("Worry2\n");

// Make it start generating data
nRetVal = context.StartGeneratingAll();
// TODO: check error code

// Build a query object
xn::Query query;
nRetVal = query.SetVendor("MyVendor");
// TODO: check error code

query.AddSupportedCapability(XN_CAPABILITY_SKELETON);
// TODO: check error code

// Enumerate
xn::NodeInfoList possibleChains;
nRetVal = context.EnumerateProductionTrees(XN_NODE_TYPE_USER, &query, possibleChains, NULL);
// TODO: check error code

// No errors so far. This means list has at least one item. Take the first one
xn::NodeInfo selected = *possibleChains.Begin();

// Create it
nRetVal = context.CreateProductionTree(selected);
// TODO: check error code

// Take the node
xn::UserGenerator userGen;
nRetVal = selected.GetInstance(userGen);
// TODO: check error code

// Now we can start to use it


    return 0;
}
