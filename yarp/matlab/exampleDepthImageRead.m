LoadYarp;  % yarp.matlab.LoadYarp

inputPort = yarp.BufferedPortImageFloat;
inputPort.open('/matlab/ecroSim/depthImage:i');
yarp.Network.connect('/ecroSim/depthImage:o', '/matlab/ecroSim/depthImage:i');

yarpImage = inputPort.read;
h = yarpImage.height;
w = yarpImage.width;
%tool to convert the yarpImage (a Java object) into a matlab matrix
tool = YarpImageHelper(h, w);  % yarp.matlab.YarpImageHelper
image = tool.get2DMatrix(yarpImage);
%norm to visualize
normImage = image - min(image(:));
normImage = normImage ./ max(normImage(:)); % *
imshow(normImage);

inputPort.close();
