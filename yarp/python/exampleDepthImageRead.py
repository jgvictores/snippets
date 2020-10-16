#! /usr/bin/env python

import numpy
import yarp
import matplotlib.pylab
import time

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print "[error] Please try running yarp server"
    quit()

input_port = yarp.Port()
input_port.open("/python/ecroSim/depthImage:i")
if not yarp.Network.connect("/ecroSim/depthImage:o", "/python/ecroSim/depthImage:i"):
    print "[error] Could not connect"
    quit()

# Just once to get measurements
yarp_tmp_image = yarp.ImageFloat()
input_port.read(yarp_tmp_image)
height = yarp_tmp_image.height()
width = yarp_tmp_image.width()

# Create numpy array to receive the image and the YARP image wrapped around it
img_array = numpy.zeros((height, width), dtype=numpy.float32)
yarp_image = yarp.ImageFloat()
yarp_image.resize(width, height)
yarp_image.setExternal(img_array, img_array.shape[1], img_array.shape[0])
# Read the data from the port into the image
input_port.read(yarp_image)
if yarp_image.getRawImage().__long__() <> img_array.__array_interface__['data'][0]:
    print "read() reallocated my yarp_image!"
# display the image that has been read
matplotlib.pylab.imshow(img_array,interpolation='none')
matplotlib.pylab.show()

# Cleanup
input_port.close()

yarp.Network.fini();
