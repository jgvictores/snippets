// Source: https://grey.colorado.edu/coin3d/classSoOffscreenRenderer.html

#include <Inventor/SoDB.h>
  #include <Inventor/SoOffscreenRenderer.h>
  #include <Inventor/engines/SoInterpolateVec3f.h>
  #include <Inventor/nodes/SoCube.h>
  #include <Inventor/nodes/SoDirectionalLight.h>
  #include <Inventor/nodes/SoPerspectiveCamera.h>
  #include <Inventor/nodes/SoSeparator.h>

  #include <iostream>

  int main()
  {
    // Init Coin
    SoDB::init();

    // The root node
    SoSeparator * root = new SoSeparator;
    root->ref();

    // It is mandatory to have at least one light for the offscreen renderer
    SoDirectionalLight * light = new SoDirectionalLight;
    root->addChild(light);

    // It is mandatory to have at least one camera for the offscreen renderer
    SoPerspectiveCamera * camera = new SoPerspectiveCamera;
    SbRotation cameraRotation = SbRotation::identity();
    cameraRotation *= SbRotation(SbVec3f(1, 0, 0), -0.4f);
    cameraRotation *= SbRotation(SbVec3f(0, 1, 0), 0.4f);
    camera->orientation = cameraRotation;
    root->addChild(camera);

    // Something to show... A box
    SoCube * cube = new SoCube;
    root->addChild(cube);

    // Set up the two camera positions we want to move the camera between
    SoInterpolateVec3f * interpolate = new SoInterpolateVec3f;
    interpolate->input0 = SbVec3f(2, 2, 9);
    interpolate->input1 = SbVec3f(2, 2, 5);
    camera->position.connectFrom(&interpolate->output);

    // Set up the offscreen renderer
    SbViewportRegion vpRegion(400, 300);
    SoOffscreenRenderer offscreenRenderer(vpRegion);

    // How many frames to render for the video
    int frames = 5;
    std::cout << "Writing " << frames << " frames..." << std::endl;

    for (int i = 0; i < frames; i++) {
      // Update the camera position
      interpolate->alpha = float(i) / (frames - 1);

      // Render the scene
      SbBool ok = offscreenRenderer.render(root);

      // Save the image to disk
      SbString filename = SbString("coinvideo-") + (i + 1) + ".jpg";
      if (ok) {
        offscreenRenderer.writeToFile(filename.getString(), "jpg");
      } else {
        std::cout << "Error saving image: " << filename.getString() << std::endl;
        break;
      }
    }

    std::cout << "Done!" << std::endl;

    root->unref();
    return 0;
  }
