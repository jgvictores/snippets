
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_utils.h>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCylinderSource.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkOBBTree.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>


int main(int, char *[]) {

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../arm.pcd", cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

///////////////////////////////////////////////////////////////////////////////

  // Create the kinect-obtained object
  vtkSmartPointer<vtkPolyData> object = vtkSmartPointer<vtkPolyData>::New();

  // Pass the surface mesh to the kinect-obtained object
  pcl::VTKUtils::convertToVTK(triangles,object);

  // Create the locator
  vtkSmartPointer<vtkOBBTree> otree = vtkSmartPointer<vtkOBBTree>::New();
  otree->SetDataSet(object);
  otree->BuildLocator();
 
  // Intersect the locator with the line
  double lineP0[3] = {-0.2, 0.0, -2.0};
  double lineP1[3] = {-0.2, 0.0, 2.0};
  vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
 
  otree->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);

  std::cout << "NumPoints: " << intersectPoints->GetNumberOfPoints() << std::endl;

  for(int i=0;i<intersectPoints->GetNumberOfPoints();i++) {
      double intersection[3];
      intersectPoints->GetPoint(i, intersection);
      std::cout << "Intersection: " << intersection[0] << ", "  << intersection[1] << ", "
          << intersection[2] << std::endl;
  }
 
  // Let's create a graphical representation of the line
  vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(lineP0);
  lineSource->SetPoint2(lineP1);
  lineSource->Update();

  // Create axis cylinders
  vtkSmartPointer<vtkCylinderSource> axisXSource = vtkSmartPointer<vtkCylinderSource>::New();
  vtkSmartPointer<vtkCylinderSource> axisYSource = vtkSmartPointer<vtkCylinderSource>::New();
  vtkSmartPointer<vtkCylinderSource> axisZSource = vtkSmartPointer<vtkCylinderSource>::New();
  axisXSource->SetRadius(0.005);
  axisXSource->SetHeight(0.1);
  axisXSource->SetCenter(0.05, 0.0, 0.0);
  axisXSource->Update();
  axisYSource->SetRadius(0.005);
  axisYSource->SetHeight(0.1);
  axisYSource->SetCenter(0.0, 0.05, 0.0);
  axisYSource->Update();
  axisZSource->SetRadius(0.005);
  axisZSource->SetHeight(0.1);
  axisZSource->SetCenter(0.0, 0.0, 0.05);
  axisZSource->Update();

  // Maps to graphics library
  vtkPolyDataMapper *map = vtkPolyDataMapper::New();
  vtkPolyDataMapper *map2 = vtkPolyDataMapper::New();
  vtkPolyDataMapper *axisXMap = vtkPolyDataMapper::New();
  vtkPolyDataMapper *axisYMap = vtkPolyDataMapper::New();
  vtkPolyDataMapper *axisZMap = vtkPolyDataMapper::New();
  map->SetInput(object);
  map2->SetInput(lineSource->GetOutput());
  axisXMap->SetInput(axisXSource->GetOutput());
  axisYMap->SetInput(axisYSource->GetOutput());
  axisZMap->SetInput(axisZSource->GetOutput());

  // Actor coordinates geometry, properties, transformation
  vtkActor *aObject = vtkActor::New();
  vtkActor *aLine = vtkActor::New();
  vtkActor *aAxisX = vtkActor::New();
  vtkActor *aAxisY = vtkActor::New();
  vtkActor *aAxisZ = vtkActor::New();

  aObject->SetMapper(map);
  aLine->SetMapper(map2);
  aAxisX->SetMapper(axisXMap);
  aAxisY->SetMapper(axisYMap);
  aAxisZ->SetMapper(axisZMap);

  aObject->GetProperty()->SetColor(0,0,1); // sphere color blue
  aLine->GetProperty()->SetColor(1,1,1); // line color green
  aAxisX->GetProperty()->SetColor(1,0,0); // sphere color red
  aAxisY->GetProperty()->SetColor(0,1,0); // sphere color green
  aAxisZ->GetProperty()->SetColor(0,0,1); // sphere color blue

  // a renderer and render window
  vtkRenderer *ren1 = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren1);

  // an interactor
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
  axes->SetTotalLength(0.1,0.1,0.1);
  axes->AxisLabelsOff();
//  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//  transform->Translate(1.0, 0.0, 0.0);
//  axes->SetUserTransform(transform);

  // add the actor to the scene
  ren1->AddActor(aObject);
  ren1->AddActor(aLine);
  ren1->AddActor(axes);
/*  ren1->AddActor(aAxisX);
  ren1->AddActor(aAxisY);
  ren1->AddActor(aAxisZ);*/
  ren1->SetBackground(0,0,0); // Background color white

  // render an image (lights and cameras are created automatically)
  renWin->Render();

  // begin mouse interaction
  iren->Start();

  // release memory and return
  object->Delete();
  map->Delete();
  ren1->Delete();
  renWin->Delete();
  iren->Delete();
  return 0;
} 
