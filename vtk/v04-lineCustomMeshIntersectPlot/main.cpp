#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkOBBTree.h>
 
#include "vtkPolygon.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h" 

int main(int, char *[]) {

  // Read in the cloud data
/*  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("kinect_cloud.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; */

  // Create the object
  vtkSmartPointer<vtkPolyData> object = vtkSmartPointer<vtkPolyData>::New();

  // Create a square in the XY plane
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 1.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
 
  // Create the polygon
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(4); // 4 corners of the square
  polygon->GetPointIds()->SetId(0, 0);
  polygon->GetPointIds()->SetId(1, 1);
  polygon->GetPointIds()->SetId(2, 2);
  polygon->GetPointIds()->SetId(3, 3);

  vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New ();
  polygons->InsertNextCell(polygon);

  object->SetPoints (points);
  object->SetPolys (polygons);

  // Create the locator
  vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
  tree->SetDataSet(object);
  tree->BuildLocator();
 
  // Intersect the locator with the line
  double lineP0[3] = {0.0, 0.0, -2.0};
  double lineP1[3] = {0.0, 0.0, 2.0};
  vtkSmartPointer<vtkPoints> intersectPoints = 
    vtkSmartPointer<vtkPoints>::New();
 
  tree->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);

  std::cout << "NumPoints: " << intersectPoints->GetNumberOfPoints()
            << std::endl;

  for(int i=0;i<intersectPoints->GetNumberOfPoints();i++) {
      double intersection[3];
      intersectPoints->GetPoint(i, intersection);
      std::cout << "Intersection: " 
                << intersection[0] << ", " 
                << intersection[1] << ", "
                << intersection[2] << std::endl;
  }
 
  // map to graphics library
  vtkPolyDataMapper *map = vtkPolyDataMapper::New();
  map->SetInput(object);

  // actor coordinates geometry, properties, transformation
  vtkActor *aObject = vtkActor::New();
  aObject->SetMapper(map);
  aObject->GetProperty()->SetColor(0,0,1); // sphere color blue

  // a renderer and render window
  vtkRenderer *ren1 = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren1);

  // an interactor
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  // add the actor to the scene
  ren1->AddActor(aObject);
  ren1->SetBackground(1,1,1); // Background color white

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
