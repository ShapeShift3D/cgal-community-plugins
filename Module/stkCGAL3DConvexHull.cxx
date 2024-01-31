#include "stkCGAL3DConvexHull.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPointSet.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/convex_hull_3_to_face_graph.h>

//---------Module-------------------------------
#include <stkCGALUtilities.h>

#include <list>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_Mesh;

typedef CGAL::Delaunay_triangulation_3<K> Delaunay;
typedef Delaunay::Vertex_handle Vertex_handle;

vtkStandardNewMacro(stkCGAL3DConvexHull);

// ----------------------------------------------------------------------------
int stkCGAL3DConvexHull::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkPointSet* inputPointSet = this->GetInputPointSet();

  vtkPolyData* outputMesh = vtkPolyData::GetData(outputVector, 0);

  if (inputPointSet == nullptr || inputPointSet->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("There are no points in the input");
    return 0;
  }

  std::vector<Point_3> points;

  for (vtkIdType pointID = 0; pointID < inputPointSet->GetNumberOfPoints(); pointID++)
  {
    auto vtkPoint = inputPointSet->GetPoint(pointID);
    points.emplace_back(vtkPoint[0], vtkPoint[1], vtkPoint[2]);
  }

  Surface_Mesh convexHull;

  if (this->ConvexHullMethod == ConvexHullMethods::QUICKHULL)
  {
    CGAL::convex_hull_3(points.begin(), points.end(), convexHull);
  }
  else if (this->ConvexHullMethod == ConvexHullMethods::DELAUNAY3D)
  {
    Delaunay T;
    T.insert(points.begin(), points.end());

    std::list<Vertex_handle> vertices;
    T.incident_vertices(T.infinite_vertex(), std::back_inserter(vertices));
    CGAL::convex_hull_3_to_face_graph(T, convexHull);
  }
  else
  {
    vtkErrorMacro("Method to generate convex hull is not selected");
    return 0;
  }

  stkCGALUtilities::SurfaceMeshToPolyData(convexHull, outputMesh);

  return 1;
}
