#include "stkCGALPoissonReconstructionMesher.h"

//---------VTK----------------------------------
#include <vtkDoubleArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <vtkTimerLog.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/pca_estimate_normals.h>
// #include <CGAL/scanline_orient_normals.h> // need 5.3
#include <CGAL/Surface_mesh.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/property_map.h>
#include <boost/iterator/transform_iterator.hpp>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> Point_with_normal;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef Kernel::Sphere_3 Sphere;
typedef std::vector<Point_with_normal> PointList;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;
typedef CGAL::Surface_mesh<Point> Mesh;

//---------Module-------------------------------
#include <stkCGALUtilities.h>

// Concurrency
typedef CGAL::Parallel_if_available_tag Concurrency_tag;

vtkStandardNewMacro(stkCGALPoissonReconstructionMesher);

//----------------------------------------------------------------------------
int stkCGALPoissonReconstructionMesher::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* pointSetInput = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));

  vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  // Poisson options
  FT sm_angle = this->MinTriangleAngle;       // Min triangle angle in degrees.
  FT sm_radius = this->MaxTriangleSizeMultiplier;     // Max triangle size with reference to the input point set's average spacing.
  FT sm_distance = this->ApproximationErrorMultiplier; // Surface Approximation error with reference to the input point set's average spacing.

  if (pointSetInput == nullptr)
  {
    vtkErrorMacro(" input is null.");
    return 0;
  }

  if (pointSetInput->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro(" input does not contain a normal array");
    return 0;
  }

  if (!this->PointNormalsArrayName.empty() && !this->UseNormalEstimation)
  {
    auto pointNormalsArray =
      pointSetInput->GetPointData()->GetArray(this->PointNormalsArrayName.c_str());

    if (pointNormalsArray != nullptr)
    {
      pointSetInput->GetPointData()->SetNormals(pointNormalsArray);
    }
    else
    {
      vtkErrorMacro("Selected Point Normals Array is not valid");
      return 0;
    }
  }
  else if (this->PointNormalsArrayName.empty() && !this->UseNormalEstimation)
  {
    vtkErrorMacro("Point Normals Array Name is not selected");
    return 0;
  }

  vtkDataArray* normals = nullptr;

  if (!this->UseNormalEstimation)
  {
    normals = pointSetInput->GetPointData()->GetNormals();
    if (!normals)
    {
      vtkErrorMacro(" input does not contain a normals array, please add one.");
      return 0;
    }
  }
  // Start
  this->SetProgressText("CGAL Poisson Reconstruction Mesher");
  this->UpdateProgress(0);

  // Convert from VTK to CGAL
  vtkTimerLog::MarkStartEvent("VTK To CGAL conversion");

  PointList points;
  double p[3] = {0.0};
  double n[3] = {0.0};
  for (int i = 0; i < pointSetInput->GetNumberOfPoints(); ++i)
  {

    pointSetInput->GetPoint(i, p);

    if (this->UseNormalEstimation)
    {
      Point_with_normal ptn(std::make_pair(Point(p[0], p[1], p[2]), Vector(0, 0, 0)));
      points.push_back(ptn);
    }
    else
    {
      normals->GetTuple(i, n);
      Point_with_normal ptn(std::make_pair(Point(p[0], p[1], p[2]), Vector(n[0], n[1], n[2])));
      points.push_back(ptn);
    }
  }

  if (this->UseNormalEstimation)
  {
    if (this->NormalEstimationFunction ==
      stkCGALPoissonReconstructionMesher::NormalEstimationFunctions::PCA)
    {
      {
        CGAL::pca_estimate_normals<Concurrency_tag>(points, this->NumberOfNeighbors,
          CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));

        // Orientation of normals, returns iterator to first unoriented point
        typename PointList::iterator unoriented_points_begin =
          CGAL::mst_orient_normals(points, this->NumberOfNeighbors,
            CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));
        points.erase(unoriented_points_begin, points.end());
      }
    }
    else if (this->NormalEstimationFunction ==
      stkCGALPoissonReconstructionMesher::NormalEstimationFunctions::JET)
    {
      CGAL::jet_estimate_normals<Concurrency_tag>(points, this->NumberOfNeighbors,
        CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));

      // Orientation of normals, returns iterator to first unoriented point
      typename PointList::iterator unoriented_points_begin = CGAL::mst_orient_normals(points,
        this->NumberOfNeighbors, CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));
      points.erase(unoriented_points_begin, points.end());
    }
  }

  vtkTimerLog::MarkEndEvent("VTK To CGAL conversion");
  this->UpdateProgress(0.1);

  // Generate mesh
  vtkTimerLog::MarkStartEvent("Mesh Generation");

  // Creates implicit function from the read points using the default solver.
  // Note: this method requires an iterator over points
  // + property maps to access each point's position and normal.
  Poisson_reconstruction_function function(points.begin(), points.end(), Point_map(), Normal_map());
  // Computes the Poisson indicator function f()
  // at each vertex of the triangulation.
  if (!function.compute_implicit_function(true))
  {
    vtkErrorMacro("Unable to compute the Poisson indicator function.");
    return 0;
  }
  // Computes average spacing
  FT average_spacing = CGAL::compute_average_spacing<Concurrency_tag>(
    points, 6 /* knn = 1 ring */, CGAL::parameters::point_map(Point_map()));
  // Gets one point inside the implicit surface
  // and computes implicit function bounding sphere radius.
  Point inner_point = function.get_inner_point();
  Sphere bsphere = function.bounding_sphere();
  FT radius = std::sqrt(bsphere.squared_radius());
  // Defines the implicit surface: requires defining a
  // conservative bounding sphere centered at inner point.
  FT sm_sphere_radius = 5.0 * radius;
  FT sm_dichotomy_error =
    sm_distance * average_spacing / 1000.0; // Dichotomy error must be << sm_distance
  Surface_3 surface(function, Sphere(inner_point, sm_sphere_radius * sm_sphere_radius),
    sm_dichotomy_error / sm_sphere_radius);
  // Defines surface mesh generation criteria
  CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle, // Min triangle angle (degrees)
    sm_radius * average_spacing,                                // Max triangle size
    sm_distance * average_spacing);                             // Approximation error

  this->UpdateProgress(0.3);
  // Generates surface mesh with manifold option
  STr tr;        // 3D Delaunay triangulation for surface mesh generation
  C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
  switch (this->TopologicalSpace)
  {
    case TopologicalSpaces::MANIFOLD:
      try
      {
        CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());
      }
      catch (const std::exception& e)
      {
        vtkErrorMacro(<< "Error caught : " << e.what());
        return 0;
      }
      break;
    case TopologicalSpaces::MANIFOLD_WITH_BOUNDARY:
      try
      {
        CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_with_boundary_tag());
      }
      catch (const std::exception& e)
      {
        vtkErrorMacro(<< "Error caught : " << e.what());
        return 0;
      }
      break;
    default:
      try
      {
        CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());
      }
      catch (const std::exception& e)
      {
        vtkErrorMacro(<< "Error caught : " << e.what());
        return 0;
      }
      break;
  }

  if (tr.number_of_vertices() == 0)
  {
    vtkErrorMacro("Unable to generate mesh.");
    return 0;
  }

  this->UpdateProgress(0.9);
  vtkTimerLog::MarkEndEvent("Mesh Generation");

  // Convert back to VTK
  vtkTimerLog::MarkStartEvent("CGAL to VTK conversion");
  Mesh pmesh;
  CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, pmesh);
  stkCGALUtilities::SurfaceMeshToPolyData(pmesh, output);
  this->UpdateProgress(1.0);
  vtkTimerLog::MarkEndEvent("CGAL to VTK conversion");

  return 1;
}
