#include "stkCGALPolygonMeshSmoothingOperator.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <stkCGALUtilities.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::edge_descriptor edge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

vtkStandardNewMacro(stkCGALPolygonMeshSmoothingOperator);

// ----------------------------------------------------------------------------
int stkCGALPolygonMeshSmoothingOperator::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputMesh = vtkPolyData::GetData(inputVector[0]);

  if (inputMesh == nullptr)
  {
    vtkErrorMacro("Input Mesh is empty.");
    return 0;
  }

  if (inputMesh->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input Mesh does not contain any point structure.");
    return 0;
  }

  if (inputMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Mesh contains no points.");
    return 0;
  }

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  Surface_Mesh mesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, mesh);

  // Constrain edges with a dihedral angle over DihedralAngleEdgeConstraint°
  typedef boost::property_map<Surface_Mesh, CGAL::edge_is_feature_t>::type EIFMap;
  EIFMap eif = get(CGAL::edge_is_feature, mesh);
  PMP::detect_sharp_edges(mesh, this->DihedralAngleEdgeConstraint, eif);

  // It could also be possible to constrain vertices based on an input array.
  // See
  // https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html#gaa0551d546f6ab2cd9402bea12d8332a3

  // int sharp_counter = 0;
  // for (edge_descriptor e : edges(mesh))
  //   if (get(eif, e))
  //     ++sharp_counter;
  //
  // std::cout << sharp_counter << " sharp edges" << std::endl;

  // Smooth with both angle and area criteria + Delaunay flips
  PMP::smooth_mesh(mesh,
    PMP::parameters::number_of_iterations(this->NumberOfIterations)
      .use_safety_constraints(this->UseSafetyConstraints)
      .do_project(this->ProjectPointsOntoSurface)
      .edge_is_constrained_map(eif)
      .use_angle_smoothing(this->UseAngleSmoothing)
      .use_area_smoothing(this->UseAreaSmoothing)); // Requires Ceres.

  stkCGALUtilities::SurfaceMeshToPolyData(mesh, output0);
  return 1;
}
