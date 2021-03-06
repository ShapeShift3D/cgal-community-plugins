/**
 * \class vtkCGALPolygonMeshSmoothingOperator
 *
 * \brief Smooths a triangulated region of a polydata by attempting to make the triangle angle and
 * area distributions as uniform as possible by moving non-constrained vertices.
 *
 * Constraints are determined by the chosen dihedral angle of the edges.
 *
 * Angle-based smoothing does not change the combinatorial information of the mesh. Area-based
 * smoothing might change the combinatorial information, unless specified otherwise. It is also
 * possible to make the smoothing algorithm "safer" by rejecting moves that, when applied, would
 * worsen the quality of the mesh, e.g. that would decrease the value of the smallest angle around a
 * vertex or create self-intersections.
 *
 * To use the area-based smoothing, Ceres needs to be binded to the CGAL library. This implies the
 * need to also bind LAPACK and BLAS.
 *
 */

//---------VTK----------------------------------
#include "vtkCGALPolygonMeshSmoothingOperator.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALPolygonMeshSmoothingOperator);

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::edge_descriptor edge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALPolygonMeshSmoothingOperator::vtkCGALPolygonMeshSmoothingOperator()
  : UseAngleSmoothing(true)
  , UseAreaSmoothing(false)
  , NumberOfIterations(10)
  , DihedralAngleEdgeConstraint(60.0)
  , UseSafetyConstraints(false)
  , ProjectPointsOntoSurface(true)
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALPolygonMeshSmoothingOperator::RequestData(
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
  vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, mesh);

  // Constrain edges with a dihedral angle over DihedralAngleEdgeConstraint°
  typedef boost::property_map<Surface_Mesh, CGAL::edge_is_feature_t>::type EIFMap;
  EIFMap eif = get(CGAL::edge_is_feature, mesh);
  PMP::detect_sharp_edges(mesh, this->DihedralAngleEdgeConstraint, eif);

  // It could also be possible to constrain vertices based on an input array. 
  // See https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html#gaa0551d546f6ab2cd9402bea12d8332a3

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

  vtkCGALUtilities::SurfaceMeshToPolyData(mesh, output0);
  return 1;
}
