/**
 * \class stkCGALBoolean3DMesher
 *
 * \brief This filter takes two inputs, inputMeshA and inputMeshB, of type vtkPolyData and applies
 *one of the four boolean operations (Union, Intersection, Difference1 (A - B) and Difference2 (B -
 *A)) to them. The user will have the option to select one of the four operations from a drop down
 *menu. The two inputs will be converted to CGAL Polygon Mesh class since vtkPolyData is not a valid
 *input. The converted inputs will then be fed into the appropriate function for execution. The
 *result of the function will be converted to a vtkUnstructuredGrid and be outputted as such.
 *
 *
 *
 * Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
 * Output: output (port == 0, vtkUnstructuredGrid)
 *
 */

//---------VTK----------------------------------
#include "stkCGALBoolean3DMesher.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <stkCGALUtilities.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

vtkStandardNewMacro(stkCGALBoolean3DMesher);

// ----------------------------------------------------------------------------
int stkCGALBoolean3DMesher::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputMeshA = this->GetInputMeshA();
  vtkPolyData* inputMeshB = this->GetInputMeshB();

  if (inputMeshA == nullptr)
  {
    vtkErrorMacro("Input Mesh A is empty.");
    return 0;
  }

  if (inputMeshA->GetPolys() == nullptr)
  {
    vtkErrorMacro("Input Mesh A does not contain any cell structure.");
    return 0;
  }

  if (inputMeshA->GetNumberOfCells() == 0)
  {
    vtkErrorMacro("Input Mesh A contains no cells.");
    return 0;
  }

  if (inputMeshA->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input Mesh A does not contain any point structure.");
    return 0;
  }

  if (inputMeshA->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Mesh A contains no points.");
    return 0;
  }

  if (inputMeshB == nullptr)
  {
    vtkErrorMacro("Input Mesh B is empty.");
    return 0;
  }

  if (inputMeshB->GetPolys() == nullptr)
  {
    vtkErrorMacro("Input Mesh B does not contain any cell structure.");
    return 0;
  }

  if (inputMeshB->GetNumberOfCells() == 0)
  {
    vtkErrorMacro("Input Mesh B contains no cells.");
    return 0;
  }

  if (inputMeshB->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input Mesh B does not contain any point structure.");
    return 0;
  }

  if (inputMeshB->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Mesh B contains no points.");
    return 0;
  }

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  vtkPolyData* output1 = vtkPolyData::GetData(outputVector->GetInformationObject(1));

  Surface_Mesh meshA, meshB;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshA, meshA);
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshB, meshB);
  Surface_Mesh boolean_op;

  // Preconditions
  if (!this->SkipPreconditions)
  {
    if (CGAL::Polygon_mesh_processing::does_self_intersect(meshA))
    {
      vtkErrorMacro("Input mesh A contains self-intersections.");
      return 0;
    }

    if (CGAL::Polygon_mesh_processing::does_self_intersect(meshB))
    {
      vtkErrorMacro("Input mesh B contains self-intersections.");
      return 0;
    }

    if (!CGAL::Polygon_mesh_processing::does_bound_a_volume(meshA))
    {
      vtkErrorMacro("Input mesh A does not bound a volume.");
      return 0;
    }

    if (!CGAL::Polygon_mesh_processing::does_bound_a_volume(meshB))
    {
      vtkErrorMacro("Input mesh B does not bound a volume.");
      return 0;
    }
  }

  try
  {
    RunBooleanOperations<Surface_Mesh>(meshA, meshB, boolean_op);
  }
  catch (const std::exception& e)
  {
    vtkErrorMacro(<< "Error caught : " << e.what());
    return 0;
  }

  stkCGALUtilities::SurfaceMeshToPolyData(boolean_op, output0);
  output0->Squeeze(); // TODO: Check if useful

  return 1;
}

//---------------------------------------------------------------------------------------
template<typename SurfaceMesh>
SurfaceMesh* stkCGALBoolean3DMesher::RunBooleanOperations(
  SurfaceMesh& tm1, SurfaceMesh& tm2, SurfaceMesh& operation)
{
  typedef boost::optional<SurfaceMesh*> OSM;
  std::array<OSM, 4> output;

  if (this->Mode == stkCGALBoolean3DMesher::Modes::UNION)
  {
    output[CGAL::Polygon_mesh_processing::Corefinement::UNION] = OSM(&operation);
    CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
    return *output[0];
  }
  if (this->Mode == stkCGALBoolean3DMesher::Modes::INTERSECTION)
  {
    output[CGAL::Polygon_mesh_processing::Corefinement::INTERSECTION] = OSM(&operation);
    CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
    return *output[1];
  }
  if (this->Mode == stkCGALBoolean3DMesher::Modes::TM1_MINUS_TM2)
  {

    output[CGAL::Polygon_mesh_processing::Corefinement::TM1_MINUS_TM2] = OSM(&operation);
    CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
    return *output[2];
  }
  if (this->Mode == stkCGALBoolean3DMesher::Modes::TM2_MINUS_TM1)
  {
    output[CGAL::Polygon_mesh_processing::Corefinement::TM2_MINUS_TM1] = OSM(&operation);
    CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
    return *output[3];
  }

  if (this->ComputeSurfaceIntersection)
  {
    vtkWarningMacro("Surface Intersections are not implemented yet.");
  }

  return nullptr;
}
