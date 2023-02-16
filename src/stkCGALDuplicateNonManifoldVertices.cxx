#include "stkCGALDuplicateNonManifoldVertices.h"

//---------VTK----------------------------------
#include <vtkCellTypes.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Surface_mesh.h>

//---------Module-------------------------------
#include "stkCGALUtilities.h"

#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_Mesh;

typedef boost::graph_traits<Surface_Mesh>::vertex_descriptor vertex_descriptor;

vtkStandardNewMacro(stkCGALDuplicateNonManifoldVertices);

// ----------------------------------------------------------------------------
int stkCGALDuplicateNonManifoldVertices::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* inputMesh = vtkPolyData::GetData(inputVector[0]);

  vtkPolyData* outputMesh = vtkPolyData::GetData(outputVector, 0);

  if (inputMesh == nullptr)
  {
    vtkErrorMacro("Input Surface Mesh is NULL");
    return 0;
  }

  if (inputMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Surface Mesh contains 0 Points");
    return 0;
  }

  if (inputMesh->GetNumberOfCells() == 0)
  {
    vtkErrorMacro("Input Surface Mesh contains 0 Cells");
    return 0;
  }

  auto cellsType = vtkSmartPointer<vtkCellTypes>::New();
  inputMesh->GetCellTypes(cellsType);
  int numCellTypes = cellsType->GetNumberOfTypes();

  if (numCellTypes > 1 || numCellTypes == 0)
  {
    vtkErrorMacro("Invalid Input Surface. It should consist of only Triangle Cells");
    return 0;
  }
  else if (cellsType->GetCellType(0) != VTK_TRIANGLE)
  {
    vtkErrorMacro(
      "Input Surface must be a Triangular Surface Mesh (PolyData with only Triangle Cells)");
    return 0;
  }

  Surface_Mesh mesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, mesh);

  CGAL::Polygon_mesh_processing::duplicate_non_manifold_vertices(mesh);

  stkCGALUtilities::SurfaceMeshToPolyData(mesh, outputMesh);

  if (this->CleanOutput)
  {
    if (this->CleanPolyDataAlgorithm != nullptr)
    {
      this->CleanPolyDataAlgorithm->SetInputData(outputMesh);
      this->CleanPolyDataAlgorithm->Update();

      outputMesh->ShallowCopy(this->CleanPolyDataAlgorithm->GetOutput());
    }
  }

  return 1;
}
