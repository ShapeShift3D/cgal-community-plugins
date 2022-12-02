#include <stkCGALRegionFairingOperator.h>

#include <stkCGALUtilities.h>

#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPointDataToCellData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkAssume.h>

#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Surface_mesh.h>

#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::vertex_descriptor Graph_Verts;

namespace PMP = CGAL::Polygon_mesh_processing;

//----------------------------------------------------------------------------
vtkStandardNewMacro(stkCGALRegionFairingOperator);

//----------------------------------------------------------------------------
int stkCGALRegionFairingOperator::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* inputMesh = vtkPolyData::GetData(inputVector[0]);
  vtkPolyData* outputMesh = vtkPolyData::GetData(outputVector, 0);

  if (inputMesh == nullptr)
  {
    vtkErrorMacro("Input Mesh is NULL");
    return 0;
  }

  if (inputMesh->GetNumberOfPoints() < 3)
  {
    vtkErrorMacro("Input Mesh contains less than 3 Points.");
    return 0;
  }

  if (inputMesh->GetNumberOfPolys() == 0)
  {
    vtkErrorMacro("Input Mesh does not contain any Polygonal Cells.");
    return 0;
  }

  auto maskArray = inputMesh->GetPointData()->GetArray(this->RegionArrayName.c_str());

  if (maskArray == nullptr)
  {
    vtkErrorMacro("Could not find Array from the provided RegionArrayName");
    return 0;
  }

  if (maskArray->GetNumberOfComponents() != 1)
  {
    vtkErrorMacro("Expected Mask Array to contain only 1 component");
    return 0;
  }

  VTK_ASSUME(maskArray->GetNumberOfComponents() == 1);

  Surface_Mesh cgalMesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, cgalMesh);

  if (!CGAL::is_triangle_mesh(cgalMesh))
  {
    vtkErrorMacro("Input Mesh is not triangular.");
    return 0;
  }

  std::vector<Graph_Verts> roi; // region of interest

  // Get the Vertices of the Patch to be faired from the Mask Array
  for (vtkIdType id = 0; id < maskArray->GetNumberOfTuples(); id++)
  {
    if (maskArray->GetTuple1(id) > 0.0)
    {
      roi.emplace_back(Graph_Verts(id));
    }
  }

  try
  {
    PMP::fair(cgalMesh, roi, PMP::parameters::fairing_continuity(this->FairingContinuity));
  }
  catch (std::exception& e)
  {
    vtkErrorMacro("CGAL Exception: " << e.what());
    return 0;
  }

  stkCGALUtilities::SurfaceMeshToPolyData(cgalMesh, outputMesh);

  // Mask Arrays to indicide faired point and cells
  if (this->GenerateFairedMaskArrays)
  {
    auto pointDataToCellData = vtkSmartPointer<vtkPointDataToCellData>::New();
    pointDataToCellData->SetInputData(inputMesh);
    pointDataToCellData->PassPointDataOff();
    pointDataToCellData->ProcessAllArraysOff();
    pointDataToCellData->CategoricalDataOff();
    pointDataToCellData->AddPointDataArray(maskArray->GetName());
    pointDataToCellData->Update();

    auto fairedCellMaskArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    fairedCellMaskArray->SetName(this->FairedCellMaskArrayName.c_str());
    fairedCellMaskArray->SetNumberOfComponents(1);
    fairedCellMaskArray->SetNumberOfTuples(inputMesh->GetNumberOfCells());
    fairedCellMaskArray->Fill(0.0);

    auto tempCellDataArray = pointDataToCellData->GetPolyDataOutput()->GetCellData()->GetArray(
      this->RegionArrayName.c_str());

    if (tempCellDataArray)
    {
      unsigned char fillValue = 1;
      for (vtkIdType id = 0; id < tempCellDataArray->GetNumberOfTuples(); id++)
      {
        if (tempCellDataArray->GetTuple1(id) > 0.0)
        {
          fairedCellMaskArray->SetValue(id, fillValue);
        }
      }
    }

    outputMesh->GetCellData()->AddArray(fairedCellMaskArray);

    auto fairedPointMaskArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    fairedPointMaskArray->DeepCopy(maskArray);
    fairedPointMaskArray->SetName(this->FairedPointMaskArrayName.c_str());

    outputMesh->GetPointData()->AddArray(fairedPointMaskArray);
  }

  if (this->PassAllArrays)
  {
    // Transfer All Point Ids Arrays
    for (int pointArrayID = 0; pointArrayID < inputMesh->GetPointData()->GetNumberOfArrays();
         pointArrayID++)
    {
      auto sourcePointArray = inputMesh->GetPointData()->GetAbstractArray(pointArrayID);

      if (this->ConsumeInputMaskArray &&
        (std::string(sourcePointArray->GetName()) == this->RegionArrayName))
      {
        continue;
      }

      outputMesh->GetPointData()->AddArray(sourcePointArray);
    }

    //  Transfer All Cells Ids Arrays
    for (int cellArrayID = 0; cellArrayID < inputMesh->GetCellData()->GetNumberOfArrays();
         cellArrayID++)
    {
      auto sourceCellArray = inputMesh->GetCellData()->GetAbstractArray(cellArrayID);
      outputMesh->GetCellData()->AddArray(sourceCellArray);
    }

    // Transfer Field Data
    outputMesh->GetFieldData()->PassData(inputMesh->GetFieldData());
  }

  return 1;
}
