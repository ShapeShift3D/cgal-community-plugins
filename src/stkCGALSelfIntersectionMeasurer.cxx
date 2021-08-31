#include "stkCGALSelfIntersectionMeasurer.h"

//---------VTK----------------------------------
#include <vtkCommand.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>

#include <vtkCellData.h>
#include <vtkIntArray.h>

#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataConnectivityFilter.h>

//---------CGAL---------------------------------
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

//---------Module-------------------------------
#include <stkCGALUtilities.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::face_descriptor face_descriptor;

vtkStandardNewMacro(stkCGALSelfIntersectionMeasurer);

// ----------------------------------------------------------------------------
int stkCGALSelfIntersectionMeasurer::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputMesh = this->GetInputMesh();

  if (inputMesh == nullptr || inputMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("No points found in input mesh.");
    return 0;
  }

  vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  if (this->IterateByConnectivity)
  {
    // Iterate over polygon meshes by connectivity
    vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
    connectivityFilter->SetInputData(inputMesh);
    connectivityFilter->SetExtractionModeToAllRegions();
    connectivityFilter->Update();

    vtkNew<vtkCleanPolyData> cleanPolyData;
    cleanPolyData->SetInputConnection(connectivityFilter->GetOutputPort());

    int numberOfRegions = connectivityFilter->GetNumberOfExtractedRegions();

    vtkNew<vtkAppendPolyData> appendFinal;

    for (vtkIdType i = 0; i < numberOfRegions; ++i)
    {
      connectivityFilter->SetExtractionModeToSpecifiedRegions();
      connectivityFilter->InitializeSpecifiedRegionList();
      connectivityFilter->AddSpecifiedRegion(i);
      cleanPolyData->Update();

      vtkPolyData* polyData = cleanPolyData->GetOutput();

      if (polyData->GetNumberOfCells() == 0)
        continue;

      std::cout << "====== Region " << i << " ======" << std::endl;

      vtkNew<vtkPolyData> singlePoly;
      this->ExecuteSelfIntersect(polyData, singlePoly);
      appendFinal->AddInputData(singlePoly);
    }

    appendFinal->Update();

    // Copy to output
    output->ShallowCopy(appendFinal->GetOutput());
  }
  else
  {
    vtkNew<vtkPolyData> outPoly;
    this->ExecuteSelfIntersect(inputMesh, outPoly);

    // Copy to output
    output->ShallowCopy(outPoly);
  }

  return 1;
}

//---------------------------------------------------
int stkCGALSelfIntersectionMeasurer::ExecuteSelfIntersect(
  vtkPolyData* polyDataIn, vtkPolyData* polyDataOut)
{
  namespace PMP = CGAL::Polygon_mesh_processing;

  Surface_Mesh surfaceMesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(polyDataIn, surfaceMesh);

  if (!CGAL::is_triangle_mesh(surfaceMesh))
  {
    vtkErrorMacro("Mesh is not triangular.");
    return 0;
  }

  bool intersecting = PMP::does_self_intersect(
    surfaceMesh, PMP::parameters::vertex_point_map(get(CGAL::vertex_point, surfaceMesh)));

  std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
  PMP::self_intersections(surfaceMesh, std::back_inserter(intersected_tris));

  vtkNew<vtkIntArray> intersectingTrisArray;
  intersectingTrisArray->SetName(this->SelfIntersectionsArrayName.c_str());
  intersectingTrisArray->SetNumberOfComponents(1);
  intersectingTrisArray->SetNumberOfTuples(polyDataIn->GetNumberOfCells());
  intersectingTrisArray->Fill(0);

  int currentTuple = 0;

  for (vtkIdType i = 0; i < intersected_tris.size(); ++i)
  {
    currentTuple = intersectingTrisArray->GetTuple1(intersected_tris[i].first.idx());
    intersectingTrisArray->SetTuple1(intersected_tris[i].first.idx(), currentTuple + 1);

    currentTuple = intersectingTrisArray->GetTuple1(intersected_tris[i].second.idx());
    intersectingTrisArray->SetTuple1(intersected_tris[i].second.idx(), currentTuple + 1);
  }

  vtkWarningMacro("Self-Intersections were found in the mesh. "
    << intersected_tris.size() << " pairs of triangles intersect.");

  if (this->PrintSelfIntersectingPairs)
  {
    for (vtkIdType i = 0; i < intersected_tris.size(); ++i)
    {
      vtkWarningMacro("Triangle " << intersected_tris[i].first.idx() << " is intersecting with "
                                  << intersected_tris[i].second.idx() << ".");
    }
  }

  polyDataOut->DeepCopy(polyDataIn);
  polyDataOut->GetCellData()->AddArray(intersectingTrisArray);

  return 1;
}
