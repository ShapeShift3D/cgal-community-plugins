#include "stkCGALSelfIntersectionMeasurer.h"

//---------VTK----------------------------------
#include <vtkCommand.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>

#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>

#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkMath.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkStaticPointLocator.h>

//---------CGAL---------------------------------
#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <CGAL/boost/graph/helpers.h>

//---------Module-------------------------------
#include <stkCGALUtilities.h>

#include <set>
#include <utility>
#include <vector>

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

      if (this->Verbose)
      {
        std::cout << "====== Region " << i << " ======" << std::endl;
      }

      vtkNew<vtkPolyData> singlePoly;
      vtkNew<vtkPolyData> tempPoly;

      if (this->RepairSelfIntersections)
      {
        this->ExecuteRepairSelfIntersect(polyData, singlePoly);
        tempPoly->ShallowCopy(singlePoly);
      }
      else
      {
        tempPoly->ShallowCopy(polyData);
      }

      this->ExecuteSelfIntersect(tempPoly, singlePoly);

      appendFinal->AddInputData(singlePoly);
    }

    appendFinal->Update();

    // Copy to output
    output->ShallowCopy(appendFinal->GetOutput());
  }
  else
  {
    vtkNew<vtkPolyData> outPoly;

    vtkNew<vtkPolyData> tempPoly;

    if (this->RepairSelfIntersections)
    {
      this->ExecuteRepairSelfIntersect(inputMesh, outPoly);
      tempPoly->ShallowCopy(outPoly);
    }
    else
    {
      tempPoly->ShallowCopy(inputMesh);
    }

    this->ExecuteSelfIntersect(tempPoly, outPoly);

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

  vtkNew<vtkIdTypeArray> cellOriginalIdsArray;
  cellOriginalIdsArray->SetName("OriginalIds");

  vtkNew<vtkIntArray> nullFaceMaskArray;
  nullFaceMaskArray->SetName(this->NullFaceMaskArrayName.c_str());

  Surface_Mesh surfaceMesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(
    polyDataIn, surfaceMesh, cellOriginalIdsArray, nullFaceMaskArray);

  if (!CGAL::is_triangle_mesh(surfaceMesh))
  {
    vtkErrorMacro("Mesh is not triangular.");
    return 0;
  }

  std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
  PMP::self_intersections(surfaceMesh, std::back_inserter(intersected_tris));

  vtkNew<vtkIntArray> intersectingTrisArray;
  intersectingTrisArray->SetName(this->SelfIntersectionsArrayName.c_str());
  intersectingTrisArray->SetNumberOfComponents(1);
  intersectingTrisArray->SetNumberOfTuples(polyDataIn->GetNumberOfCells());
  intersectingTrisArray->Fill(0);

  int currentTuple = 0;
  int originalId = 0;

  for (auto i = 0; i < intersected_tris.size(); ++i)
  {
    originalId = cellOriginalIdsArray->GetTuple1(intersected_tris[i].first.idx());
    currentTuple = intersectingTrisArray->GetTuple1(originalId);
    intersectingTrisArray->SetTuple1(originalId, currentTuple + 1);

    originalId = cellOriginalIdsArray->GetTuple1(intersected_tris[i].second.idx());
    currentTuple = intersectingTrisArray->GetTuple1(originalId);
    intersectingTrisArray->SetTuple1(originalId, currentTuple + 1);
  }

  if (intersected_tris.size() == 0)
  {
    if (this->Verbose)
    {
      vtkWarningMacro("No Self-Intersections were found in the mesh. "
        << intersected_tris.size() << " pairs of triangles intersect.");
    }
  }
  else
  {
    if (this->Verbose)
    {
      vtkWarningMacro("Self-Intersections were found in the mesh. "
        << intersected_tris.size() << " pairs of triangles intersect.");
    }
  }

  if (this->PrintSelfIntersectingPairs)
  {
    for (auto i = 0; i < intersected_tris.size(); ++i)
    {
      if (this->Verbose)
      {
        vtkWarningMacro("Triangle " << intersected_tris[i].first.idx() << " is intersecting with "
                                    << intersected_tris[i].second.idx() << ".");
      }
    }
  }

  polyDataOut->DeepCopy(polyDataIn);
  polyDataOut->GetCellData()->AddArray(intersectingTrisArray);

  if (this->OutputNullFaceMaskArray)
  {
    polyDataOut->GetCellData()->AddArray(nullFaceMaskArray);
  }

  return 1;
}

//---------------------------------------------------
int stkCGALSelfIntersectionMeasurer::ExecuteRepairSelfIntersect(
  vtkPolyData* polyDataIn, vtkPolyData* polyDataOut)
{
  namespace PMP = CGAL::Polygon_mesh_processing;

  Surface_Mesh surfaceMesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(polyDataIn, surfaceMesh);

  polyDataOut->ShallowCopy(polyDataIn);

  if (!CGAL::is_triangle_mesh(surfaceMesh))
  {
    vtkErrorMacro("Mesh is not triangular.");
    return 0;
  }

  if (!CGAL::is_valid_polygon_mesh(surfaceMesh))
  {
    // checks the integrity of graph.
    // graph is valid if it is a valid FaceListGraph and it has distinct faces on each side of an
    // edge
    vtkErrorMacro("Mesh is not a Valid Mesh. It doesn't have Valid FaceListGraph or doesn'has "
                  "distinct faces on each side of an edge");
    return 0;
  }

  bool intersecting = PMP::does_self_intersect(
    surfaceMesh, PMP::parameters::vertex_point_map(get(CGAL::vertex_point, surfaceMesh)));

  if (intersecting)
  {
    if (this->Verbose)
    {
      vtkWarningMacro("Found some Self-Intersections.Trying to Repair Self-Intersections");
    }
    vtkNew<vtkIdTypeArray> OrginalIDArray;

    // Unused for now. There will be Named parameter for this once repair_self_intersection is
    // included in PMP package
    const double weak_dihedral_angle = this->WeakDihedralAngle;

    auto parameters = PMP::parameters::number_of_iterations(this->MaxStep)
                        .preserve_genus(this->PreserveGenus)
                        .apply_per_connected_component(this->OnlyTreatSelfIntersectionsLocally)
                        .with_dihedral_angle(this->StrongDihedralAngle);

    // We need to remove the experimental namespace once the feature is included in PMP package
    try
    {
      PMP::experimental::remove_self_intersections(
        CGAL::faces(surfaceMesh), surfaceMesh, parameters);
    }
    catch (const std::exception& e)
    {
      if (this->Verbose)
      {
        vtkWarningMacro(<< "Failed to Repair Self-Intersections");
        std::cerr << e.what() << '\n';
      }
    }

    if (this->Verbose)
    {
      vtkWarningMacro(<< "Done Repairing. Measuring Self-intersection in the repaired mesh");
    }

    stkCGALUtilities::SurfaceMeshToPolyData(surfaceMesh, polyDataOut);

    if (this->GenerateOldIDArray)
    {
      auto pointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();
      pointLocator->SetDataSet(polyDataIn);
      pointLocator->BuildLocator();

      OrginalIDArray->SetName((this->OldIDArrayName).c_str());
      OrginalIDArray->SetNumberOfComponents(1);
      OrginalIDArray->SetNumberOfTuples(polyDataOut->GetNumberOfPoints());
      OrginalIDArray->Fill(-1);

      // map point from irepaired mesh to input mesh and store points Ids in a pair
      for (int pointID_repaired_mesh = 0; pointID_repaired_mesh < polyDataOut->GetNumberOfPoints();
           pointID_repaired_mesh++)
      {
        double pointID_repaired_coords[3] = { 0.0 };
        double locatedID_coords[3] = { 0.0 };

        polyDataOut->GetPoint(pointID_repaired_mesh, pointID_repaired_coords);
        auto locatedID_input_mesh = pointLocator->FindClosestPoint(pointID_repaired_coords);

        polyDataIn->GetPoint(locatedID_input_mesh, locatedID_coords);

        if (vtkMath::Distance2BetweenPoints(pointID_repaired_coords, locatedID_coords) == 0.0)
        {
          OrginalIDArray->SetTuple1(pointID_repaired_mesh, locatedID_input_mesh);
        }
      }
      polyDataOut->GetPointData()->AddArray(OrginalIDArray);
    }
  }

  return 1;
}
