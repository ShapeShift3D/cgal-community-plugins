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

//---------Module-------------------------------
#include <stkCGALUtilities.h>

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

      std::cout << "====== Region " << i << " ======" << std::endl;

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

  if (intersected_tris.size() == 0)
  {
    vtkWarningMacro("No Self-Intersections were found in the mesh. "
      << intersected_tris.size() << " pairs of triangles intersect.");
  }
  else
  {
    vtkWarningMacro("Self-Intersections were found in the mesh. "
      << intersected_tris.size() << " pairs of triangles intersect.");
  }

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

//---------------------------------------------------
int stkCGALSelfIntersectionMeasurer::ExecuteRepairSelfIntersect(
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

  if (intersecting)
  {
    vtkWarningMacro("Found some Self-Intersections.Trying to Repair Self-Intersections");

    vtkNew<vtkIdTypeArray> OrginalIDArray;

    // Most of the code below is replicated from the experimental remove_self_intersections method
    // PMP::experimental::remove_self_intersections(surfaceMesh);

    auto face_range = CGAL::faces(surfaceMesh);
    std::set<face_descriptor> working_face_range(face_range.begin(), face_range.end());

    std::pair<bool, bool> result_pair;

    // Look for self-intersections in the mesh and remove them
    bool all_fixed = true; // indicates if the filling of all created holes went fine
    bool topology_issue =
      false; // indicates if some boundary cycles of edges are blocking the fixing
    std::set<face_descriptor> faces_to_remove;

    int step = -1;
    int maxStep = this->MaxStep;
    bool preserve_genus = this->PreserveGenus;
    bool only_treat_self_intersections_locally = this->OnlyTreatSelfIntersectionsLocally;
    const double strong_dihedral_angle = this->StrongDihedralAngle;
    const double weak_dihedral_angle = this->WeakDihedralAngle;

    typedef CGAL::Named_function_parameters<bool, CGAL::internal_np::all_default_t,
      CGAL::internal_np::No_property>
      NamedParameters;

    NamedParameters np;

    typedef typename CGAL::GetVertexPointMap<Surface_Mesh, NamedParameters>::type VertexPointMap;

    VertexPointMap vpm = CGAL::parameters::choose_parameter(
      CGAL::parameters::get_parameter(np, CGAL::internal_np::vertex_point),
      CGAL::get_property_map(CGAL::vertex_point, surfaceMesh));

    typedef typename CGAL::GetGeomTraits<Surface_Mesh, NamedParameters>::type GeomTraits;

    GeomTraits gt = CGAL::parameters::choose_parameter<GeomTraits>(
      CGAL::parameters::get_parameter(np, CGAL::internal_np::geom_traits));

    if (!this->PreserveGenus)
      PMP::duplicate_non_manifold_vertices(surfaceMesh, np);

    while (++step < maxStep)
    {
      if (faces_to_remove
            .empty()) // the previous round might have been blocked due to topological constraints
      {
        typedef std::pair<face_descriptor, face_descriptor> Face_pair;

        std::vector<Face_pair> intersected_tris;
        PMP::self_intersections(surfaceMesh, std::back_inserter(intersected_tris));

        for (const Face_pair& fp : intersected_tris)
        {
          faces_to_remove.insert(fp.first);
          faces_to_remove.insert(fp.second);
        }
      }

      if (faces_to_remove.empty() && all_fixed)
      {
        break;
      }

      result_pair = PMP::internal::remove_self_intersections_one_step(faces_to_remove,
        working_face_range, surfaceMesh, step, preserve_genus,
        only_treat_self_intersections_locally, strong_dihedral_angle, weak_dihedral_angle, vpm, gt);

      all_fixed = result_pair.first;
      topology_issue = result_pair.second;
    }

    vtkWarningMacro(<< "Done Repairing. Measuring Self-intersection in the repaired mesh");

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
        else
        {
          OrginalIDArray->SetTuple1(pointID_repaired_mesh, -1);
        }
      }
      polyDataOut->GetPointData()->AddArray(OrginalIDArray);
    }
  }
  else
  {
    polyDataOut->ShallowCopy(polyDataIn);
  }

  return 1;
}
