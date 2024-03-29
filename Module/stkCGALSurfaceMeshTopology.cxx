#include "stkCGALSurfaceMeshTopology.h"

#include <CGAL/Curves_on_surface_topology.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <vtkAppendPolyData.h>
#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkContourLoopExtraction.h>
#include <vtkDoubleArray.h>
#include <vtkIdTypeArray.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPointSet.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkStaticPointLocator.h>

#include "stkCGALUtilities.h"

vtkStandardNewMacro(stkCGALSurfaceMeshTopology);

typedef CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> Surface_Mesh;
typedef Surface_Mesh::Vertex_index Vertex_Index;
typedef Surface_Mesh::Halfedge_index Halfedge_Index;
typedef CGAL::Surface_mesh_topology::Path_on_surface<Surface_Mesh> Path;

static vtkNew<vtkPolyData> path_to_polydata(const Path& path, const Surface_Mesh& mesh)
{
  vtkNew<vtkPolyData> cycle;
  vtkNew<vtkPoints> cyclePoints;
  vtkNew<vtkIdList> pointIds;
  pointIds->SetNumberOfIds(path.length() + 1);

  for (auto i = 0; i < path.length(); i++)
  {
    Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
    double cyclePoint[3] = { 0.0 };
    cyclePoint[0] = CGAL::to_double(mesh.point(vtx).x());
    cyclePoint[1] = CGAL::to_double(mesh.point(vtx).y());
    cyclePoint[2] = CGAL::to_double(mesh.point(vtx).z());
    cyclePoints->InsertNextPoint(cyclePoint);
    pointIds->SetId(i, i);
  }

  // close loop
  pointIds->SetId(path.length(), 0);

  cycle->SetPoints(cyclePoints);
  cycle->Allocate();
  cycle->InsertNextCell(VTK_POLY_LINE, pointIds);

  return cycle;
}

static void remove_seen_vtx(
  const Path& path, const Surface_Mesh& mesh, std::unordered_set<Vertex_Index>& vertices)
{
  for (auto i = 0; i < path.length(); i++)
  {
    Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
    vertices.erase(vtx);
  }
}

//----------------------------------------------------------------------------
int stkCGALSurfaceMeshTopology::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkPolyData* inMesh = this->GetInputMesh();

  vtkPolyData* outMesh = vtkPolyData::GetData(outputVector, 0);

  if (inMesh == nullptr || inMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Mesh is empty.");
    return 0;
  }

  if (inMesh->GetNumberOfCells() == 0)
  {
    vtkErrorMacro("Mesh is does not contain any Cell.");
    return 0;
  }

  auto maskedVertexArray = inMesh->GetPointData()->GetArray(this->BasePointMaskArrayName.c_str());

  if (maskedVertexArray == nullptr)
  {
    vtkErrorMacro("Vertex To Check Point Mask Array is not selected");
    return 0;
  }

  vtkNew<vtkPoints> maskedPoints;
  vtkNew<vtkPointSet> basePoints;
  for (vtkIdType pointID = 0; pointID < maskedVertexArray->GetNumberOfTuples(); pointID++)
  {
    if (maskedVertexArray->GetTuple1(pointID) > 0)
    {
      maskedPoints->InsertNextPoint(inMesh->GetPoint(pointID));
    }
  }
  basePoints->SetPoints(maskedPoints);
  vtkNew<vtkStaticPointLocator> pointLocator;
  pointLocator->SetDataSet(basePoints);
  pointLocator->BuildLocator();

  if (basePoints->GetNumberOfPoints() <= 0)
  {
    vtkErrorMacro("There is no vertex to check in select Point mask");
    return 0;
  }

  if (outMesh == nullptr)
  {
    vtkErrorMacro("Out mesh is empty.");
    return 0;
  }

  // -------------------------------
  Surface_Mesh cMesh;
  bool ok = stkCGALUtilities::vtkPolyDataToPolygonMesh(inMesh, cMesh);
  if (!ok)
  {
    vtkErrorMacro("CGAL mesh conversion error.");
    return 0;
  }

  CGAL::Surface_mesh_topology::Curves_on_surface_topology<Surface_Mesh> curves(cMesh);
  CGAL::Surface_mesh_topology::Euclidean_length_weight_functor<Surface_Mesh> wf(cMesh);

  std::unordered_set<Vertex_Index> vtx_to_check;

  for (Vertex_Index vtx : cMesh.vertices())
  {
    // Point to check
    double meshPoint[3] = { 0.0 };
    double locatedIDCoords[3] = { 0.0 };
    meshPoint[0] = CGAL::to_double(cMesh.point(vtx).x());
    meshPoint[1] = CGAL::to_double(cMesh.point(vtx).y());
    meshPoint[2] = CGAL::to_double(cMesh.point(vtx).z());

    auto locatedID = pointLocator->FindClosestPoint(meshPoint);
    basePoints->GetPoint(locatedID, locatedIDCoords);

    double squaredTolerance = std::pow(this->ContraintSearchTolerance, 2);

    if (vtkMath::Distance2BetweenPoints(meshPoint, locatedIDCoords) < squaredTolerance)
    {
      vtx_to_check.insert(vtx);
    }
  }

  vtkNew<vtkAppendPolyData> outputCycles;

  vtkNew<vtkCleanPolyData> cleanCycle;
  cleanCycle->ConvertPolysToLinesOff();
  cleanCycle->ConvertStripsToPolysOff();
  cleanCycle->ConvertLinesToPointsOff();
  cleanCycle->PointMergingOn();

  vtkNew<vtkContourLoopExtraction> loopExtract;
  loopExtract->SetOutputModeToPolylines();

  vtkIdType cycleID = 0;

  auto loopSize = vtx_to_check.size();
  int process_percentage = 1;
  this->SetProgressText("Progress");
  this->UpdateProgress(0.0);

  double progress = 0.0;

  while (!vtx_to_check.empty())
  {
    progress = (double)(loopSize - vtx_to_check.size()) / loopSize;

    if (progress > process_percentage * .1)
    {
      process_percentage++;
      this->UpdateProgress(progress);
    }

    Vertex_Index vtx = *(vtx_to_check.begin());
    vtx_to_check.erase(vtx_to_check.begin());

    Halfedge_Index he = cMesh.halfedge(vtx);

    Path path = curves.compute_shortest_non_contractible_cycle_with_base_point(he, wf);

    if (path.is_empty())
    {
      continue;
    }

    if (path.is_closed())
    {

      if (this->OptimizeForExecutionTime)
      {
        remove_seen_vtx(path, cMesh, vtx_to_check);
      }
      auto cycle = path_to_polydata(path, cMesh);

      vtkNew<vtkPolyData> nthCycle;

      if (this->ExtractSimpleCycles)
      {
        cleanCycle->SetInputData(cycle);
        cleanCycle->Update();

        loopExtract->SetInputData(cleanCycle->GetOutput());
        loopExtract->Update();

        if (loopExtract->GetOutput()->GetNumberOfPoints() <= 0)
        {
          continue;
        }
        else
        {
          nthCycle->ShallowCopy(loopExtract->GetOutput());
        }
      }
      else
      {
        nthCycle->ShallowCopy(cycle);
      }

      if (this->GenerateCycleIDs)
      {
        vtkNew<vtkIdTypeArray> cycleIDArray;
        cycleIDArray->SetNumberOfComponents(1);
        cycleIDArray->SetNumberOfTuples(1);
        cycleIDArray->SetName(this->CycleIDArrayName.c_str());

        cycleIDArray->SetTuple1(0, cycleID);
        nthCycle->GetCellData()->AddArray(cycleIDArray);
        cycleID++;
      }

      if (this->CalculateCycleLength)
      {
        vtkNew<vtkDoubleArray> cycleLengthArray;
        cycleLengthArray->SetNumberOfComponents(1);
        cycleLengthArray->SetNumberOfTuples(1);
        cycleLengthArray->SetName(this->CycleLengthArrayName.c_str());

        double cycleLength = 0.0;
        double tmpPt0[3] = { 0.0 }, tmpPt1[3] = { 0.0 };

        vtkNew<vtkIdList> ptIdsList;
        nthCycle->GetLines()->GetCell(0, ptIdsList);
        nthCycle->GetPoint(ptIdsList->GetId(0), tmpPt0);
        for (vtkIdType pointId = 1; pointId < ptIdsList->GetNumberOfIds(); ++pointId)
        {
          nthCycle->GetPoint(ptIdsList->GetId(pointId), tmpPt1);
          cycleLength += std::sqrt(vtkMath::Distance2BetweenPoints(tmpPt0, tmpPt1));
          std::copy(tmpPt1, tmpPt1 + 3, tmpPt0);
        }

        cycleLengthArray->SetTuple1(0, cycleLength);
        nthCycle->GetCellData()->AddArray(cycleLengthArray);
      }

      outputCycles->AddInputData(nthCycle);
    }
  }

  outputCycles->Update();
  outMesh->ShallowCopy(outputCycles->GetOutput());

  return 1;
}
