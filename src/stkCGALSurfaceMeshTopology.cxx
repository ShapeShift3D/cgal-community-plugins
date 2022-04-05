#include "stkCGALSurfaceMeshTopology.h"

#include <CGAL/Curves_on_surface_topology.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Mesh_3/io_signature.h>
#include <CGAL/Surface_mesh.h>

#include <stkCGALUtilities.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkThresholdPoints.h>
#include <vtkAppendPolyData.h>
#include <vtkStaticPointLocator.h>
#include <vtkMath.h>
#include <vtkContourLoopExtraction.h>
#include <vtkCleanPolyData.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkDoubleArray.h>

vtkStandardNewMacro(stkCGALSurfaceMeshTopology);

typedef CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> Surface_Mesh;
typedef Surface_Mesh::Vertex_index Vertex_Index;
typedef Surface_Mesh::Halfedge_index Halfedge_Index;
typedef Surface_Mesh::Edge_index Edge_Index;
typedef CGAL::Surface_mesh_topology::Path_on_surface<Surface_Mesh> Path;

//----------------------------------------------------------------------------
stkCGALSurfaceMeshTopology::stkCGALSurfaceMeshTopology()
{
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(1);
    this->VertexToCheckPointMaskName="VertexToCheckMask";
    this->SquaredContraintSearchTolerance= 1e-6;
    this->ExtractSimpleCycles = true;
    this->GenerateCycleIDs = true;
    this->CycleIDArrayName = "Cycle ID";
    this->CalculateCycleLength = true;
    this->CycleLengthArrayName = "Cycle Length";
}

static vtkNew<vtkPolyData> path_to_polydata(const Path &path, const Surface_Mesh &mesh) 
{
    vtkNew<vtkPolyData> cycle;
    vtkNew<vtkPoints> cyclePoints;
    vtkNew<vtkIdList> pointIds;
    pointIds->SetNumberOfIds(path.length() + 1);

    for(auto i=0; i < path.length(); i++) {
        Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
        double cyclePoint[3] = {0.0};
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
    cycle->InsertNextCell(VTK_POLY_LINE,pointIds);

    return cycle;
}

static void remove_seen_vtx(const Path &path,
                              const Surface_Mesh &mesh,
                              std::unordered_set<Vertex_Index> &vertices) {
    for(auto i=0; i < path.length(); i++) {
        Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
        vertices.erase(vtx);
    }
}


//----------------------------------------------------------------------------
int stkCGALSurfaceMeshTopology::RequestData(vtkInformation* vtkNotUsed(request),
                                            vtkInformationVector** inputVector,
                                            vtkInformationVector* outputVector)
{
    vtkPolyData *inMesh = vtkPolyData::GetData(inputVector[0], 0);
    vtkPolyData *outMesh = vtkPolyData::GetData(outputVector, 0);

    if (inMesh == nullptr || inMesh->GetNumberOfPoints() == 0) {
        vtkErrorMacro("Mesh is empty.");
        return 0;
    }

    auto maskedVertexArray = inMesh->GetPointData()->GetArray(this->VertexToCheckPointMaskName.c_str());

    if (maskedVertexArray == nullptr)
    {
        vtkErrorMacro("Vertex To Check Point Mask Array is not selected");
        return 0;
    }

    if (outMesh == nullptr) {
        vtkErrorMacro("Out mesh is empty.");
        return 0;
    }
    // -------------------------------

    Surface_Mesh cMesh;
    bool ok = stkCGALUtilities::vtkPolyDataToPolygonMesh(inMesh, cMesh);
    if(!ok) {
        vtkErrorMacro("CGAL mesh conversion error.");
        return 0;
    }

    CGAL::Surface_mesh_topology::Curves_on_surface_topology<Surface_Mesh> curves(cMesh);
    CGAL::Surface_mesh_topology::Euclidean_length_weight_functor<Surface_Mesh> wf(cMesh);

    std::unordered_set<Vertex_Index> vtx_to_check;

    auto thresholdVertexToCheck = vtkSmartPointer<vtkThresholdPoints>::New();
    thresholdVertexToCheck->SetInputData(inMesh);
    thresholdVertexToCheck->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, this->VertexToCheckPointMaskName.c_str());
    thresholdVertexToCheck->ThresholdByUpper(0.00001);
    thresholdVertexToCheck->Update();

    if (thresholdVertexToCheck->GetOutput()->GetNumberOfPoints() <= 0)
    {
        vtkErrorMacro("There is no vertex to check in select Point mask");
        return 0;
    }

    auto pointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();
    pointLocator->SetDataSet(thresholdVertexToCheck->GetOutput()); 
    pointLocator->BuildLocator();

    for(Vertex_Index vtx : cMesh.vertices()) { 
        // Point to check 
        double meshPoint[3] = {0.0};
        double locatedID_Coords[3] = {0.0};
        meshPoint[0] = CGAL::to_double(cMesh.point(vtx).x());
        meshPoint[1] = CGAL::to_double(cMesh.point(vtx).y());
        meshPoint[2] = CGAL::to_double(cMesh.point(vtx).z());
 
        auto locatedID_T = pointLocator->FindClosestPoint(meshPoint);
        thresholdVertexToCheck->GetOutput()->GetPoint(locatedID_T, locatedID_Coords);

        if (vtkMath::Distance2BetweenPoints(meshPoint, locatedID_Coords) < this->SquaredContraintSearchTolerance)
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

    vtkIdType cycleID = 0 ;

    vtkNew<vtkIdTypeArray> cycleIDArray;
    cycleIDArray->SetNumberOfComponents(1);
    cycleIDArray->SetNumberOfValues(1);
    cycleIDArray->SetName(this->CycleIDArrayName.c_str());

    vtkNew<vtkDoubleArray> cycleLengthArray;
    cycleLengthArray->SetNumberOfComponents(1);
    cycleLengthArray->SetNumberOfValues(1);
    cycleLengthArray->SetName(this->CycleLengthArrayName.c_str());

    while(!vtx_to_check.empty()) {
        Vertex_Index vtx = *(vtx_to_check.begin());
        vtx_to_check.erase(vtx_to_check.begin());

        Halfedge_Index he = cMesh.halfedge(vtx);
        // TODO : Check if its valid to use the overload of the function
        Path path = curves.compute_shortest_non_contractible_cycle_with_base_point(he, wf);

        if(path.is_empty()) {
            continue;
        }

        // TODO : Perform this under Advanced Boolean Property 
        // Check that path passes through the vtx_to_check
        bool vertex_in_path = false;
        for(auto i=0; i < path.length(); i++) 
        {
          Vertex_Index vtx_cycle= path.get_flip()[i] ? cMesh.target(path[i]) : cMesh.source(path[i]);
          vertex_in_path = (vtx_cycle == vtx) ?  true : false;
          if (vertex_in_path == true)
          {
              break;
          }
        }

        if (vertex_in_path == false)
        {
            continue;
        }

        if (path.is_closed())
        {
        remove_seen_vtx(path, cMesh, vtx_to_check); // TODO : This could be made optional under advanced property, increase coverage and execution counts

        auto cycle = path_to_polydata(path,cMesh);

        vtkNew<vtkPolyData> nthCycle;

        if(this->ExtractSimpleCycles)
        {
        cleanCycle->SetInputData(cycle);
        cleanCycle->Update();

        loopExtract->SetInputData(cleanCycle->GetOutput());
        loopExtract->Update();

        if(loopExtract->GetOutput()->GetNumberOfPoints() <= 0)
        {
         continue;
        }
        else{
        nthCycle->ShallowCopy(loopExtract->GetOutput());
        }
        }
        else
        {
            nthCycle->ShallowCopy(cycle);
        }

        if(this->GenerateCycleIDs)
        {
        cycleIDArray->SetTuple1(0,cycleID);
        nthCycle->GetCellData()->AddArray(cycleIDArray);
        cycleID++;
        }

        if(this->CalculateCycleLength)
        {
        double cycleLength = 0.0;
        double tmpPt0[3] = { 0.0 }, tmpPt1[3] = { 0.0 };

        vtkSmartPointer<vtkIdList> ptIdsList = vtkSmartPointer<vtkIdList>::New();
        nthCycle->GetLines()->GetCell(0,ptIdsList);
        nthCycle->GetPoint(ptIdsList->GetId(0), tmpPt0);
        for (vtkIdType pointId = 1; pointId < ptIdsList->GetNumberOfIds(); ++pointId)
        {
          nthCycle->GetPoint(ptIdsList->GetId(pointId), tmpPt1);
          cycleLength += std::sqrt(vtkMath::Distance2BetweenPoints(tmpPt0, tmpPt1));
          std::copy(tmpPt1, tmpPt1 + 3, tmpPt0);
        }

        cycleLengthArray->SetTuple1(0,cycleLength);
        nthCycle->GetCellData()->AddArray(cycleLengthArray);
        }

        outputCycles->AddInputData(nthCycle);
        }
    }

    outputCycles->Update();
    outMesh->ShallowCopy(outputCycles->GetOutput());

    return 1;
}
