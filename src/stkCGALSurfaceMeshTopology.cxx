#include <stdlib.h>

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

#include <vtkStaticPointLocator.h>
#include <vtkMath.h>

#include "stkCGALSurfaceMeshTopology.h"

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
}

static vtkNew<vtkPolyLine> path_to_polyline(const Path &path, const Surface_Mesh &mesh) {
    vtkNew<vtkPolyLine> pline;
    auto pids = pline->GetPointIds();
    pids->SetNumberOfIds(path.length() + 1);

    for(auto i=0; i < path.length(); i++) {
        Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
        pids->SetId(i, vtx);
    }

    // close loop
    pids->SetId(path.length(), pids->GetId(0));

    return pline;
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

    // TODO : Add a Debug MultiBlock
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

        // TODO : Expose Tol as a advanced property 
        auto locatedID_T = pointLocator->FindClosestPoint(meshPoint);
        thresholdVertexToCheck->GetOutput()->GetPoint(locatedID_T, locatedID_Coords);

        if (vtkMath::Distance2BetweenPoints(meshPoint, locatedID_Coords) < this->SquaredContraintSearchTolerance)
        {
        vtx_to_check.insert(vtx);
        }
    }

    vtkNew<vtkCellArray> cells;
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
        // Check that path passes the vtx_to_check
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
        auto pline = path_to_polyline(path, cMesh);

        // TODO : Put this in Debug 
        cells->InsertNextCell(pline);

        // TODO : We have polyline at this point
        // TODO : Apply Contour Loop Extract on the Polyline
        // TODO : Calculate Distance of a polyline 
        // TODO : Keep Original Point IDs on the Polyline in an Array
        // TODO : Sort Polyling based on their length. Can possibilty use Length Criteria in PolyDataCellGroupClassfier? 
        // TODO : Append Polyline in the Output 
        }
    }

    // TODO : This can go in Debug MutliBlock
    outMesh->SetPoints(inMesh->GetPoints());
    outMesh->SetLines(cells);

    return 1;
}
