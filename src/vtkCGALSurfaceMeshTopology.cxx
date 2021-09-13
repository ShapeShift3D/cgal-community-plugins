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

#include "vtkCGALSurfaceMeshTopology.h"

vtkStandardNewMacro(vtkCGALSurfaceMeshTopology);

typedef CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> Surface_Mesh;
typedef Surface_Mesh::Vertex_index Vertex_Index;
typedef Surface_Mesh::Halfedge_index Halfedge_Index;
typedef Surface_Mesh::Edge_index Edge_Index;
typedef CGAL::Surface_mesh_topology::Path_on_surface<Surface_Mesh> Path;

//----------------------------------------------------------------------------
vtkCGALSurfaceMeshTopology::vtkCGALSurfaceMeshTopology()
{
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(1);
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
    std::cerr << "path len:" << path.length() << std::endl;
    for(auto i=0; i < path.length(); i++) {
        Vertex_Index vtx = path.get_flip()[i] ? mesh.target(path[i]) : mesh.source(path[i]);
        std::cerr << "vtx:" << vtx << std::endl;
        vertices.erase(vtx);
    }
}


//----------------------------------------------------------------------------
int vtkCGALSurfaceMeshTopology::RequestData(vtkInformation* vtkNotUsed(request),
                                            vtkInformationVector** inputVector,
                                            vtkInformationVector* outputVector)
{
    vtkPolyData *inMesh = vtkPolyData::GetData(inputVector[0], 0);
    vtkPolyData *outMesh = vtkPolyData::GetData(outputVector, 0);

    if (inMesh == nullptr || inMesh->GetNumberOfPoints() == 0) {
        vtkErrorMacro("Mesh is empty.");
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

    for(Vertex_Index vtx : cMesh.vertices()) { 
        vtx_to_check.insert(vtx);
        //std::cerr << "edge:" << edge << std::endl;
    }
    std::cerr << "vertices:" << vtx_to_check.size() << std::endl;

    vtkNew<vtkCellArray> cells;
    auto nbiters=0;
    
    while(!vtx_to_check.empty()) {
        nbiters++;
        Vertex_Index vtx = *(vtx_to_check.begin());
        vtx_to_check.erase(vtx_to_check.begin());
        std::cerr << "\nsource vertex: " << vtx << std::endl;

        Halfedge_Index he = cMesh.halfedge(vtx);
        std::cerr << "source he: " << he << std::endl;
        Path path = curves.compute_shortest_non_contractible_cycle_with_base_point(he, wf);
        if(path.is_empty()) {
            continue;
        }

        remove_seen_vtx(path, cMesh, vtx_to_check);
        auto pline = path_to_polyline(path, cMesh);
        cells->InsertNextCell(pline);
    }
    std::cerr << "nbiters:" << nbiters << std::endl;

    outMesh->SetPoints(inMesh->GetPoints());
    outMesh->SetLines(cells);

    return 1;
}
