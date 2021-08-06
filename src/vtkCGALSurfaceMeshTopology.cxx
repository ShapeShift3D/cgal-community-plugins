#include <CGAL/Curves_on_surface_topology.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Mesh_3/io_signature.h>
#include <CGAL/Surface_mesh.h>

#include <vtkCGALUtilities.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>

#include "vtkCGALSurfaceMeshTopology.h"

vtkStandardNewMacro(vtkCGALSurfaceMeshTopology);

typedef CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> Surface_Mesh;
typedef Surface_Mesh::Vertex_index Vertex_Index;
typedef std::pair<Vertex_Index, Vertex_Index> Vertex_Pair;

static bool sort_vertices(const std::vector<Vertex_Pair> &vertex_in, std::vector<Vertex_Index> &vertex_out);

//----------------------------------------------------------------------------
vtkCGALSurfaceMeshTopology::vtkCGALSurfaceMeshTopology()
{
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(1);
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
    bool ok = vtkCGALUtilities::vtkPolyDataToPolygonMesh(inMesh, cMesh);
    if(!ok) {
        vtkErrorMacro("CGAL mesh conversion error.");
        return 0;
    }        
        
    
    CGAL::Surface_mesh_topology::Curves_on_surface_topology<Surface_Mesh> curves(cMesh, true);

    CGAL::Surface_mesh_topology::Euclidean_length_weight_functor<Surface_Mesh> wf(cMesh);
    auto path = curves.compute_shortest_non_contractible_cycle(wf, true);

    vtkWarningMacro("path is " << path.length() << " " << path.is_empty());

    if (!path.length()) {
        vtkErrorMacro("Un-holy mesh detected.");
        return 0;
    }

    // 4 8 5
    std::vector<Vertex_Pair> vertex_in;
    std::vector<Vertex_Index> vertex_new;
    path.reverse();
    for(auto i=0; i < path.length(); i++) {
        vtkWarningMacro("path " << i << " " << (int)(path[i]));
        auto vertex1 = cMesh.vertex(cMesh.edge(path[i]), 0);
        auto vertex2 = cMesh.vertex(cMesh.edge(path[i]), 1);
        auto vertex3 = cMesh.source(path[i]);
        auto vertex4 = cMesh.target(path[i]);
        vertex_new.push_back(vertex3);
        vtkWarningMacro("tentative v: " << (int)vertex3 << ", " << (int)vertex4);
        
        vertex_in.push_back(Vertex_Pair(vertex1, vertex2));
        //auto point1 = cMesh.point(vertex1);
        //auto point2 = cMesh.point(vertex2);
        vtkWarningMacro("p: " << (int)vertex1 << " " << (int)vertex2);
        

        auto he = cMesh.halfedge(vertex3, vertex4);
        vtkWarningMacro("he: " << (int)he);

        vtkWarningMacro("");
    }
    std::vector<Vertex_Index> vertex_out;
    ok = sort_vertices(vertex_in, vertex_out);
    if(!ok) {
        vtkErrorMacro("CGAL returned a malformed cycle.");
        return 0;
    }

    vtkNew<vtkPolyLine> pline;
    auto pids = pline->GetPointIds();
    pids->SetNumberOfIds(vertex_out.size() + 1);
    for(auto i=0; i < vertex_out.size(); i++) {
        pids->SetId(i, vertex_out[i]);
    }
    // close loop
    pids->SetId(vertex_out.size(), vertex_out[0]);

    vtkNew<vtkCellArray> cells;
    cells->InsertNextCell(pline);

    for(auto i=0; i < vertex_out.size(); i++) {
        vtkWarningMacro("v: " << (int)vertex_out[i]);
    }

    outMesh->SetPoints(inMesh->GetPoints());
    outMesh->SetLines(cells);

    return 1;
}

static bool sort_vertices(const std::vector<Vertex_Pair> &vertex_in, std::vector<Vertex_Index> &vertex_out)
{
    std::vector<Vertex_Pair> vertex_tmp;
    vertex_tmp.push_back(vertex_in.front());

    while(vertex_tmp.size() != vertex_in.size()) {
        auto target = vertex_tmp.back().second;
        // brute force search
        bool found=false;
        for(auto i=0; i < vertex_in.size(); i++) {
            if(vertex_in[i].first == target) {
                found=true;
                vertex_tmp.push_back(vertex_in[i]);
                break;
            }
        }
        if(!found) {
            // Could not find chain
            return false;
        }
    }

    if(vertex_tmp.front().first != vertex_tmp.back().second) {
        // Non-cycle
        return false;
    }

    for(auto i=0; i < vertex_tmp.size(); i++) {
        vertex_out.push_back(vertex_tmp[i].first);
    }

    return true;
}
