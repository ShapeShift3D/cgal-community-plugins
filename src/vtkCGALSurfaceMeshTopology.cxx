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
        
    CGAL::Surface_mesh_topology::Curves_on_surface_topology<Surface_Mesh> curves(cMesh);
    CGAL::Surface_mesh_topology::Euclidean_length_weight_functor<Surface_Mesh> wf(cMesh);

    auto path = curves.compute_shortest_non_contractible_cycle(wf);

    if (path.is_empty()) {
        vtkErrorMacro("Un-holy mesh detected.");
        return 0;
    }

    vtkNew<vtkPolyLine> pline;
    auto pids = pline->GetPointIds();
    pids->SetNumberOfIds(path.length() + 1);

    for(auto i=0; i < path.length(); i++) {
        Vertex_Index vtx;
        if(path.get_flip()[i]) {
            vtx = cMesh.target(path[i]);
        } else {
            vtx = cMesh.source(path[i]);
        }
        pids->SetId(i, vtx);
    }

    // close loop
    pids->SetId(path.length(), pids->GetId(0));

    vtkNew<vtkCellArray> cells;
    cells->InsertNextCell(pline);

    outMesh->SetPoints(inMesh->GetPoints());
    outMesh->SetLines(cells);

    return 1;
}
