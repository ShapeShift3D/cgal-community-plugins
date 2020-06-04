#ifndef vtkCGALUtilities_h
#define vtkCGALUtilities_h

#include "vtkObject.h"
#include <vtkCGALModule.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Mesh_polyhedron_3.h>

class vtkPointSet;
class vtkUnstructuredGrid;

class VTKCGAL_EXPORT vtkCGALUtilities : public vtkObject
{
public:
    static vtkCGALUtilities* New();
    vtkTypeMacro(vtkCGALUtilities, vtkObject);

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Surface_Mesh& tmesh);

    typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron& tmesh);

    static bool PolygonMeshToVtkUnstructuredGrid(const Surface_Mesh& pmesh, vtkUnstructuredGrid* usg);

protected:
    vtkCGALUtilities();
    ~vtkCGALUtilities();

private:
    vtkCGALUtilities(const vtkCGALUtilities&) = delete;
    void operator=(const vtkCGALUtilities&) = delete;

};
#endif
