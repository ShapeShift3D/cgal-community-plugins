#ifndef vtkCGALUtilities_h
#define vtkCGALUtilities_h

#include "vtkObject.h"
#include <vtkCGALModule.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>

class vtkPointSet;
class vtkUnstructuredGrid;

class VTKCGAL_EXPORT vtkCGALUtilities : public vtkObject
{
public:
    static vtkCGALUtilities* New();
    vtkTypeMacro(vtkCGALUtilities, vtkObject);

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Surface_mesh<K::Point_3> SurfaceMesh0;
    typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron0;
    typedef CGAL::Simple_cartesian<double> Kernel;
    typedef CGAL::Surface_mesh<Kernel::Point_3>  SurfaceMesh1;
    typedef CGAL::Polyhedron_3<Kernel::Point_3> Polyhedron1;
    

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh0& tmesh);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron0& tmesh);
    
    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh1& tmesh);

    static bool PolygonMeshToVtkUnstructuredGrid(const SurfaceMesh0& pmesh, vtkUnstructuredGrid* usg);

protected:
    vtkCGALUtilities();
    ~vtkCGALUtilities();

private:
    vtkCGALUtilities(const vtkCGALUtilities&) = delete;
    void operator=(const vtkCGALUtilities&) = delete;
    
    template <typename KernelType, typename MeshType>
    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, MeshType& tmesh);

    template <typename VPMapType, typename PointType, typename VertexDescriptor, typename MeshType>
    static bool vtkPolyDataToPolygonMeshImpl(vtkPointSet* polyData, MeshType& tmesh);
};
#endif
