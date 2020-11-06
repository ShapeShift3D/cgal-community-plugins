#ifndef vtkCGALUtilities_h
#define vtkCGALUtilities_h

#include "vtkObject.h"
#include <vtkCGALModule.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Simple_cartesian.h>

#include <utility>
#include <vector>

class vtkPointSet;
class vtkUnstructuredGrid;
class vtkPolyData;

class VTKCGAL_EXPORT vtkCGALUtilities : public vtkObject
{
public:
    static vtkCGALUtilities* New();
    vtkTypeMacro(vtkCGALUtilities, vtkObject);

    // Kernel 0
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K0;
    typedef CGAL::Surface_mesh<K0::Point_3>                     SurfaceMesh0;
    typedef CGAL::Mesh_polyhedron_3<K0>::type                   Polyhedron0;
    typedef std::pair<K0::Point_3, K0::Vector_3>                Point_with_normal0;
    typedef std::vector<Point_with_normal0>                     Pwn_vector0;

    // Kernel 1
    typedef CGAL::Simple_cartesian<double>                      K1;
    typedef CGAL::Surface_mesh<K1::Point_3>                     SurfaceMesh1;
    typedef CGAL::Polyhedron_3<K1>                              Polyhedron1;
    typedef std::pair<K1::Point_3, K1::Vector_3>                Point_with_normal1;
    typedef std::vector<Point_with_normal1>                     Pwn_vector1;

    // Kernel 2
    typedef CGAL::Exact_predicates_exact_constructions_kernel   K2;
    typedef CGAL::Surface_mesh<K2::Point_3>                     SurfaceMesh2;
    typedef CGAL::Polyhedron_3<K2>                              Polyhedron2;
    typedef std::pair<K2::Point_3, K2::Vector_3>                Point_with_normal2;
    typedef std::vector<Point_with_normal2>                     Pwn_vector2;

    // Kernel 3
    typedef CGAL::Homogeneous<CGAL::Exact_integer>              K3;
    typedef CGAL::Nef_polyhedron_3<K3>                          Nef_polyhedron3;
    typedef CGAL::Polyhedron_3<K3>                              Polyhedron3;
    typedef CGAL::Surface_mesh<K3::Point_3>                     SurfaceMesh3;
    typedef std::pair<K3::Point_3, K3::Vector_3>                Point_with_normal3;
    typedef std::vector<Point_with_normal3>                     Pwn_vector3;

    // Converters from VTK to CGAL
    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh0& tmesh);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron0& tmesh);

    static bool vtkPolyDataToPointsWithNormals(vtkPolyData* poly_data, Pwn_vector0& pwn_vector);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh1& tmesh);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron1& tmesh);

    static bool vtkPolyDataToPointsWithNormals(vtkPolyData* poly_data, Pwn_vector1& pwn_vector);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh2& tmesh);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron2& tmesh);

    static bool vtkPolyDataToPointsWithNormals(vtkPolyData* poly_data, Pwn_vector2& pwn_vector);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron3& tmesh);

    static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh3& tmesh);

    static bool vtkPolyDataToPointsWithNormals(vtkPolyData* poly_data, Pwn_vector3& pwn_vector);

    // Converters from CGAL to VTK
    static bool PolygonMeshToVtkUnstructuredGrid(const SurfaceMesh0& pmesh, vtkUnstructuredGrid* usg);

    static bool SurfaceMeshToPolyData(const SurfaceMesh0& pmesh, vtkPolyData* poly);

    static bool SurfaceMeshToPolyData(const SurfaceMesh2& pmesh, vtkPolyData* poly);

    static bool SurfaceMeshToPolyData(const SurfaceMesh3& pmesh, vtkPolyData* poly);

    static bool PolyhedronToPolyData(const Polyhedron3& pmesh, vtkPolyData* poly);

    template <typename MeshType>
    static bool SurfaceMeshToPolyDataImpl(const MeshType& pmesh, vtkPolyData* poly);

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

    template <typename PointType, typename VectorType, typename PWVType>
    static bool vtkPolyDataToPointsWithNormalsImpl(vtkPolyData* poly_data, PWVType& points);
};
#endif
