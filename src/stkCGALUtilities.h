/**
 * @class stkCGALUtilities
 * @brief Set of CGAL utility functions usable by all classes in this module.
 *
 * @sa
 * stkCGALUtilities
 */
#pragma once

#include <stkCGALModule.h>
#include <vtkObject.h>

#include <CGAL/Exact_integer.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkDataArray.h>

class vtkPointSet;
class vtkUnstructuredGrid;
class vtkIdTypeArray;
class vtkIntArray;

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALUtilities : public vtkObject
{
public:
  static stkCGALUtilities* New();
  vtkTypeMacro(stkCGALUtilities, vtkObject);

  // Kernel 0
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K0;
  typedef CGAL::Surface_mesh<K0::Point_3> SurfaceMesh0;
  typedef CGAL::Mesh_polyhedron_3<K0>::type Polyhedron0;

  // Kernel 1
  typedef CGAL::Simple_cartesian<double> K1;
  typedef CGAL::Surface_mesh<K1::Point_3> SurfaceMesh1;
  typedef CGAL::Polyhedron_3<K1> Polyhedron1;

  // Kernel 2
  typedef CGAL::Exact_predicates_exact_constructions_kernel K2;
  typedef CGAL::Surface_mesh<K2::Point_3> SurfaceMesh2;
  typedef CGAL::Polyhedron_3<K2> Polyhedron2;

  // Kernel 3
  typedef CGAL::Homogeneous<CGAL::Exact_integer> K3;
  typedef CGAL::Nef_polyhedron_3<K3> Nef_polyhedron3;
  typedef CGAL::Polyhedron_3<K3> Polyhedron3;
  typedef CGAL::Surface_mesh<K3::Point_3> SurfaceMesh3;

  // Converters from VTK to CGAL
  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh0& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  template<typename KernelType, typename Point_with_normal>
  static bool vtkPolyDataToOrientedPoints(
    vtkPolyData* polyData, Point_with_normal& orientedPoints);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron0& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh1& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron1& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh2& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron2& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, Polyhedron3& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, SurfaceMesh3& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  // Converters from CGAL to VTK
  static bool PolygonMeshToVtkUnstructuredGrid(const SurfaceMesh0& pmesh, vtkUnstructuredGrid* usg);

  static bool SurfaceMeshToPolyData(const SurfaceMesh0& pmesh, vtkPolyData* poly);

  static bool SurfaceMeshToPolyData(const SurfaceMesh2& pmesh, vtkPolyData* poly);

  static bool SurfaceMeshToPolyData(const SurfaceMesh3& pmesh, vtkPolyData* poly);

  template<typename MeshType>
  static bool SurfaceMeshToPolyDataImpl(const MeshType& pmesh, vtkPolyData* poly);

protected:
  stkCGALUtilities();
  ~stkCGALUtilities();

private:
  stkCGALUtilities(const stkCGALUtilities&) = delete;
  void operator=(const stkCGALUtilities&) = delete;

  template<typename KernelType, typename MeshType>
  static bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, MeshType& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);

  template<typename VPMapType, typename PointType, typename VertexDescriptor, typename MeshType>
  static bool vtkPolyDataToPolygonMeshImpl(vtkPointSet* polyData, MeshType& tmesh,
    vtkIdTypeArray* cellOriginalIdsArray = nullptr, vtkIntArray* nullFaceMaskArray = nullptr);
};

//----------------------------------------------------------------------------
/** @brief Convert a polydata to a CGAL compatible data structure
 *
 *  @param polyData                  input data structure with Point Normals
 *  @param orientedPoints            output data structure
 *  @tparam KernelType               must be a CGAL kernel compatible type
 *  @tparam Point_with_normal        must be a vector of pairs { Point_3, Vector_3 }
 *                                   using the corresponding CGAL kernel
 *
 *  @return bool Success (true) or failure (false)
 */
template<typename KernelType, typename Point_with_normal>
bool stkCGALUtilities::vtkPolyDataToOrientedPoints(
  vtkPolyData* polyData, Point_with_normal& orientedPoints)
{
  using Point = typename KernelType::Point_3;
  using Vector = typename KernelType::Vector_3;

  vtkDataArray* normals = polyData->GetPointData()->GetNormals();
  if (!normals)
  {
    return false;
  }

  // Extract points and normals
  double p[3], v[3];
  vtkIdType num_points = polyData->GetNumberOfPoints();
  for (vtkIdType i = 0; i < num_points; ++i)
  {
    polyData->GetPoint(i, p);
    normals->GetTuple(i, v);

    orientedPoints.push_back(std::make_pair(Point(p[0], p[1], p[2]), Vector(v[0], v[1], v[2])));
  }

  return true;
}

//----------------------------------------------------------------------------