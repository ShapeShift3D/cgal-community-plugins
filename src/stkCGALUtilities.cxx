/**
 * \class stkCGALUtilities
 *
 * \brief Set of CGAL utility functions usable by other classes in this module.
 *
 */

#include "stkCGALUtilities.h"

#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

#include <vtkAppendPolyData.h>

vtkStandardNewMacro(stkCGALUtilities);

// ----------------------------------------------------------------------------
stkCGALUtilities::stkCGALUtilities() {}

// ----------------------------------------------------------------------------
stkCGALUtilities::~stkCGALUtilities() {}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Surface Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh0& tmesh)
{
  typedef typename boost::property_map<SurfaceMesh0, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<SurfaceMesh0, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<SurfaceMesh0>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    SurfaceMesh0>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron0& tmesh)
{
  typedef typename boost::property_map<Polyhedron0, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<Polyhedron0, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<Polyhedron0>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    Polyhedron0>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Surface Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh1& tmesh)
{
  typedef typename boost::property_map<SurfaceMesh1, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<SurfaceMesh1, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<SurfaceMesh1>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    SurfaceMesh1>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron1& tmesh)
{
  typedef typename boost::property_map<Polyhedron1, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<Polyhedron1, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<Polyhedron1>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    Polyhedron1>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Surface Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh2& tmesh)
{
  typedef typename boost::property_map<SurfaceMesh2, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<SurfaceMesh2, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<SurfaceMesh2>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    SurfaceMesh2>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron2& tmesh)
{
  typedef typename boost::property_map<Polyhedron2, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<Polyhedron2, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<Polyhedron2>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    Polyhedron2>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron3& tmesh)
{
  typedef typename boost::property_map<Polyhedron3, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<Polyhedron3, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<Polyhedron3>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    Polyhedron3>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Surface Mesh
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh3& tmesh)
{
  typedef typename boost::property_map<SurfaceMesh3, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<SurfaceMesh3, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<SurfaceMesh3>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    SurfaceMesh3>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * VTK_io_plugin.cpp located at
 * https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
template<typename KernelType, typename MeshType>
bool stkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, MeshType& tmesh)
{
  typedef typename boost::property_map<MeshType, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<MeshType, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<MeshType>::vertex_descriptor vertex_descriptor;

  return stkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor,
    MeshType>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the
 * vtkPointSet_to_polygon_mesh located at
 * https://github.com/CGAL/cgal/blob/9e0dfe81ba6327951637c2768363f8917d7ea12d/BGL/include/CGAL/boost/graph/IO/VTK.h
 *          This method does not write into our PolyData structure. Hence, we do not need to copy
 * them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon Mesh
 *  @return bool Success (true) or failure (false)
 */
template<typename VPMapType, typename PointType, typename VertexDescriptor, typename MeshType>
bool stkCGALUtilities::vtkPolyDataToPolygonMeshImpl(vtkPointSet* polyData, MeshType& tmesh)
{
  VPMapType vpmap = get(CGAL::vertex_point, tmesh);

  // get nb of points and cells
  vtkIdType nb_points = polyData->GetNumberOfPoints();
  vtkIdType nb_cells = polyData->GetNumberOfCells();

  // extract points
  std::vector<VertexDescriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    polyData->GetPoint(i, coords);

    VertexDescriptor v = add_vertex(tmesh);
    put(vpmap, v, PointType(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  // extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    if (polyData->GetCellType(i) != 5 && polyData->GetCellType(i) != 7 &&
      polyData->GetCellType(i) != 9) // only supported cells are triangles, quads and polygons
    {
      continue;
    }

    vtkCell* cell_ptr = polyData->GetCell(i);

    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    if (nb_vertices < 3)
    {
      return false;
    }
      
    std::vector<VertexDescriptor> vr(nb_vertices);
    for (vtkIdType k = 0; k < nb_vertices; ++k)
    {
      vr[k] = vertex_map[cell_ptr->GetPointId(k)];
    }

    CGAL::Euler::add_face(vr, tmesh);
  }

  return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Polygonal Mesh (CGAL) into an Unstructured Grid (VTK).
 *
 *  @param pmesh The input Polygonal Mesh
 *  @param usg The output Unstructured Grid
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(
  const SurfaceMesh0& pmesh, vtkUnstructuredGrid* usg)
{
  typedef typename boost::graph_traits<SurfaceMesh0>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<SurfaceMesh0>::face_descriptor face_descriptor;
  typedef typename boost::graph_traits<SurfaceMesh0>::halfedge_descriptor halfedge_descriptor;

  typedef typename boost::property_map<SurfaceMesh0, CGAL::vertex_point_t>::const_type VPMap;
  typedef typename boost::property_map_value<SurfaceMesh0, CGAL::vertex_point_t>::type Point_3;

  VPMap vpmap = get(CGAL::vertex_point, pmesh);

  vtkNew<vtkPoints> vtk_points;
  vtkNew<vtkCellArray> vtk_cells;

  vtk_points->Allocate(num_vertices(pmesh));
  vtk_cells->Allocate(num_faces(pmesh));

  std::map<vertex_descriptor, vtkIdType> Vids;
  vtkIdType inum = 0;

  for (vertex_descriptor v : vertices(pmesh))
  {
    const Point_3& p = get(vpmap, v);
    vtk_points->InsertNextPoint(
      CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
    Vids[v] = inum++;
  }

  for (face_descriptor f : faces(pmesh))
  {
    vtkIdList* cell = vtkIdList::New();
    for (halfedge_descriptor h : halfedges_around_face(halfedge(f, pmesh), pmesh))
    {
      cell->InsertNextId(Vids[target(h, pmesh)]);
    }
    vtk_cells->InsertNextCell(cell);
    cell->Delete();
  }

  usg->SetPoints(vtk_points);
  usg->SetCells(5, vtk_cells);

  return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
 *
 *  @param pmesh The input Surface Mesh
 *  @param usg The output Unstructured Grid
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::SurfaceMeshToPolyData(const SurfaceMesh0& pmesh, vtkPolyData* poly)
{
  return stkCGALUtilities::SurfaceMeshToPolyDataImpl<SurfaceMesh0>(pmesh, poly);
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
 *
 *  @param pmesh The input Surface Mesh
 *  @param usg The output Unstructured Grid
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::SurfaceMeshToPolyData(const SurfaceMesh2& pmesh, vtkPolyData* poly)
{
  return stkCGALUtilities::SurfaceMeshToPolyDataImpl<SurfaceMesh2>(pmesh, poly);
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
 *
 *  @param pmesh The input Surface Mesh
 *  @param usg The output Unstructured Grid
 *  @return bool Success (true) or failure (false)
 */
bool stkCGALUtilities::SurfaceMeshToPolyData(const SurfaceMesh3& pmesh, vtkPolyData* poly)
{
  return stkCGALUtilities::SurfaceMeshToPolyDataImpl<SurfaceMesh3>(pmesh, poly);
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
 *
 *  @param pmesh The input Surface Mesh
 *  @param usg The output Unstructured Grid
 *  @return bool Success (true) or failure (false)
 */
template<typename MeshType>
bool stkCGALUtilities::SurfaceMeshToPolyDataImpl(const MeshType& pmesh, vtkPolyData* poly)
{
  typedef typename boost::graph_traits<MeshType>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<MeshType>::face_descriptor face_descriptor;
  typedef typename boost::graph_traits<MeshType>::halfedge_descriptor halfedge_descriptor;

  typedef typename boost::property_map<MeshType, CGAL::vertex_point_t>::const_type VPMap;
  typedef typename boost::property_map_value<MeshType, CGAL::vertex_point_t>::type Point_3;

  VPMap vpmap = get(CGAL::vertex_point, pmesh);

  vtkNew<vtkPoints> vtk_points;
  vtkNew<vtkCellArray> vtk_cells;

  vtk_points->Allocate(num_vertices(pmesh));
  vtk_cells->Allocate(num_faces(pmesh));

  std::map<vertex_descriptor, vtkIdType> Vids;
  vtkIdType inum = 0;

  for (vertex_descriptor v : vertices(pmesh))
  {
    const Point_3& p = get(vpmap, v);
    vtk_points->InsertNextPoint(
      CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
    Vids[v] = inum++;
  }

  for (face_descriptor f : faces(pmesh))
  {
    vtkIdList* cell = vtkIdList::New();
    for (halfedge_descriptor h : halfedges_around_face(halfedge(f, pmesh), pmesh))
    {
      cell->InsertNextId(Vids[target(h, pmesh)]);
    }
    vtk_cells->InsertNextCell(cell);
    cell->Delete();
  }

  poly->SetPoints(vtk_points);
  poly->SetPolys(vtk_cells);

  return true;
}
