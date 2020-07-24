/**
* \class vtkCGALUtilities
*
* \brief Set of CGAL utility functions usable by other classes in this module.
*        
*/


#include "vtkCGALUtilities.h"

#include <vtkObjectFactory.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>

#include <vtkAppendPolyData.h>

vtkStandardNewMacro(vtkCGALUtilities);

// ----------------------------------------------------------------------------
vtkCGALUtilities::vtkCGALUtilities()
{}

// ----------------------------------------------------------------------------
vtkCGALUtilities::~vtkCGALUtilities()
{}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Surface Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh0& tmesh)
{
    typedef typename boost::property_map<SurfaceMesh0, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<SurfaceMesh0, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<SurfaceMesh0>::vertex_descriptor vertex_descriptor;

    return vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor, SurfaceMesh0>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron0& tmesh)
{
    typedef typename boost::property_map<Polyhedron0, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<Polyhedron0, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<Polyhedron0>::vertex_descriptor vertex_descriptor;

    return vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor, Polyhedron0>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Surface Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh1& tmesh)
{
    typedef typename boost::property_map<SurfaceMesh1, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<SurfaceMesh1, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<SurfaceMesh1>::vertex_descriptor vertex_descriptor;

    return vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor, SurfaceMesh1>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Surface Mesh (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Surface Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, SurfaceMesh2& tmesh)
{
	typedef typename boost::property_map<SurfaceMesh2, CGAL::vertex_point_t>::type VPMap;
	typedef typename boost::property_map_value<SurfaceMesh2, CGAL::vertex_point_t>::type Point_3;
	typedef typename boost::graph_traits<SurfaceMesh2>::vertex_descriptor vertex_descriptor;

	return vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor, SurfaceMesh2>(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon Mesh
*  @return bool Success (true) or failure (false)
*/
template <typename KernelType, typename MeshType>
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, MeshType& tmesh)
{
    typedef typename boost::property_map<MeshType, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<MeshType, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<MeshType>::vertex_descriptor vertex_descriptor;

    return vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl<VPMap, Point_3, vertex_descriptor, MeshType >(polyData, tmesh);
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polyhedron (CGAL). Code taken from the VTK_io_plugin.cpp
*          located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon Mesh
*  @return bool Success (true) or failure (false)
*/
template <typename VPMapType, typename PointType, typename VertexDescriptor, typename MeshType>
bool vtkCGALUtilities::vtkPolyDataToPolygonMeshImpl(vtkPointSet* polyData, MeshType& tmesh)
{
    VPMapType vpmap = get(CGAL::vertex_point, tmesh);

    // get nb of points and cells
    vtkIdType nb_points = polyData->GetNumberOfPoints();
    vtkIdType nb_cells = polyData->GetNumberOfCells();

    //extract points
    std::vector<VertexDescriptor> vertex_map(nb_points);
    for (vtkIdType i = 0; i < nb_points; ++i)
    {
        double coords[3];
        polyData->GetPoint(i, coords);

        VertexDescriptor v = add_vertex(tmesh);
        put(vpmap, v, PointType(coords[0], coords[1], coords[2]));
        vertex_map[i] = v;
    }

    //extract cells
    for (vtkIdType i = 0; i < nb_cells; ++i)
    {
        if (polyData->GetCellType(i) != 5
            && polyData->GetCellType(i) != 7
            && polyData->GetCellType(i) != 9) //only supported cells are triangles, quads and polygons
            continue;
        vtkCell* cell_ptr = polyData->GetCell(i);

        vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
        if (nb_vertices < 3)
            return false;
        std::vector<VertexDescriptor> vr(nb_vertices);
        for (vtkIdType k = 0; k < nb_vertices; ++k)
            vr[k] = vertex_map[cell_ptr->GetPointId(k)];

        CGAL::Euler::add_face(vr, tmesh);
    }

    return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into Polygon 2 (CGAL).
*          This method does not write into our PolyData structure. 
*		   Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon 2 Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::vtkPolyDataToPolygon2(vtkPointSet* polyData, Polygon_2& tmesh)
{
    // get nb of points and cells
    vtkIdType nb_points = polyData->GetNumberOfPoints();

    // Extract points
    for (vtkIdType i = 0; i < nb_points; ++i)
    {
        double coords[3];
        polyData->GetPoint(i, coords);

        tmesh.push_back(Point_2(coords[0], coords[1]));
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
bool vtkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(const SurfaceMesh0& pmesh, vtkUnstructuredGrid* usg)
{
	typedef typename boost::graph_traits<SurfaceMesh0>::vertex_descriptor   vertex_descriptor;
	typedef typename boost::graph_traits<SurfaceMesh0>::face_descriptor     face_descriptor;
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
		vtk_points->InsertNextPoint(CGAL::to_double(p.x()),
									CGAL::to_double(p.y()),
									CGAL::to_double(p.z()));
		Vids[v] = inum++;
	}

	for (face_descriptor f : faces(pmesh))
	{
		vtkIdList* cell = vtkIdList::New();
		for (halfedge_descriptor h :
		halfedges_around_face(halfedge(f, pmesh), pmesh))
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
bool vtkCGALUtilities::SurfaceMeshToPolyData(const SurfaceMesh0& pmesh, vtkPolyData* poly)
{
	return vtkCGALUtilities::SurfaceMeshToPolyDataImpl<SurfaceMesh0>(pmesh, poly);
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Surface Mesh
*  @param usg The output Unstructured Grid
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::SurfaceMeshToPolyData(const SurfaceMesh2& pmesh, vtkPolyData* poly)
{
	return vtkCGALUtilities::SurfaceMeshToPolyDataImpl<SurfaceMesh2>(pmesh, poly);
}

//----------------------------------------------------------------------------

/** @brief Converts a Surface Mesh (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Surface Mesh
*  @param usg The output Unstructured Grid
*  @return bool Success (true) or failure (false)
*/
template <typename MeshType>
bool vtkCGALUtilities::SurfaceMeshToPolyDataImpl(const MeshType& pmesh, vtkPolyData* poly)
{
	typedef typename boost::graph_traits<MeshType>::vertex_descriptor   vertex_descriptor;
	typedef typename boost::graph_traits<MeshType>::face_descriptor     face_descriptor;
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
		vtk_points->InsertNextPoint(CGAL::to_double(p.x()),
			CGAL::to_double(p.y()),
			CGAL::to_double(p.z()));
		Vids[v] = inum++;
	}

	for (face_descriptor f : faces(pmesh))
	{
		vtkIdList* cell = vtkIdList::New();
		for (halfedge_descriptor h :
		halfedges_around_face(halfedge(f, pmesh), pmesh))
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

//----------------------------------------------------------------------------

/** @brief Converts a Polygon 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Surface Mesh
*  @param usg The output Unstructured Grid
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::Polygon2ToPolyLine(const Polygon_2& pmesh, vtkPolyData* polyline)
{
	vtkNew<vtkPoints> vtk_points;
	
	typename Polygon_2::Vertex_const_iterator vertex_iterator;

	for (vertex_iterator = pmesh.vertices_begin(); vertex_iterator != pmesh.vertices_end(); ++vertex_iterator)
	{
		std::cout << "Point: " << vertex_iterator->x().exact().to_double() << ", " << vertex_iterator->y().exact().to_double() << ", " << "0" << endl;
		vtk_points->InsertNextPoint(vertex_iterator->x().exact().to_double(), 
									vertex_iterator->y().exact().to_double(), 
									0);
	}


	// Not elegant but functional
	vtkNew<vtkCellArray> vtk_cells;

	for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints() - 1; ++i)
	{
		vtk_cells->InsertNextCell(2);
		vtk_cells->InsertCellPoint(i);
		vtk_cells->InsertCellPoint(i + 1);
	}

	vtk_cells->InsertNextCell(2);
	vtk_cells->InsertCellPoint(vtk_points->GetNumberOfPoints() - 1);
	vtk_cells->InsertCellPoint(0);

	polyline->SetPoints(vtk_points);
	polyline->SetLines(vtk_cells);

	return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Polygon with holes list 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Surface Mesh
*  @param usg The output Unstructured Grid
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALUtilities::PwhList2ToPolyData(const Pwh_list_2& pmesh, vtkPolyData* polydata)
{
	vtkNew<vtkAppendPolyData> appendFilter;

	typename Pwh_list_2::const_iterator polygon_iterator;
	typename CGAL::Polygon_with_holes_2<K2>::Hole_const_iterator hole_iterator;

	for (polygon_iterator = pmesh.begin(); polygon_iterator != pmesh.end(); ++polygon_iterator)
	{
		vtkNew<vtkPolyData> boundary;
		vtkCGALUtilities::Polygon2ToPolyLine(polygon_iterator->outer_boundary(), boundary);
		appendFilter->AddInputData(boundary);

		for (hole_iterator = polygon_iterator->holes_begin(); hole_iterator != polygon_iterator->holes_begin(); ++hole_iterator)
		{
			vtkNew<vtkPolyData> hole;
			vtkCGALUtilities::Polygon2ToPolyLine(*hole_iterator, hole);
			appendFilter->AddInputData(hole);
		}
	}

	appendFilter->Update();

	polydata->ShallowCopy(appendFilter->GetOutput());


	return true;
}