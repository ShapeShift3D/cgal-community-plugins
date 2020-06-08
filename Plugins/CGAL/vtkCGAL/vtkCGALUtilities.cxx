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
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Surface_Mesh& tmesh)
{
    typedef typename boost::property_map<Surface_Mesh, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<Surface_Mesh, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<Surface_Mesh>::vertex_descriptor vertex_descriptor;

    VPMap vpmap = get(CGAL::vertex_point, tmesh);

    // get nb of points and cells
    vtkIdType nb_points = polyData->GetNumberOfPoints();
    vtkIdType nb_cells = polyData->GetNumberOfCells();

    //extract points
    std::vector<vertex_descriptor> vertex_map(nb_points);
    for (vtkIdType i = 0; i < nb_points; ++i)
    {
        double coords[3];
        polyData->GetPoint(i, coords);

        vertex_descriptor v = add_vertex(tmesh);
        put(vpmap, v, Point_3(coords[0], coords[1], coords[2]));
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
        std::vector<vertex_descriptor> vr(nb_vertices);
        for (vtkIdType k = 0; k < nb_vertices; ++k)
            vr[k] = vertex_map[cell_ptr->GetPointId(k)];

        CGAL::Euler::add_face(vr, tmesh);
    }

    return true;
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
bool vtkCGALUtilities::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, Polyhedron& tmesh)
{
    typedef typename boost::property_map<Polyhedron, CGAL::vertex_point_t>::type VPMap;
    typedef typename boost::property_map_value<Polyhedron, CGAL::vertex_point_t>::type Point_3;
    typedef typename boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;

    VPMap vpmap = get(CGAL::vertex_point, tmesh);

    // get nb of points and cells
    vtkIdType nb_points = polyData->GetNumberOfPoints();
    vtkIdType nb_cells = polyData->GetNumberOfCells();

    //extract points
    std::vector<vertex_descriptor> vertex_map(nb_points);
    for (vtkIdType i = 0; i < nb_points; ++i)
    {
        double coords[3];
        polyData->GetPoint(i, coords);

        vertex_descriptor v = add_vertex(tmesh);
        put(vpmap, v, Point_3(coords[0], coords[1], coords[2]));
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
        std::vector<vertex_descriptor> vr(nb_vertices);
        for (vtkIdType k = 0; k < nb_vertices; ++k)
            vr[k] = vertex_map[cell_ptr->GetPointId(k)];

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
bool vtkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(const Surface_Mesh& pmesh, vtkUnstructuredGrid* usg)
{
	typedef typename boost::graph_traits<Surface_Mesh>::vertex_descriptor   vertex_descriptor;
	typedef typename boost::graph_traits<Surface_Mesh>::face_descriptor     face_descriptor;
	typedef typename boost::graph_traits<Surface_Mesh>::halfedge_descriptor halfedge_descriptor;

	typedef typename boost::property_map<Surface_Mesh, CGAL::vertex_point_t>::const_type VPMap;
	typedef typename boost::property_map_value<Surface_Mesh, CGAL::vertex_point_t>::type Point_3;

	VPMap vpmap = get(CGAL::vertex_point, pmesh);

	vtkPoints* const vtk_points = vtkPoints::New();
	vtkCellArray* const vtk_cells = vtkCellArray::New();

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
	vtk_points->Delete();

	usg->SetCells(5, vtk_cells);
	vtk_cells->Delete();
	return true;
}
