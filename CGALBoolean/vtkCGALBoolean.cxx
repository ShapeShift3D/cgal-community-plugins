/**
* \class vtkCGALBoolean
*
* \brief This filter takes two inputs, inputMeshA and inputMeshB, of type vtkPolyData and applies one of the four boolean operations (Union, Intersection, Difference1 (A - B) and Difference2 (B - A))
*        to them. The user will have the option to select one of the four operations from a drop down menu. The two inputs will be converted to CGAL Polygon Mesh class since vtkPolyData is not a valid input. 
*		 The converted inputs will then be fed into the appropriate function for execution. The result of the function will be converted to a vtkUnstructuredGrid and be outputted as such. 
*        
*		 
*        
* Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
* Output: output (port == 0, vtkUnstructuredGrid)
* 
*/

//---------vtk----------------------------------
#include "vtkCGALBoolean.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include "vtkInformation.h"
#include <vtkInformationVector.h>

//---------CGAL---------------------------------
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Mesh_3/io_signature.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/Euler_operations.h>	
#include <CGAL/property_map.h>
#include <CGAL/IO/Complex_3_in_triangulation_3_to_vtk.h>
#include <CGAL/boost/graph/io.h>
//---------Boost--------------------------------------------------
#include <boost/graph/graph_traits.hpp>
#include <boost/unordered_map.hpp>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALBoolean);

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALBoolean::vtkCGALBoolean()
{
  SetNumberOfInputPorts(2);
  SetNumberOfOutputPorts(1);

  this->Mode = UNION;
}

//---------------------------------------------------
vtkPolyData* vtkCGALBoolean::GetInputMeshA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALBoolean::GetInputMeshB()
{
	if (this->GetNumberOfInputConnections(1) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(1, 0));
}

//----------------------------------------------------
int vtkCGALBoolean::FillOutputPortInformation(int,
	vtkInformation* info)
{
	// Always returns a vtkPolyData
	info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid");
	return 1;
}

//----------------------------------------------------
void vtkCGALBoolean::SetModeToUnion()
{
	this->SetMode(Modes::UNION);
}

//----------------------------------------------------
void vtkCGALBoolean::SetModeToIntersection()
{
	this->SetMode(Modes::INTERSECTION);
}

//----------------------------------------------------
void vtkCGALBoolean::SetModeToDifference()
{
	this->SetMode(Modes::TM1_MINUS_TM2);
}

//----------------------------------------------------
void vtkCGALBoolean::SetModeToDifference2()
{
	this->SetMode(Modes::TM2_MINUS_TM1);
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::run_boolean_operations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALBoolean::RequestData(vtkInformation *,
                                             vtkInformationVector **inputVector,
                                             vtkInformationVector *outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
	vtkPolyData* inputMeshA = this->GetInputMeshA();
	vtkPolyData* inputMeshB = this->GetInputMeshB();

	if (inputMeshB == nullptr || inputMeshB->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("No points found in input mesh.");
		return 0;
	}

	if (inputMeshA == nullptr || inputMeshA->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("No points found in input point set.");
		return 0;
	}

	vtkInformation* outInfo = outputVector->GetInformationObject(0);
	auto output = vtkUnstructuredGrid::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

	Surface_Mesh input1, input2;
	vtkPointSet_to_polygon_mesh(inputMeshA, input1);
	vtkPointSet_to_polygon_mesh(inputMeshB, input2);	
	Surface_Mesh boolean_op;
	std::size_t id = 1;
	//Result_checking rc;
	run_boolean_operations(input1, input2, boolean_op, Mode);
	polygon_mesh_to_vtkUnstructured(boolean_op, output);

  return 1;
}

//----------------------------------------------------
// Converts a vtkPointSet to a CGAL Polygon Mesh Class
template <typename TM>
bool vtkCGALBoolean::vtkPointSet_to_polygon_mesh(vtkPointSet* poly_data,
	TM& tmesh)
{
	typedef typename boost::property_map<TM, CGAL::vertex_point_t>::type VPMap;
	typedef typename boost::property_map_value<TM, CGAL::vertex_point_t>::type Point_3;
	typedef typename boost::graph_traits<TM>::vertex_descriptor vertex_descriptor;

	VPMap vpmap = get(CGAL::vertex_point, tmesh);

	// get nb of points and cells
	vtkIdType nb_points = poly_data->GetNumberOfPoints();
	vtkIdType nb_cells = poly_data->GetNumberOfCells();

	//extract points
	std::vector<vertex_descriptor> vertex_map(nb_points);
	for (vtkIdType i = 0; i < nb_points; ++i)
	{
		double coords[3];
		poly_data->GetPoint(i, coords);

		vertex_descriptor v = add_vertex(tmesh);
		put(vpmap, v, Point_3(coords[0], coords[1], coords[2]));
		vertex_map[i] = v;
	}

	//extract cells
	for (vtkIdType i = 0; i < nb_cells; ++i)
	{
		if (poly_data->GetCellType(i) != 5
			&& poly_data->GetCellType(i) != 7
			&& poly_data->GetCellType(i) != 9) //only supported cells are triangles, quads and polygons
			continue;
		vtkCell* cell_ptr = poly_data->GetCell(i);

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

//----------------------------------------------------------------------------------------------------------
// Converts a CGAL Polygon Mesh to a vtkUnstructuredGrid
template<typename PM>
vtkUnstructuredGrid* vtkCGALBoolean::polygon_mesh_to_vtkUnstructured(const PM& pmesh,//PolygonMesh
	vtkUnstructuredGrid* usg=0)
{
	typedef typename boost::graph_traits<PM>::vertex_descriptor   vertex_descriptor;
	typedef typename boost::graph_traits<PM>::face_descriptor     face_descriptor;
	typedef typename boost::graph_traits<PM>::halfedge_descriptor halfedge_descriptor;

	typedef typename boost::property_map<PM, CGAL::vertex_point_t>::const_type VPMap;
	typedef typename boost::property_map_value<PM, CGAL::vertex_point_t>::type Point_3;
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
	return usg;
}

//---------------------------------------------------------------------------------------
Surface_Mesh* vtkCGALBoolean::run_boolean_operations(
	Surface_Mesh& tm1,
	Surface_Mesh& tm2,
	Surface_Mesh& operation,
	int Mode)
{
	typedef boost::optional<Surface_Mesh*> OSM;
	std::array<OSM, 4> output;

	if (Mode == vtkCGALBoolean::Modes::UNION) {
		output[CGAL::Polygon_mesh_processing::Corefinement::UNION] = OSM(&operation);
		CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
		return *output[0];
	}
	if (Mode == vtkCGALBoolean::Modes::INTERSECTION) {
		output[CGAL::Polygon_mesh_processing::Corefinement::INTERSECTION] = OSM(&operation);
		CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
		return *output[1];
	}
	if (Mode == vtkCGALBoolean::Modes::TM1_MINUS_TM2) {

		output[CGAL::Polygon_mesh_processing::Corefinement::TM1_MINUS_TM2] = OSM(&operation);
		CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
		return *output[2];
	}
	if (Mode == vtkCGALBoolean::Modes::TM2_MINUS_TM1) {
		output[CGAL::Polygon_mesh_processing::Corefinement::TM2_MINUS_TM1] = OSM(&operation);
		CGAL::Polygon_mesh_processing::corefine_and_compute_boolean_operations(tm1, tm2, output);
		return *output[3];
	}
}
