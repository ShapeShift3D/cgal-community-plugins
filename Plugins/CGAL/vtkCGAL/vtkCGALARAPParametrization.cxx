/**
* \class vtkCGALARAPParametrization
*
* \brief This filter evaluates self-intersections inside a PolyData. PolyData made of two non-connected surfaces
*        that intersect each other are counted as self-intersections.
*		 
*     
* Inputs: inputMesh (port == 0, vtkPolyData)
* Output: output (port == 0, vtkPolyData)
* 
*/

//---------VTK----------------------------------
#include "vtkCGALARAPParametrization.h"

#include <vtkCommand.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include <vtkUnstructuredGrid.h>
#include <vtkCellData.h>
#include <vtkIntArray.h>

#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>

//---------Boost----------------------------------
#include <boost/foreach.hpp>

//---------CGAL---------------------------------
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/properties_Surface_mesh_features.h>
#include <CGAL/Surface_mesh_parameterization/Two_vertices_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>

//---------Module-------------------------------
#include <vtkCGALUtilities.h>

typedef CGAL::Simple_cartesian<double>          Kernel;
typedef Kernel::Point_2							Point_2;
typedef Kernel::Point_3                         Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3>		Surface_Mesh;

typedef boost::graph_traits<Surface_Mesh>::vertex_descriptor     vertex_descriptor;
typedef boost::graph_traits<Surface_Mesh>::halfedge_descriptor   halfedge_descriptor;
typedef boost::graph_traits<Surface_Mesh>::face_descriptor       face_descriptor;

//typedef CGAL::Unique_hash_map<SM_halfedge_descriptor, Point_2>	UV_uhm;
//typedef boost::associative_property_map<UV_uhm>					UV_pmap;
typedef Surface_Mesh::Property_map<vertex_descriptor, Point_2>  UV_pmap;

namespace SMP = CGAL::Surface_mesh_parameterization;

vtkStandardNewMacro(vtkCGALARAPParametrization);

// -----------------------------------------------------------------------------
vtkCGALARAPParametrization::vtkCGALARAPParametrization()
{
	this->Input = nullptr;
	this->Output = nullptr;

	this->Lambda = 1000;
	this->NumberOfIterations = 50;
	this->Tolerance = 1e-6;
}

// -----------------------------------------------------------------------------
vtkCGALARAPParametrization::~vtkCGALARAPParametrization() {}

//---------------------------------------------------
void vtkCGALARAPParametrization::SetInputMesh(vtkPolyData* mesh)
{
	this->Input = mesh;
}

//---------------------------------------------------
vtkPolyData* vtkCGALARAPParametrization::GetInputMesh()
{
	return Input;
}

//---------------------------------------------------
void vtkCGALARAPParametrization::SetOutput(vtkPolyData* mesh)
{
	this->Output = mesh;
}

//---------------------------------------------------
vtkPolyData* vtkCGALARAPParametrization::GetOutput()
{
	return Output;
}

//---------------------------------------------------
int vtkCGALARAPParametrization::Update()
{
	Surface_Mesh surfaceMesh;
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(this->Input, surfaceMesh);
	
	if (!CGAL::is_triangle_mesh(surfaceMesh))
	{
		vtkErrorMacro("Mesh is not triangular.");
		return 0;
	}

	// a halfedge on the border
	halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(surfaceMesh, CGAL::Polygon_mesh_processing::parameters::all_default()).first;
	
	//UV_uhm uv_uhm;
	//UV_pmap uv_map(uv_uhm);
	UV_pmap uv_map = surfaceMesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;
	
	typedef SMP::Two_vertices_parameterizer_3<Surface_Mesh> Border_Parameterizer;

	SMP::Error_code err = SMP::parameterize(surfaceMesh,
		SMP::ARAP_parameterizer_3<Surface_Mesh, Border_Parameterizer>(
			Border_Parameterizer(),
			CGAL::Eigen_solver_traits<Eigen::SparseLU<CGAL::Eigen_sparse_matrix<double>::EigenType> >(),
			this->Lambda,
			this->NumberOfIterations,
			this->Tolerance),
		bhd,
		uv_map);

	if (err != SMP::OK) {
		vtkErrorMacro("Error: " << SMP::get_error_message(err));
		return 0;
	}

	OutputToPolyData<Surface_Mesh, halfedge_descriptor, UV_pmap>(surfaceMesh, bhd, uv_map, Output);
	
	return 1;
}

//---------------------------------------------------
template <typename SurfaceMesh, typename HalfedgeDescriptor, typename VertexUVMap>
bool vtkCGALARAPParametrization::OutputToPolyData(SurfaceMesh& sm, HalfedgeDescriptor bhd, VertexUVMap uv_map, vtkPolyData* polyDataOut)
{
	// Output To file
	std::size_t vertices_counter = 0, faces_counter = 0;
	typedef boost::unordered_map<vertex_descriptor, std::size_t> Vertex_index_map;
	Vertex_index_map vium;
	boost::associative_property_map<Vertex_index_map> vimap(vium);

	std::ofstream out("debug.off");

	// Process
	cout << "OFF\n";
	cout << sm.number_of_vertices() << " " << sm.number_of_faces() << " 0\n";
	boost::graph_traits<SurfaceMesh>::vertex_iterator vit, vend;
	boost::tie(vit, vend) = CGAL::vertices(sm);
	while (vit != vend)
	{
		vertex_descriptor vd = *vit++;
		out << get(uv_map, vd) << " 0\n";
		put(vimap, vd, vertices_counter++);
	}

	BOOST_FOREACH(face_descriptor fd, CGAL::faces(sm)) {
		HalfedgeDescriptor hd = halfedge(fd, sm);
		out << "3";
		BOOST_FOREACH(vertex_descriptor vd, vertices_around_face(hd, sm)) {
			out << " " << get(vimap, vd);
		}
		out << '\n';
		faces_counter++;
	}
	if (vertices_counter != sm.number_of_vertices())
		return 0;
	else if (faces_counter != sm.number_of_faces())
		return 0;
	else
		return 1;
}