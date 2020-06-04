/**
* \class vtkCGALSelfIntersectionMeasurer
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
#include "vtkCGALSelfIntersectionMeasurer.h"

#include <vtkCommand.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include <vtkUnstructuredGrid.h>
#include <vtkCellData.h>
#include <vtkIntArray.h>

//---------CGAL---------------------------------
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

//---------Module-------------------------------
#include <vtkCGALUtilities.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::face_descriptor face_descriptor;


vtkStandardNewMacro(vtkCGALSelfIntersectionMeasurer);

// -----------------------------------------------------------------------------
vtkCGALSelfIntersectionMeasurer::vtkCGALSelfIntersectionMeasurer()
{
	this->SetNumberOfInputPorts(1);
	this->SetNumberOfOutputPorts(1);

	this->SelfIntersectionsArrayName = "Self-Intersections";
	this->PrintSelfIntersectingPairs = false;
}

// -----------------------------------------------------------------------------
vtkCGALSelfIntersectionMeasurer::~vtkCGALSelfIntersectionMeasurer() {}

//---------------------------------------------------
vtkPolyData* vtkCGALSelfIntersectionMeasurer::GetInputMesh()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::run_boolean_operations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALSelfIntersectionMeasurer::RequestData(vtkInformation *,
												vtkInformationVector **inputVector,
												vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputMesh = this->GetInputMesh();

	if (inputMesh == nullptr || inputMesh->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("No points found in input mesh.");
		return 0;
	}

	vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	Surface_Mesh surfaceMesh;
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, surfaceMesh);

	if (!CGAL::is_triangle_mesh(surfaceMesh))
	{
		vtkErrorMacro("Mesh is not triangular.");
		return 0;
	}

	namespace PMP = CGAL::Polygon_mesh_processing;

	bool intersecting = CGAL::Polygon_mesh_processing::does_self_intersect(surfaceMesh,
		PMP::parameters::vertex_point_map(get(CGAL::vertex_point, surfaceMesh)));

	std::vector<std::pair<face_descriptor, face_descriptor>> intersected_tris;
	PMP::self_intersections(surfaceMesh, std::back_inserter(intersected_tris));

	if (intersecting)
	{
		vtkErrorMacro("Self-Intersections were found in the mesh. " << 
			intersected_tris.size() << " pairs of triangles intersect.");
	}

	// Copy to output
	output->DeepCopy(inputMesh);

	vtkNew<vtkIntArray> intersectingTrisArray;
	intersectingTrisArray->SetName(this->SelfIntersectionsArrayName.c_str());
	intersectingTrisArray->SetNumberOfComponents(1);
	intersectingTrisArray->SetNumberOfTuples(output->GetNumberOfCells());
	intersectingTrisArray->Fill(0);

	int currentTuple = 0;

	for (vtkIdType i = 0; i < intersected_tris.size(); ++i)
	{
		currentTuple = intersectingTrisArray->GetTuple1(intersected_tris[i].first.idx());
		intersectingTrisArray->SetTuple1(intersected_tris[i].first.idx(), currentTuple + 1);

		currentTuple = intersectingTrisArray->GetTuple1(intersected_tris[i].second.idx());
		intersectingTrisArray->SetTuple1(intersected_tris[i].second.idx(), currentTuple + 1);
	}

	if (this->PrintSelfIntersectingPairs)
	{
		for (vtkIdType i = 0; i < intersected_tris.size(); ++i)
		{
			vtkWarningMacro("Triangle " << intersected_tris[i].first.idx() << " is intersecting with "
							<< intersected_tris[i].second.idx() << ".");
		}
	}

	output->GetCellData()->AddArray(intersectingTrisArray);
	return 1;
}
