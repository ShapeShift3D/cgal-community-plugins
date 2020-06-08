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

//---------VTK----------------------------------
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

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

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
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);

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
// Calls the CGAL::RunBooleanOperations
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
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshA, input1);
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshB, input2);
	Surface_Mesh boolean_op;
	std::size_t id = 1;
	//Result_checking rc;
	try
	{
		RunBooleanOperations(input1, input2, boolean_op, Mode);
	}
	catch (const std::exception& e)
	{
		vtkErrorMacro(<< "Error caught : " << e.what());
		return 0;
	}

	vtkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(boolean_op, output);
	output->Squeeze();

	return 1;
}

//---------------------------------------------------------------------------------------
Surface_Mesh* vtkCGALBoolean::RunBooleanOperations(
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

	return nullptr;
}
