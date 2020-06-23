/**
* \class vtkCGAL2DBoolean
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
#include "vtkCGAL2DBoolean.h"
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
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <list>

//---------Boost--------------------------------------------------
#include <boost/graph/graph_traits.hpp>
#include <boost/unordered_map.hpp>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGAL2DBoolean);

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGAL2DBoolean::vtkCGAL2DBoolean()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);

}

//---------------------------------------------------
vtkPolyData* vtkCGAL2DBoolean::GetInputMeshA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGAL2DBoolean::GetInputMeshB()
{
	if (this->GetNumberOfInputConnections(1) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(1, 0));
}

//----------------------------------------------------
int vtkCGAL2DBoolean::FillOutputPortInformation(int,
	vtkInformation* info)
{
	// Always returns a vtkPolyData
	info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid");
	return 1;
}


// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGAL2DBoolean::RequestData(vtkInformation *,
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
	std::size_t id = 1;
	//Result_checking rc;

	Surface_Mesh symmR;
	Pwh_list_2::const_iterator it;

	CGAL::symmetric_difference(input1, input2, std::back_inserter(symmR));

	vtkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(symmR, output);
	output->Squeeze();

	return 1;
}

