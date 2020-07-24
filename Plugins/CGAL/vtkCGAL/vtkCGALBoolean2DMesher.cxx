/**
* \class vtkCGALBoolean2DMesher
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
#include "vtkCGALBoolean2DMesher.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

//---------CGAL---------------------------------
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Boolean_set_operations_2.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_2.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>
#include <vtkCGALPHBooleanSetOperations.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALBoolean2DMesher);

typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2											Point_2;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>						Pwh_list_2;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALBoolean2DMesher::vtkCGALBoolean2DMesher()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//---------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputMeshA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputMeshB()
{
	if (this->GetNumberOfInputConnections(1) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(1, 0));
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALBoolean2DMesher::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputMeshA = this->GetInputMeshA();
	vtkPolyData* inputMeshB = this->GetInputMeshB();

	if (inputMeshA == nullptr)
	{
		vtkErrorMacro("Input Mesh A is empty.");
		return 0;
	}

	if (inputMeshA->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Mesh A does not contain any cell structure.");
		return 0;
	}

	if (inputMeshA->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Mesh A contains no cells.");
		return 0;
	}

	if (inputMeshA->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Mesh A does not contain any point structure.");
		return 0;
	}

	if (inputMeshA->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Mesh A contains no points.");
		return 0;
	}

	if (inputMeshB == nullptr)
	{
		vtkErrorMacro("Input Mesh B is empty.");
		return 0;
	}

	if (inputMeshB->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Mesh B does not contain any cell structure.");
		return 0;
	}

	if (inputMeshB->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Mesh B contains no cells.");
		return 0;
	}

	if (inputMeshB->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Mesh B does not contain any point structure.");
		return 0;
	}

	if (inputMeshB->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Mesh B contains no points.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	// A polygon is a closed chain of edges. Several algorithms are available for polygons. 
	// For some of those algorithms, it is necessary that the polygon is simple. 
	// A polygon is simple if edges don't intersect, except consecutive edges, which intersect in their common vertex.
	// Taken from https://doc.cgal.org/4.14.3/Polygon/index.html
	Polygon_2 input0, input1;
	//vtkCGALUtilities::vtkPolyDataToPolygon2(inputMeshA, input0);
	//vtkCGALUtilities::vtkPolyDataToPolygon2(inputMeshB, input1);

	input0.push_back(Point_2(0, 0));
	input0.push_back(Point_2(5, 0));
	input0.push_back(Point_2(3.5, 1.5));
	input0.push_back(Point_2(2.5, 0.5));
	input0.push_back(Point_2(1.5, 1.5));
	//std::cout << "P = "; print_polygon(P);
	input1.push_back(Point_2(0, 2));
	input1.push_back(Point_2(1.5, 0.5));
	input1.push_back(Point_2(2.5, 1.5));
	input1.push_back(Point_2(3.5, 0.5));
	input1.push_back(Point_2(5, 2));


	// check if the polygon is simple.
	cout << " === Input 0 === " << endl;
	cout << "The polygon is " <<
		(input0.is_simple() ? "" : "not ") << "simple." << endl;
	// check if the polygon is convex
	cout << "The polygon is " <<
		(input0.is_convex() ? "" : "not ") << "convex." << endl;
	cout << "The polygon is " <<
		(input0.is_clockwise_oriented() ? "" : "not ") << "clockwise." << endl;
	cout << "Signed area: " << input0.area() << endl;
	CGAL::Bbox_2 input0Bbox = input0.bbox();
	cout << "The origin is " <<
		(input0.bounded_side(Point_2(0, 0)) ? "" : "not ") << "inside the polygon." << endl;

	cout << " === Input 1 === " << endl;
	cout << "The polygon is " <<
		(input1.is_simple() ? "" : "not ") << "simple." << endl;
	// check if the polygon is convex
	cout << "The polygon is " <<
		(input1.is_convex() ? "" : "not ") << "convex." << endl;
	cout << "The polygon is " <<
		(input1.is_clockwise_oriented() ? "" : "not ") << "clockwise." << endl;
	cout << "Signed area: " << input1.area() << endl;
	cout << "The origin is " <<
		(input1.bounded_side(Point_2(0, 0)) ? "" : "not ") << "inside the polygon." << endl;

	Pwh_list_2 result;
	CGAL::intersection(input0, input1, std::back_inserter(result));

	vtkNew<vtkCGALPHBooleanSetOperations> booleanSetOperations;
	booleanSetOperations->SetInputModeToWithoutHole();
	booleanSetOperations->SetPolygonAInput(&input0);
	booleanSetOperations->SetPolygonBInput(&input1);
	booleanSetOperations->SetPolygonWithHoleListOutput(&result);
	booleanSetOperations->SetOperationModeToIntersection();
	booleanSetOperations->Update();

	vtkCGALUtilities::PwhList2ToPolyData(result, output0);
	return 1;
}

