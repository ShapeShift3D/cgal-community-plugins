/**
* \class vtkCGALBoolean2DMesher
*
* \brief 
*        
*		 Conditions for valid polygons:
*
*		 Closed Boundary - the polygon's outer boundary must be a connected sequence of curves, that start and end at the same vertex.
*		 Simplicity - the polygon must be simple.
*		 Orientation - the polygon's outer boundary must be counter-clockwise oriented.
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
#include <CGAL/Polygon_set_2.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALBoolean2DMesher);

typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2											Point_2;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>						Pwh_list_2;
typedef CGAL::Polygon_set_2<K>								Polygon_set_2;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALBoolean2DMesher::vtkCGALBoolean2DMesher()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
  this->OperationMode = vtkCGALBoolean2DMesher::OperationModes::INTERSECTION;
  this->DebugMode = false;

  // PolyLine A
  this->InvertPolyLineAOrientation = false;
  this->ForcePolyLineAOrientation = false;
  this->PolyLineAOrientation = vtkCGALBoolean2DMesher::PolygonOrientations::CLOCKWISE;

  // PolyLine B
  this->InvertPolyLineBOrientation = false;
  this->ForcePolyLineBOrientation = false;
  this->PolyLineBOrientation = vtkCGALBoolean2DMesher::PolygonOrientations::CLOCKWISE;
}

//---------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputPolyLineA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputPolyLineB()
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
	vtkPolyData* inputPolyLineA = this->GetInputPolyLineA();
	vtkPolyData* inputPolyLineB = this->GetInputPolyLineB();

	if (inputPolyLineA == nullptr)
	{
		vtkErrorMacro("Input PolyLine A is empty.");
		return 0;
	}

	if (inputPolyLineA->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input PolyLine A does not contain any point structure.");
		return 0;
	}

	if (inputPolyLineA->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input PolyLine A contains no points.");
		return 0;
	}

	if (inputPolyLineB == nullptr)
	{
		vtkErrorMacro("Input PolyLine B is empty.");
		return 0;
	}

	if (inputPolyLineB->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input PolyLine B does not contain any point structure.");
		return 0;
	}

	if (inputPolyLineB->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input PolyLine B contains no points.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	// A polygon is a closed chain of edges. Several algorithms are available for polygons. 
	// For some of those algorithms, it is necessary that the polygon is simple. 
	// A polygon is simple if edges don't intersect, except consecutive edges, which intersect in their common vertex.
	// Taken from https://doc.cgal.org/4.14.3/Polygon/index.html
	Polygon_2 polygonA, polygonB;

	vtkCGALUtilities::vtkPolyDataToPolygon2(inputPolyLineA, polygonA);
	vtkCGALUtilities::vtkPolyDataToPolygon2(inputPolyLineB, polygonB);

	if (this->DebugMode)
	{
		vtkCGALUtilities::PrintPolygonProperties(polygonA, "Polygon A");
		vtkCGALUtilities::PrintPolygonProperties(polygonB, "Polygon B");
	}

	if (this->InvertPolyLineAOrientation)
		polygonA.reverse_orientation();
	
	if (this->InvertPolyLineBOrientation)
		polygonB.reverse_orientation();

	if (this->ForcePolyLineAOrientation)
	{
		CGAL::Orientation orient = polygonA.orientation();

		switch (this->PolyLineAOrientation)
		{
		case vtkCGALBoolean2DMesher::PolygonOrientations::CLOCKWISE:
		{
			if (orient == CGAL::COUNTERCLOCKWISE) { polygonA.reverse_orientation(); }
			break;
		}
		case vtkCGALBoolean2DMesher::PolygonOrientations::COUNTERCLOCKWISE:
		{
			if (orient == CGAL::CLOCKWISE) { polygonA.reverse_orientation(); }
			break;
		}
		default:
		{
			vtkErrorMacro("Unknown polygon orientation for PolyLine A.");
			return 0;
		}
		}
	}

	if (this->ForcePolyLineBOrientation)
	{
		CGAL::Orientation orient = polygonB.orientation();

		switch (this->PolyLineBOrientation)
		{
		case vtkCGALBoolean2DMesher::PolygonOrientations::CLOCKWISE:
		{
			if (orient == CGAL::COUNTERCLOCKWISE) { polygonB.reverse_orientation(); }
			break;
		}
		case vtkCGALBoolean2DMesher::PolygonOrientations::COUNTERCLOCKWISE:
		{
			if (orient == CGAL::CLOCKWISE) { polygonB.reverse_orientation(); }
			break;
		}
		default:
		{
			vtkErrorMacro("Unknown polygon orientation for PolyLine B.");
			return 0;
		}
		}
	}
	
	if (this->DebugMode)
	{
		CGAL::Orientation finalOrientA = polygonA.orientation();
		cout << "Orient A: " << finalOrientA << endl;
		CGAL::Orientation finalOrientB = polygonB.orientation();
		cout << "Orient B: " << finalOrientB << endl;
	}

	try
	{
		if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::JOIN)
		{
			Polygon_with_holes_2 result;
			CGAL::join(polygonA, polygonB, result);
			vtkCGALUtilities::PolygonWithHoles2ToPolyData(result, output0);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPolygonWithHoles2Properties(result);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::INTERSECTION)
		{
			Pwh_list_2 result;
			CGAL::intersection(polygonA, polygonB, std::back_inserter(result));
			vtkCGALUtilities::PwhList2ToPolyData(result, output0);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::DIFFERENCE)
		{
			Pwh_list_2 result;
			CGAL::difference(polygonA, polygonB, std::back_inserter(result));
			vtkCGALUtilities::PwhList2ToPolyData(result, output0);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::SYMMETRIC_DIFFERENCE)
		{
			Pwh_list_2 result;
			CGAL::symmetric_difference(polygonA, polygonB, std::back_inserter(result));
			vtkCGALUtilities::PwhList2ToPolyData(result, output0);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result);
		}
	}
	catch (const std::exception& e)
	{
		vtkErrorMacro(<< "Error caught : " << e.what());
		return 0;
	}
	
	return 1;
}

