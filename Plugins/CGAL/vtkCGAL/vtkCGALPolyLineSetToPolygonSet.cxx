/**
* \class vtkCGALPolyLineSetToPolygonSet
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
#include "vtkCGALPolyLineSetToPolygonSet.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCleanPolyData.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALPolyLineSetToPolygonSet);

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALPolyLineSetToPolygonSet::vtkCGALPolyLineSetToPolygonSet()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
  this->Plane = vtkCGALPolyLineSetToPolygonSet::Planes::XY;
  this->DebugMode = true;
}

//---------------------------------------------------
vtkPolyData* vtkCGALPolyLineSetToPolygonSet::GetInputPolyLineSet()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//---------------------------------------------------
Polygon_set_2* vtkCGALPolyLineSetToPolygonSet::GetOutputPolygonSet()
{
	return &PolygonSet;
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALPolyLineSetToPolygonSet::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputPolyLineSet = this->GetInputPolyLineSet();

	if (inputPolyLineSet == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set is empty.");
		return 0;
	}

	if (inputPolyLineSet->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set does not contain any point structure.");
		return 0;
	}

	if (inputPolyLineSet->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input PolyLine Set contains no points.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	this->PolygonSet.clear();
	
	int firstCoordinate = -1;
	int secondCoordinate = -1;
	switch (this->Plane)
	{
	case Planes::XY:
	{
		firstCoordinate = 0;
		secondCoordinate = 1;
		break;
	}
	case Planes::YZ:
	{
		firstCoordinate = 1;
		secondCoordinate = 2;
		break;
	}
	case Planes::XZ:
	{
		firstCoordinate = 0;
		secondCoordinate = 2;
		break;
	}
	default:
	{
		vtkErrorMacro("Unknown Plane.");
		return 0;
	}
	}

	vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
	connectivityFilter->SetInputData(inputPolyLineSet);
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();

	int nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();
	connectivityFilter->SetExtractionModeToSpecifiedRegions();

	vtkNew<vtkCleanPolyData> cleanFilter;
	cleanFilter->SetInputConnection(connectivityFilter->GetOutputPort());

	for (vtkIdType i = 0; i < nbOfPolylines; ++i)
	{
		connectivityFilter->InitializeSpecifiedRegionList();
		connectivityFilter->AddSpecifiedRegion(i);
		cleanFilter->Update();

		Polygon_2 polygon;
		vtkCGALUtilities::vtkPolyDataToPolygon2(cleanFilter->GetOutput(), polygon, 
													firstCoordinate, secondCoordinate);

		

		CGAL::Orientation orient = polygon.orientation();
		if (this->DebugMode)
			vtkCGALUtilities::PrintPolygonProperties(polygon, "Polyline " + std::to_string(i), false);


		if (polygon.is_clockwise_oriented())
		{
			Polygon_with_holes_2 polygonResult;
			polygonResult.add_hole(polygon);
			PolygonSet.insert(polygonResult); // Disjoint polygons only
		}
		else
		{
			PolygonSet.insert(polygon);
		}
	}

	if (this->DebugMode)
	{
		vtkCGALUtilities::PrintPolygonSet2Properties(PolygonSet, "Polygon Set", false);
	}
	
	return 1;
}
