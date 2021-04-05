/**
* \class stkCGALPolygonSetToPolyLineSet
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
#include "stkCGALPolygonSetToPolyLineSet.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>

//---------Module--------------------------------------------------
#include <stkCGALPolygonUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(stkCGALPolygonSetToPolyLineSet);

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
stkCGALPolygonSetToPolyLineSet::stkCGALPolygonSetToPolyLineSet()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
  this->Plane = stkCGALPolygonSetToPolyLineSet::Planes::XY;
  this->PwhIdArrayName = "PolygonWithHolesId";
  this->OneCell = true;
  this->DebugMode = false;
  this->PrintPoints = false;
}

//---------------------------------------------------
vtkPolyData* stkCGALPolygonSetToPolyLineSet::GetOutputPolyLineSet()
{
	return vtkPolyData::SafeDownCast(this->GetOutputDataObject(0));
}

//---------------------------------------------------
void stkCGALPolygonSetToPolyLineSet::SetInputPolygonSet(Polygon_set_2& polygonSet)
{
	this->PolygonSet = polygonSet;
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int stkCGALPolygonSetToPolyLineSet::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
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

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	Pwh_list_2 result;
	this->PolygonSet.polygons_with_holes(std::back_inserter(result));
	stkCGALPolygonUtilities::PwhList2ToPolyData(result, output0, this->PwhIdArrayName, this->OneCell);

	if (this->DebugMode)
	{
		stkCGALPolygonUtilities::PrintPolygonSet2Properties(this->PolygonSet, "Polygon Set", this->PrintPoints);
	}
	
	return 1;
}