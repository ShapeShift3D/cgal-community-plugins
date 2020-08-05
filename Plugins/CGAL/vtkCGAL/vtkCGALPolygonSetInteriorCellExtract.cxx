/**
* \class vtkCGALPolygonSetInteriorCellExtract
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
#include "vtkCGALPolygonSetInteriorCellExtract.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkIdTypeArray.h>

#include <vtkCellCenters.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkGeometryFilter.h>

//---------CGAL---------------------------------
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>
#include <vtkCGALPolyLineSetToPolygonSet.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALPolygonSetInteriorCellExtract);

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
vtkCGALPolygonSetInteriorCellExtract::vtkCGALPolygonSetInteriorCellExtract()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
  this->Criterion = vtkCGALPolygonSetInteriorCellExtract::Criteria::CENTROID;
  this->Plane = vtkCGALPolygonSetInteriorCellExtract::Planes::XY;
  this->DebugMode = true;
}

//---------------------------------------------------
vtkPolyData* vtkCGALPolygonSetInteriorCellExtract::GetInputPolyLineSet()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALPolygonSetInteriorCellExtract::GetInputCells()
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
int vtkCGALPolygonSetInteriorCellExtract::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputPolyLineSet = this->GetInputPolyLineSet();
	vtkPolyData* inputCells = this->GetInputCells();

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

	if (inputCells == nullptr)
	{
		vtkErrorMacro("Input Cells is empty.");
		return 0;
	}

	if (inputCells->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Cells does not contain any point structure.");
		return 0;
	}

	if (inputCells->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Cells contains no points.");
		return 0;
	}

	if (inputCells->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Cells does not contain any cell structure.");
		return 0;
	}

	if (inputCells->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Cells contains no cells.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));
	
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

	int orientedSideValue = -1;
	switch (this->OrientedSide)
	{
	case OrientedSides::INSIDE:
	{
		orientedSideValue = 1;
		break;
	}
	case OrientedSides::OUTSIDE:
	{
		orientedSideValue = -1;
		break;
	}
	case OrientedSides::BOUNDARY:
	{
		orientedSideValue = 0;
		break;
	}
	default:
	{
		vtkErrorMacro("Unknown Oriented Side.");
		return 0;
	}
	}

	vtkNew<vtkCGALPolyLineSetToPolygonSet> polylineSetToPolygonSetFilter;
	polylineSetToPolygonSetFilter->SetPlane(this->Plane);
	polylineSetToPolygonSetFilter->SetInputData(0, inputPolyLineSet);
	polylineSetToPolygonSetFilter->Update();

	Polygon_set_2 polygonSet = *polylineSetToPolygonSetFilter->GetOutputPolygonSet();

	if (this->DebugMode)
	{
		vtkCGALUtilities::PrintPolygonSet2Properties(polygonSet, "Polygon Set", false);
	}

	if (this->Criterion == vtkCGALPolygonSetInteriorCellExtract::Criteria::CENTROID)
	{
		vtkNew<vtkSelectionNode> selectionNode;
		selectionNode->SetFieldType(vtkSelectionNode::SelectionField::CELL);
		selectionNode->SetContentType(vtkSelectionNode::SelectionContent::INDICES);

		vtkNew<vtkIdTypeArray> selectionListArray;
		selectionListArray->SetName("SelectionList");
		selectionListArray->SetNumberOfComponents(1);

		vtkNew<vtkCellCenters> generateCentersFilter;
		generateCentersFilter->SetInputData(0, inputCells);
		generateCentersFilter->Update();

		vtkPolyData* centroids = generateCentersFilter->GetOutput();

		double centroid[3] = { 0.0, 0.0, 0.0 };
		for (vtkIdType i = 0; i < centroids->GetNumberOfPoints(); ++i)
		{
			centroids->GetPoint(i, centroid);

			if (polygonSet.oriented_side(Point_2(centroid[firstCoordinate], centroid[secondCoordinate])) == orientedSideValue)
			{
				selectionListArray->InsertNextTuple1(i); // Assuming cellCenters conserves order.
			}
		}

		selectionNode->SetSelectionList(selectionListArray);

		vtkNew<vtkSelection> selection;
		selection->AddNode(selectionNode);

		vtkNew<vtkExtractSelection> extractCellsFilter;
		extractCellsFilter->SetInputData(0, inputCells);
		extractCellsFilter->SetInputData(1, selection);

		vtkNew<vtkGeometryFilter> UGToPoly;
		UGToPoly->SetInputConnection(extractCellsFilter->GetOutputPort());
		UGToPoly->Update();

		output0->ShallowCopy(UGToPoly->GetOutput());
	}
	
	return 1;
}

