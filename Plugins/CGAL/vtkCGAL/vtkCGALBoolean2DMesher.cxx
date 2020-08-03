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

#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCleanPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
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
  this->OperationMode = vtkCGALBoolean2DMesher::OperationModes::INTERSECT;
  this->OneCell = true;
  this->DebugMode = true;
}

//---------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputPolyLineSetA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALBoolean2DMesher::GetInputPolyLineSetB()
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
	vtkPolyData* inputPolyLineSetA = this->GetInputPolyLineSetA();
	vtkPolyData* inputPolyLineSetB = this->GetInputPolyLineSetB();

	if (inputPolyLineSetA == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set A is empty.");
		return 0;
	}

	if (inputPolyLineSetA->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set A does not contain any point structure.");
		return 0;
	}

	if (inputPolyLineSetA->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input PolyLine Set A contains no points.");
		return 0;
	}

	if (inputPolyLineSetB == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set B is empty.");
		return 0;
	}

	if (inputPolyLineSetB->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input PolyLine Set B does not contain any point structure.");
		return 0;
	}

	if (inputPolyLineSetB->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input PolyLine Set B contains no points.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
	connectivityFilter->SetInputData(inputPolyLineSetA);
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();

	int nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();
	connectivityFilter->SetExtractionModeToSpecifiedRegions();

	vtkNew<vtkCleanPolyData> cleanFilter;
	cleanFilter->SetInputConnection(connectivityFilter->GetOutputPort());

	Polygon_set_2 polygonSetA;

	for (vtkIdType i = 0; i < nbOfPolylines; ++i)
	{
		connectivityFilter->InitializeSpecifiedRegionList();
		connectivityFilter->AddSpecifiedRegion(i);
		cleanFilter->Update();

		Polygon_2 polygonA;
		vtkCGALUtilities::vtkPolyDataToPolygon2(cleanFilter->GetOutput(), polygonA);

		Polygon_with_holes_2 polygonWithHolesA;

		CGAL::Orientation orientA = polygonA.orientation();

		if (orientA == CGAL::CLOCKWISE)
		{
			polygonWithHolesA.add_hole(polygonA);
		}
		else if (orientA == CGAL::COUNTERCLOCKWISE)
		{
			polygonWithHolesA.outer_boundary() = polygonA;
		}

		polygonSetA.insert(polygonWithHolesA); // Disjoint polygons only
	}

	connectivityFilter->SetInputData(inputPolyLineSetB);
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();

	nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();
	connectivityFilter->SetExtractionModeToSpecifiedRegions();

	Polygon_set_2 polygonSetB;

	for (vtkIdType i = 0; i < nbOfPolylines; ++i)
	{
		connectivityFilter->InitializeSpecifiedRegionList();
		connectivityFilter->AddSpecifiedRegion(i);
		cleanFilter->Update();

		Polygon_2 polygonB;
		vtkCGALUtilities::vtkPolyDataToPolygon2(cleanFilter->GetOutput(), polygonB);

		Polygon_with_holes_2 polygonWithHolesB;

		CGAL::Orientation orientB = polygonB.orientation();

		if (orientB == CGAL::CLOCKWISE)
		{
			polygonWithHolesB.add_hole(polygonB);
		}
		else if (orientB == CGAL::COUNTERCLOCKWISE)
		{
			polygonWithHolesB.outer_boundary() = polygonB;
		}

		polygonSetB.insert(polygonWithHolesB); // Disjoint polygons only
	}

	if (this->DebugMode)
	{
		vtkCGALUtilities::PrintPolygonSet2Properties(polygonSetA, "Polygon Set A", false);
		vtkCGALUtilities::PrintPolygonSet2Properties(polygonSetB, "Polygon Set B", false);
	}

	try
	{
		if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::ADD)
		{
			polygonSetA.join(polygonSetB);

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));

			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::INTERSECT)
		{
			polygonSetA.intersection(polygonSetB);

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));

			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::A_MINUS_B)
		{
			polygonSetA.difference(polygonSetB);

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));

			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::B_MINUS_A)
		{
			polygonSetB.difference(polygonSetA);

			Pwh_list_2 result;
			polygonSetB.polygons_with_holes(std::back_inserter(result));

			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::EXCLUDE_OVERLAP)
		{
			polygonSetA.symmetric_difference(polygonSetB);

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));

			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::COMPLEMENT)
		{
			if (this->ComplementOf == vtkCGALBoolean2DMesher::Inputs::A)
			{
				polygonSetA.complement();

				Pwh_list_2 result;
				polygonSetA.polygons_with_holes(std::back_inserter(result));

				vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
				if (this->DebugMode)
					vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
			}
			else if (this->ComplementOf == vtkCGALBoolean2DMesher::Inputs::B)
			{
				polygonSetB.complement();

				Pwh_list_2 result;
				polygonSetB.polygons_with_holes(std::back_inserter(result));

				vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
				if (this->DebugMode)
					vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
			}
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::INTERSECT_COMPLEMENT)
		{
			// NAND
			polygonSetA.intersection(polygonSetB);
			polygonSetA.complement();

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));
			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
		else if (this->OperationMode == vtkCGALBoolean2DMesher::OperationModes::EXCLUSIVE_ADD)
		{
			// Intersect Complement (NAND)
			Polygon_set_2 S = polygonSetA;
			Polygon_set_2 Q = polygonSetB;
			S.intersection(Q);
			S.complement();

			// OR
			polygonSetA.join(polygonSetB);

			// AND
			polygonSetA.intersection(S);

			Pwh_list_2 result;
			polygonSetA.polygons_with_holes(std::back_inserter(result));
			
			vtkCGALUtilities::PwhList2ToPolyData(result, output0, this->OneCell);
			if (this->DebugMode)
				vtkCGALUtilities::PrintPwhList2Properties(result, "Result", false);
		}
	}
	catch (const std::exception& e)
	{
		vtkErrorMacro(<< "Error caught : " << e.what());
		return 0;
	}
	
	return 1;
}

