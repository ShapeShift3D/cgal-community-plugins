/**
* \class vtkCGAL2DPolygonParentTree
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
#include "vtkCGAL2DPolygonParentTree.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>

#include <vtkBox.h>
#include <vtkImplicitDataSet.h>
#include <vtkElevationFilter.h>

#include <vtkExtractPolyDataGeometry.h>

#include <vtkIdFilter.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkThreshold.h>

//---------CGAL---------------------------------
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Bbox_2.h>

//---------Module--------------------------------------------------
#include <vtkCGALUtilities.h>

#include <functional>
#include <set>
#include <vector>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGAL2DPolygonParentTree);

typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2											Point_2;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>						Pwh_list_2;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGAL2DPolygonParentTree::vtkCGAL2DPolygonParentTree()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
  this->Plane = vtkCGAL2DPolygonParentTree::Planes::XY;
  this->DebugMode = true;
}

//---------------------------------------------------
vtkPolyData* vtkCGAL2DPolygonParentTree::GetInputMesh()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGAL2DPolygonParentTree::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputMesh = this->GetInputMesh();

	if (inputMesh == nullptr)
	{
		vtkErrorMacro("Input Mesh is empty.");
		return 0;
	}

	if (inputMesh->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Mesh does not contain any cell structure.");
		return 0;
	}

	if (inputMesh->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Mesh contains no cells.");
		return 0;
	}

	if (inputMesh->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Mesh does not contain any point structure.");
		return 0;
	}

	if (inputMesh->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Mesh contains no points.");
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

	std::string cellIdsArrayName = "cellIds";

	vtkNew<vtkIdFilter> generateIds;
	generateIds->SetInputData(0, inputMesh);
	generateIds->PointIdsOff();
	generateIds->CellIdsOn();
	generateIds->SetCellIdsArrayName(cellIdsArrayName.c_str());
	generateIds->FieldDataOff();
	generateIds->Update();

	vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
	connectivityFilter->SetInputData(generateIds->GetOutput());
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();

	int nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();
	connectivityFilter->SetExtractionModeToSpecifiedRegions();

	vtkNew<vtkCleanPolyData> cleanFilter;
	cleanFilter->SetInputConnection(connectivityFilter->GetOutputPort());

	double pt[3] = { 0.0, 0.0, 0.0 };

	std::vector<std::pair<int, std::set<int>>> siblings;
	std::vector<std::pair<int, std::set<int>>> nodeCellIds;

	for (vtkIdType i = 0; i < nbOfPolylines; ++i)
	{
		connectivityFilter->InitializeSpecifiedRegionList();
		connectivityFilter->AddSpecifiedRegion(i);
		cleanFilter->Update();

		vtkPolyData* polyline = vtkPolyData::SafeDownCast(cleanFilter->GetOutput());
		vtkIdTypeArray* cellIdArray = vtkArrayDownCast<vtkIdTypeArray>(polyline->GetCellData()
																		->GetAbstractArray(cellIdsArrayName.c_str()));

		// TODO: Check if polyline is well formed.

		// Mark cellIds associated to node
		std::set<int> cellIds;
		for (vtkIdType j = 0; j < polyline->GetNumberOfCells(); ++j)
		{
			cellIds.insert(cellIdArray->GetTuple1(j));
		}
		nodeCellIds.push_back(std::make_pair(i, cellIds));

		// Convert to CGAL Polygon
		Polygon_2 polygon;
		vtkCGALUtilities::vtkPolyDataToPolygon2(polyline, polygon);

		std::set<int> descendants;

		// Find all polylines inside the ith polyline
		// Assuming that different polylines do not cross each other
		// TODO: Optimize algorithm for something better than n^2
		for (vtkIdType j = 0; j < nbOfPolylines; ++j)
		{
			connectivityFilter->InitializeSpecifiedRegionList();
			connectivityFilter->AddSpecifiedRegion(j);
			cleanFilter->Update();

			cleanFilter->GetOutput()->GetPoint(0, pt);
			if (i == 3)
			{
				std::cout << "pt: " << pt[0] << ", " << pt[1] << ", " << pt[2] << endl;
				std::cout << "bounded side result: " << polygon.bounded_side(Point_2(pt[firstCoordinate], pt[secondCoordinate])) << endl;
			}

			if (polygon.bounded_side(Point_2(pt[firstCoordinate], pt[secondCoordinate])) == CGAL::ON_BOUNDED_SIDE) // is inside
			{
				descendants.insert(j);
			}
		}

		siblings.push_back(std::make_pair(i, descendants));
	}

	// Debug info on sibling relationships
	if (this->DebugMode)
	{
		std::cout << "===== CELL IDs =====" << endl;
		for (unsigned int i = 0; i < nodeCellIds.size(); ++i)
		{
			std::cout << "Polyline: " << nodeCellIds[i].first << endl;
			std::cout << "Cell IdS : {";
			for (int const& cellId : nodeCellIds[i].second)
			{
				std::cout << cellId << ", ";
			}
			std::cout << "}" << endl;
		}
	}

	// Cell Ids
	if (this->DebugMode)
	{
		std::cout << "===== Cell IDs =====" << endl;
		for (unsigned int i = 0; i < siblings.size(); ++i)
		{
			std::cout << "Node: " << siblings[i].first << endl;
			std::cout << "Descendants : {";
			for (int const& sibling : siblings[i].second)
			{
				std::cout << sibling << ", ";
			}
			std::cout << "}" << endl;
		}
	}

	// Find children -> parent relationships
	std::vector<std::pair<int, int>> parentChildrenRelationship;
	int iter = 0;
	while (siblings.size() > 0)
	{
		// Remove empty relationships
		for (auto it = siblings.begin(); it != siblings.end(); )
		{
			if ((*it).second.size() == 0)
				siblings.erase(it);
			else
				++it;
		}

		// Process 1-to-1 relationships
		for (unsigned int i = 0; i < siblings.size(); ++i)
		{
			// Assumption: Every children has only one parent.
			if (siblings[i].second.size() == 1)
			{
				std::set<int>& relationship = siblings[i].second;
				int parent = *relationship.begin();
				parentChildrenRelationship.push_back(std::make_pair(parent, siblings[i].first));

				// Remove the parent from all sets
				for (unsigned int j = 0; j < siblings.size(); ++j)
				{
					std::set<int>& relationship = siblings[j].second;
					std::set<int>::iterator it = relationship.find(parent);
					if (it != relationship.end()) {
						relationship.erase(it);
					}
				}
			}
		}

		// Debug info on sibling relationships
		if (this->DebugMode)
		{
			std::cout << "===== ITER " << iter << " =====" << endl;
			for (unsigned int i = 0; i < siblings.size(); ++i)
			{
				std::cout << "Node: " << siblings[i].first << endl;
				std::cout << "Descendants : {";
				for (int const& sibling : siblings[i].second)
				{
					std::cout << sibling << ", ";
				}
				std::cout << "}" << endl;
			}
		}

		++iter;
	}

	// Debug info on parent-children relationships
	if (this->DebugMode)
	{
		std::cout << "===== PARENT-CHILDREN REL =====" << endl;
		for (unsigned int i = 0; i < parentChildrenRelationship.size(); ++i)
		{
			std::cout << "Parent: " << parentChildrenRelationship[i].first << endl;
			std::cout << "Children: " << parentChildrenRelationship[i].second << endl;
		}
	}

	// Write to arrays
	vtkNew<vtkIdTypeArray> parentRelationshipArray;
	parentRelationshipArray->SetName(this->ParentRelationshipArrayName.c_str());
	parentRelationshipArray->SetNumberOfComponents(1);
	parentRelationshipArray->SetNumberOfTuples(inputMesh->GetNumberOfCells());
	parentRelationshipArray->Fill(-1);

	vtkNew<vtkIdTypeArray> nodeIdArray;
	nodeIdArray->SetName(this->NodeIdArrayName.c_str());
	nodeIdArray->SetNumberOfComponents(1);
	nodeIdArray->SetNumberOfTuples(inputMesh->GetNumberOfCells());

	vtkNew<vtkThreshold> thresholdFilter;
	thresholdFilter->SetInputData(output0);
	thresholdFilter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, this->NodeIdArrayName.c_str());

	for (unsigned int i = 0; i < nodeCellIds.size(); ++i)
	{
		int& polylineId = nodeCellIds[i].first;
		std::set<int>& cellIds = nodeCellIds[i].second;

		auto parent_it = std::find_if(parentChildrenRelationship.begin(), parentChildrenRelationship.end(),
			[&polylineId](const std::pair<int, int>& element) { return element.first == polylineId; });

		int childId = parent_it != parentChildrenRelationship.end() ? (*parent_it).second : -1;

		for (auto it = cellIds.begin(); it != cellIds.end(); ++it)
		{
			nodeIdArray->SetTuple1(*it, polylineId);
			parentRelationshipArray->SetTuple1(*it, childId);
		}
	}

	output0->DeepCopy(inputMesh);
	output0->GetCellData()->AddArray(nodeIdArray);
	output0->GetCellData()->AddArray(parentRelationshipArray);
	
	return 1;
}

