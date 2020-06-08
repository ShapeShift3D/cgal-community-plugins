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

#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

//---------Module-------------------------------
#include <vtkCGALUtilities.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;
typedef boost::graph_traits<Surface_Mesh>::face_descriptor face_descriptor;

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

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

	// Iterate over polygon meshes by connectivity
	vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
	connectivityFilter->SetInputData(inputMesh);
	connectivityFilter->SetExtractionModeToAllRegions();
	connectivityFilter->Update();

	vtkNew<vtkCleanPolyData> cleanPolyData;
	cleanPolyData->SetInputConnection(connectivityFilter->GetOutputPort());

	int numberOfRegions = connectivityFilter->GetNumberOfExtractedRegions();

	vtkNew<vtkAppendPolyData> appendFinal;
	namespace PMP = CGAL::Polygon_mesh_processing;

	for (vtkIdType i = 0; i < numberOfRegions; ++i)
	{
		connectivityFilter->SetExtractionModeToSpecifiedRegions();
		connectivityFilter->InitializeSpecifiedRegionList();
		connectivityFilter->AddSpecifiedRegion(i);
		cleanPolyData->Update();

		vtkPolyData* polyData = cleanPolyData->GetOutput();

		if (polyData->GetNumberOfCells() == 0)
			continue;

		Surface_Mesh surfaceMesh;
		vtkCGALUtilities::vtkPolyDataToPolygonMesh(polyData, surfaceMesh);

		if (!CGAL::is_triangle_mesh(surfaceMesh))
		{
			vtkErrorMacro("Mesh is not triangular.");
			return 0;
		}

		bool intersecting = PMP::does_self_intersect(surfaceMesh,
			PMP::parameters::vertex_point_map(get(CGAL::vertex_point, surfaceMesh)));

		std::vector<std::pair<face_descriptor, face_descriptor>> intersected_tris;
		PMP::self_intersections(surfaceMesh, std::back_inserter(intersected_tris));

		if (intersecting)
		{
			vtkErrorMacro("Self-Intersections were found in the mesh (region " << i << "). " <<
				intersected_tris.size() << " pairs of triangles intersect.");
		}

		vtkNew<vtkIntArray> intersectingTrisArray;
		intersectingTrisArray->SetName(this->SelfIntersectionsArrayName.c_str());
		intersectingTrisArray->SetNumberOfComponents(1);
		intersectingTrisArray->SetNumberOfTuples(polyData->GetNumberOfCells());
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
			std::cout << "====== Region " << i << " ======" << std::endl;
			for (vtkIdType i = 0; i < intersected_tris.size(); ++i)
			{
				vtkWarningMacro("Triangle " << intersected_tris[i].first.idx() << " is intersecting with "
					<< intersected_tris[i].second.idx() << ".");
			}
		}

		vtkNew<vtkPolyData> singlePoly;
		singlePoly->DeepCopy(polyData);
		singlePoly->GetCellData()->AddArray(intersectingTrisArray);

		appendFinal->AddInputData(singlePoly);
	}

	appendFinal->Update();

	// Copy to output
	output->DeepCopy(appendFinal->GetOutput());
	
	return 1;
}
