/**
 * \class vtkCGALSurfaceMeshSegmentation
 *
 * \brief This filter takes two inputs, inputMeshA and inputMeshB, of type vtkPolyData and applies
 *one of the four boolean operations (Union, Intersection, Difference1 (A - B) and Difference2 (B -
 *A)) to them. The user will have the option to select one of the four operations from a drop down
 *menu. The two inputs will be converted to CGAL Polygon Mesh class since vtkPolyData is not a valid
 *input. The converted inputs will then be fed into the appropriate function for execution. The
 *result of the function will be converted to a vtkUnstructuredGrid and be outputted as such.
 *
 *
 *
 * Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
 * Output: output (port == 0, vtkUnstructuredGrid)
 *
 */

//---------VTK----------------------------------
#include "vtkCGALSurfaceMeshSegmentation.h"
#include "vtkInformation.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

//---------CGAL---------------------------------
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Polygon_mesh_processing/corefinement.h>
//#include <CGAL/Mesh_3/io_signature.h>
//#include <CGAL/boost/graph/properties.h>
//#include <CGAL/boost/graph/Euler_operations.h>
//#include <CGAL/property_map.h>
//#include <CGAL/IO/Complex_3_in_triangulation_3_to_vtk.h>
//#include <CGAL/boost/graph/io.h>
//

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <vtkIdFilter.h>
#include <vtkThreshold.h>

//---------Boost--------------------------------------------------
//#include <boost/graph/graph_traits.hpp>
//#include <boost/unordered_map.hpp>

//---------Module--------------------------------------------------
#include "vtkSMPTools.h"
#include <vtkCGALUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALSurfaceMeshSegmentation);

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;


//----------------------------------------------------------------------------
namespace stk
{
class MeanFunctor
{
public:
  MeanFunctor(vtkDataSet* InputDataset, vtkDoubleArray* averageOutputArray)
    : InputDataset(InputDataset)
    , AverageOutputArray(averageOutputArray)
  {
  }

  void operator()(vtkIdType begin, vtkIdType end)
  {
    
      for (vtkIdType ptId = begin; ptId < end; ++ptId)
    {
        vtkNew<vtkThreshold> thresholdFilter;
        thresholdFilter->SetInputData(this->InputDataset);
        thresholdFilter->SetInputArrayToProcess(
          0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "segmentation");
        thresholdFilter->ThresholdBetween(ptId, ptId);
        thresholdFilter->Update();

        vtkUnstructuredGrid* zone = thresholdFilter->GetOutput();

        auto sdfZoneData = vtkArrayDownCast<vtkDoubleArray>(zone->GetCellData()->GetArray("sdf"));
        auto originalIdArray =
          vtkArrayDownCast<vtkIdTypeArray>(zone->GetCellData()->GetArray("originalIdArray"));
        vtkIdType nbOfValues = sdfZoneData->GetNumberOfValues();

        double total = 0.0;
        for (int y = 0; y < nbOfValues; ++y)
        {
          total += sdfZoneData->GetValue(y);
        }

        double zoneMean = total / double(nbOfValues);

        for (int id = 0; id < nbOfValues; ++id)
        {
          this->AverageOutputArray->SetValue(originalIdArray->GetValue(id), zoneMean);
        }
    }


  }

private:
  vtkDataSet* InputDataset;
  vtkDoubleArray* AverageOutputArray;
};
} // anonymous namespace

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALSurfaceMeshSegmentation::vtkCGALSurfaceMeshSegmentation()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);

  this->NumberOfClusters = 5;
  this->SmoothingLambda = 0.26;
}

//---------------------------------------------------
vtkPolyData* vtkCGALSurfaceMeshSegmentation::GetInputMesh()
{
  if (this->GetNumberOfInputConnections(0) < 1)
  {
    return nullptr;
  }

  return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
int vtkCGALSurfaceMeshSegmentation::FillOutputPortInformation(int, vtkInformation* info)
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
int vtkCGALSurfaceMeshSegmentation::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputMesh = this->GetInputMesh();

  if (inputMesh == nullptr || inputMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("No points found in input point set.");
    return 0;
  }

  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  auto output = vtkUnstructuredGrid::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkNew<vtkUnstructuredGrid> tempDataSet;

  Surface_Mesh mesh;
  vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, mesh);

  //------------------------------------------------------------------------------------------
  if (!CGAL::is_triangle_mesh(mesh))
  {
    std::cerr << "Input is not a triangle mesh" << std::endl;
    return EXIT_FAILURE;
  }
  typedef Mesh::Property_map<face_descriptor, double> Facet_double_map;
  Facet_double_map sdf_property_map;
  sdf_property_map = mesh.add_property_map<face_descriptor, double>("f:sdf").first;
  // compute SDF values
  // We can't use default parameters for number of rays, and cone angle
  // and the postprocessing
  //
  CGAL::sdf_values(mesh, sdf_property_map, 2.0 / 3.0 * CGAL_PI, 25, true);
  // create a property-map for segment-ids
  typedef Mesh::Property_map<face_descriptor, std::size_t> Facet_int_map;
  Facet_int_map segment_property_map =
    mesh.add_property_map<face_descriptor, std::size_t>("f:sid").first;
  ;
  // segment the mesh using default parameters for number of levels, and smoothing lambda
  // Any other scalar values can be used instead of using SDF values computed using the CGAL
  // function

  // const std::size_t number_of_clusters = 4; // use 4 clusters in soft clustering
  // const double smoothing_lambda =
  //  0.3; // importance of surface features, suggested to be in-between [0,1]
  // default number of clusters =5
  // default smoothing_lambda = 0.26

  std::size_t number_of_segments = CGAL::segmentation_from_sdf_values(
    mesh, sdf_property_map, segment_property_map, this->NumberOfClusters, this->SmoothingLambda);
  std::cout << "Number of segments: " << number_of_segments << std::endl;

  // segmentation values ---------------------------------------------------

  auto numbeOfCells = num_faces(mesh);

  auto segmentData = vtkSmartPointer<vtkIntArray>::New();
  segmentData->SetName("segmentation");
  segmentData->SetNumberOfValues(numbeOfCells);

  auto sdfData = vtkSmartPointer<vtkDoubleArray>::New();
  sdfData->SetName("sdf");
  sdfData->SetNumberOfValues(numbeOfCells);

  for (face_descriptor fd : faces(mesh))
  {
    sdfData->SetValue(fd.idx(), sdf_property_map[fd]);
    segmentData->SetValue(fd.idx(), segment_property_map[fd]);
    // ids are between [0, number_of_segments -1]
    // std::cout << segment_property_map[fd] << " ";
  }

  // TODO: add sdf mean to region

  // std::cout << std::endl;

  // output ------------------------------------------------------------
  vtkCGALUtilities::PolygonMeshToVtkUnstructuredGrid(mesh, tempDataSet);
  // output->Squeeze();

  vtkNew<vtkIdFilter> idFilter;
  idFilter->SetInputData(tempDataSet);
  idFilter->SetCellIdsArrayName("originalIdArray");
  idFilter->Update();

  auto datasetOriginalIds = idFilter->GetOutput();
  datasetOriginalIds->GetCellData()->AddArray(segmentData);
  datasetOriginalIds->GetCellData()->AddArray(sdfData);

  // Add sdf average

  auto nbSegRegions = segmentData->GetNumberOfValues();

  vtkNew<vtkDoubleArray> averageRegionArray;
  averageRegionArray->SetName("average");
  averageRegionArray->SetNumberOfValues(numbeOfCells);

  stk::MeanFunctor meanFunctor(datasetOriginalIds, averageRegionArray);
  vtkSMPTools::For(0, number_of_segments, meanFunctor);

  /*for (int i = 0; i < nbSegRegions; ++i)
  {

    cout << "i " << i << endl;

    vtkNew<vtkThreshold> thresholdFilter;
    thresholdFilter->SetInputData(datasetOriginalIds);
    thresholdFilter->SetInputArrayToProcess(
      0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "segmentation");
    thresholdFilter->ThresholdBetween(i, i);
    thresholdFilter->Update();

    vtkUnstructuredGrid* zone = thresholdFilter->GetOutput();

    auto sdfZoneData = vtkArrayDownCast<vtkDoubleArray>(zone->GetCellData()->GetArray("sdf"));
    auto originalIdArray =
      vtkArrayDownCast<vtkIdTypeArray>(zone->GetCellData()->GetArray("originalIdArray"));
    vtkIdType nbOfValues = sdfZoneData->GetNumberOfValues();

    double total = 0.0;
    for (int y = 0; y < nbOfValues; ++y)
    {
      total += sdfZoneData->GetValue(y);
    }

    double zoneMean = total / nbOfValues;

    for (int id = 0; id < nbOfValues; ++id)
    {
      averageRegionArray->SetValue(originalIdArray->GetValue(id), zoneMean);
    }
  }*/

  output->DeepCopy(datasetOriginalIds);
  output->GetCellData()->AddArray(averageRegionArray);

  return 1;
}