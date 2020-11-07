/**
* \class vtkCGALEfficientRANSAC
* 
* \brief todo
* 
* 
* 
* Inputs: todo
* Output: todo
* 
*/

#include <vtkCGALEfficientRANSAC.h>

// -- VTK
#include <vtkCellData.h>
#include <vtkCGALUtilities.h>
#include <vtkFieldData.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>

// -- CGAL
#include <CGAL/property_map.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Shape_detection/Efficient_RANSAC.h>

// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel                     Kernel;
typedef Kernel::FT                                                              FT;
typedef CGAL::First_of_pair_property_map<vtkCGALUtilities::Point_with_normal0>  Point_map;
typedef CGAL::Second_of_pair_property_map<vtkCGALUtilities::Point_with_normal0> Normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
<Kernel, vtkCGALUtilities::Pwn_vector0, Point_map, Normal_map>  Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits>         Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>                    Plane;

#define ITERATIVE_METHOD

// -----------------------------------------------------------------------------
// Declare the plugin
vtkStandardNewMacro(vtkCGALEfficientRANSAC);

// -----------------------------------------------------------------------------
// Constructor
// TODO: description
vtkCGALEfficientRANSAC::vtkCGALEfficientRANSAC()
{
  this->NumberOfIterations = 1;
  this->UseParameters = 0;
  this->MinPoints = 200;

  this->Probability = 0.05;
  this->Epsilon = 0.002;
  this->ClusterEpsilon = 0.01;
  this->MaxNormalDeviation = 0.9;
}

// -----------------------------------------------------------------------------
// TODO: description
int vtkCGALEfficientRANSAC::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // Get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  // Get the input and output
  vtkPolyData *input = vtkPolyData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData *output = vtkPolyData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));

  output->CopyStructure(input);
  output->GetCellData()->PassData(input->GetCellData());
  output->GetFieldData()->PassData(input->GetFieldData());
  output->GetPointData()->PassData(input->GetPointData());

  // Points with normals.
  vtkCGALUtilities::Pwn_vector0 points;

  if (!vtkCGALUtilities::vtkPolyDataToPointsWithNormals(input, points))
  {
    vtkErrorMacro("Failed to convert input mesh. Make sure to supply points normals.");
    return 0;
  }

  vtkIdType numPoints = input->GetNumberOfPoints();

  auto colors = vtkIntArray::New();
  colors->SetName("regions");
  colors->SetNumberOfComponents(1);
  colors->SetNumberOfTuples(numPoints);
  colors->Fill(-1);

  auto distances = vtkFloatArray::New();
  distances->SetName("distances");
  distances->SetNumberOfComponents(1);
  distances->SetNumberOfTuples(numPoints);
  distances->Fill(-1);

  int regionIndex = 0;

  // Instantiate shape detection engine.
  Efficient_ransac ransac;

  // Provide input data.
  ransac.set_input(points);

  // Register planar shapes via template method.
  ransac.add_shape_factory<Plane>();

  // TODO: Use detect() with callback so we can see the progress

  // Set parameters for shape detection.
  Efficient_ransac::Parameters parameters;
  if (this->UseParameters)
  {
    // Set probability to miss the largest primitive at each iteration.
    parameters.probability = this->Probability;

    // Detect shapes with at least a certain amount of points.
    parameters.min_points = this->MinPoints;

    // Set maximum Euclidean distance between a point and a shape.
    parameters.epsilon = this->Epsilon;

    // Set maximum Euclidean distance between points to be clustered.
    parameters.cluster_epsilon = this->ClusterEpsilon;

    // Set maximum normal deviation.
    // MaxNormalDeviation < dot(surface_normal, point_normal);
    parameters.normal_threshold = this->MaxNormalDeviation;
  }

  vtkTimerLog::MarkStartEvent("Preprocessing");

  // Build internal data structures.
  ransac.preprocess();

  vtkTimerLog::MarkEndEvent("Preprocessing");

  vtkTimerLog::MarkStartEvent("Detection");

  // Efficient_ransac::shapes() provides
  // an iterator range to the detected shapes.
  Efficient_ransac::Shape_range shapes = ransac.shapes();

#ifndef ITERATIVE_METHOD
  // Detect registered shapes with default parameters.
  ransac.detect(parameters);
  shapes = ransac.shapes();
#else
  // Perform detection several times and choose result with the highest coverage.
  FT best_coverage = 0;
  for (std::size_t i = 0; i < this->NumberOfIterations; ++i)
  {
    vtkTimerLog::MarkStartEvent("Detection iteration");

    // Detect shapes.
    ransac.detect(parameters);

    vtkTimerLog::MarkEndEvent("Detection iteration");

    // Compute coverage, i.e. ratio of the points assigned to a shape.
    FT coverage =
      FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());

    // Print number of assigned shapes and unassigned points.
    vtkWarningMacro(<< "Iteration #" << i
      << " | " << ransac.shapes().end() - ransac.shapes().begin() << " primitives"
      << " | " << coverage << " coverage");

    // Choose result with the highest coverage.
    if (coverage > best_coverage)
    {
      best_coverage = coverage;
      shapes = ransac.shapes();
    }
  }
#endif // !ITERATIVE_METHOD

  vtkTimerLog::MarkEndEvent("Detection");

  // Print number of detected shapes.
  vtkWarningMacro(<< ransac.shapes().end() - ransac.shapes().begin() << " shapes detected.");

  Efficient_ransac::Shape_range::iterator it = shapes.begin();
  while (it != shapes.end())
  {
    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;

    // Sums distances of points to the detected shapes.
    FT sum_distances = 0;

    // Iterate through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator
      index_it = (*it)->indices_of_assigned_points().begin();

    while (index_it != (*it)->indices_of_assigned_points().end())
    {
      // Retrieve point.
      const vtkCGALUtilities::Point_with_normal0& p = *(points.begin() + (*index_it));

      // Adds Euclidean distance between point and shape.
      FT tmp_distance = CGAL::sqrt((*it)->squared_distance(p.first));
      sum_distances += tmp_distance;

      colors->SetValue(
        static_cast<vtkIdType>((*index_it)), regionIndex);

      distances->SetValue(
        static_cast<vtkIdType>((*index_it)), tmp_distance);

      // Proceed with the next point.
      index_it++;
    }

    // Compute and print the average distance.
    FT average_distance = sum_distances /
      shape->indices_of_assigned_points().size();

    vtkWarningMacro(<< "Average distance region #"
      << regionIndex << ": " << average_distance);

    // Proceed with the next detected shape.
    it++;
    regionIndex++;
  }

  output->GetPointData()->SetScalars(colors);
  colors->Delete();

  // This array is here to help testing
  output->GetPointData()->AddArray(distances);
  distances->Delete();

  return 1;
}
