#include <stkCGALEfficientRANSAC.h>

// -- VTK
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>
#include <vtkMath.h>
#include <vtkStaticPointLocator.h>

// -- CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>

#include <CGAL/Shape_detection/Efficient_RANSAC.h>

vtkStandardNewMacro(stkCGALEfficientRANSAC);

//----------------------------------------------------------------------------
int stkCGALEfficientRANSAC::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // Get the info objects
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // Get the input and output
  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // Copy input data to the output
  output->DeepCopy(input);

  // TODO : Explicitly ask for Normals array as an Input 
  // Set Array to process 

  // TODO : Check if we can add various kernel options 
  return this->Detection<CGAL::Exact_predicates_inexact_constructions_kernel>(input, output);
}

//----------------------------------------------------------------------------
/** @brief Proceed to detect the regions contained within the input dataset.
 *
 *  @param input       source data structure
 *  @param output      copy of the input data structure with relevant data arrays appended
 *  @tparam CGalKernel must be a CGAL kernel compatible type
 *
 *  @return int Success (1) or failure (0)
 */
template<typename CGalKernel>
int stkCGALEfficientRANSAC::Detection(vtkPolyData* input, vtkPolyData* output)
{
  // Type declarations
  typedef std::pair<typename CGalKernel::Point_3, typename CGalKernel::Vector_3> CGalOrientedPoint;
  typedef std::vector<CGalOrientedPoint> CGalOrientedPointVector;

  typedef typename CGalKernel::FT FT;
  typedef CGAL::First_of_pair_property_map<CGalOrientedPoint> Point_map;
  typedef CGAL::Second_of_pair_property_map<CGalOrientedPoint> Normal_map;

  typedef CGAL::Shape_detection::Efficient_RANSAC_traits<CGalKernel, CGalOrientedPointVector,
    Point_map, Normal_map>
    Traits;
  typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
  typedef CGAL::Shape_detection::Plane<Traits> CGALPlane;
  typedef CGAL::Shape_detection::Shape_base<Traits> CGALShape;

  vtkTimerLog::MarkStartEvent("Preprocessing");

  if (!this->PointNormalsArrayName.empty())
  {
    auto pointNormalsArray = input->GetPointData()->GetArray((this->PointNormalsArrayName.c_str()));

    if (pointNormalsArray != nullptr)
    {
      input->GetPointData()->SetNormals(pointNormalsArray);
    }
    else
    {
      vtkErrorMacro("Selected Point Normals Array is not valid");
      return 0;
    }
  }
  else
  {
    vtkErrorMacro("Point Normals Array is not selected");
    return 0;
  }

  // Points with normals
  CGalOrientedPointVector points;
  if (!stkCGALEfficientRANSAC::vtkPolyDataToOrientedPoints<CGalKernel, CGalOrientedPointVector>(
        input, points))
  {
    vtkErrorMacro("Failed to convert input mesh. Make sure to supplied points normals array is valid.");
    return 0;
  }

  // Instantiate shape detection engine
  Efficient_ransac ransac;

  // Provide input data
  ransac.set_input(points);

  // Register planar shapes via template method
  ransac.template add_shape_factory<CGALPlane>();

  // Set parameters for shape detection.
  typename Efficient_ransac::Parameters parameters;
  // TODO : Add default paramters values to the documentation
  // TODO : Make dropdown for Default/Custom in the UI
  // TODO : Absolute and Percentage Options as applicable 

  if (this->UserDefinedParameters) 
  {
    // Set probability to miss the largest primitive at each iteration
    parameters.probability = this->Probability;

    // Detect shapes with at least a certain amount of points
    parameters.min_points = this->MinPoints;

    // Set maximum Euclidean distance between a point and a shape
    parameters.epsilon = this->Epsilon;

    // Set maximum Euclidean distance between points to be clustered
    parameters.cluster_epsilon = this->ClusterEpsilon;

    // Set maximum normal deviation.
    // MaxNormalDeviation < dot(surface_normal, point_normal);
    parameters.normal_threshold = this->MaxNormalDeviation;
  }

  // Build internal data structures
  ransac.preprocess();

  vtkTimerLog::MarkEndEvent("Preprocessing");

  vtkTimerLog::MarkStartEvent("Detection");

  // Efficient_ransac::shapes() provides
  // an iterator range to the detected shapes.
  // TODO : Add to following to Doc 
  //  Depending on the
  //     chosen probability for the detection, the planes are ordered
  //     with decreasing size.
  Efficient_ransac::Plane_range planes = ransac.planes();


  // Perform detection several times and choose result with the highest coverage
  FT best_coverage = 0;
  for (std::size_t i = 0; i < this->NumberOfIterations; ++i)
  {
    vtkTimerLog::MarkStartEvent("Detection iteration");

    // Detect shapes
    ransac.detect(parameters);

    // TODO : Remove Timer log, we are sending Logs anywhere 
    vtkTimerLog::MarkEndEvent("Detection iteration");

    Efficient_ransac::Plane_range iterationPlanes = ransac.planes();

    // Compute coverage, i.e. ratio of the points assigned to a shape
    FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());

    // Print number of assigned shapes and unassigned points
    vtkWarningMacro(<< "Iteration #" << i << " | "
                    <<iterationPlanes.end() - iterationPlanes.begin() << " planes "
                    << " | " << coverage << " coverage");

    vtkWarningMacro("Probabilty=" << parameters.probability << ", MinPoints="
                                  << parameters.min_points << ", epsilon=" << parameters.epsilon
                                  << ", cluster_epsilon=" << parameters.cluster_epsilon
                                  << ", normal_threshold=" << parameters.normal_threshold);

    // Choose result with the highest coverage
    if (coverage > best_coverage)
    {
      best_coverage = coverage;
      planes = iterationPlanes;
    }
  }

  // Print number of detected shapes
  vtkWarningMacro(<< planes.end() -planes.begin() << " planes detected.");

  vtkTimerLog::MarkEndEvent("Detection");

  vtkIdType numPoints = input->GetNumberOfPoints();

  auto regionsArray = vtkIntArray::New();
  regionsArray->SetName("RANSACRegions");
  regionsArray->SetNumberOfComponents(1);
  regionsArray->SetNumberOfTuples(numPoints);
  regionsArray->Fill(-1);

  auto distancesArray = vtkFloatArray::New();
  distancesArray->SetName("RANSACDistances");
  distancesArray->SetNumberOfComponents(1);
  distancesArray->SetNumberOfTuples(numPoints);
  distancesArray->Fill(-1);

  auto pointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();
  pointLocator->SetDataSet(input);
  pointLocator->BuildLocator();

  int regionIndex = 0;
  typename Efficient_ransac::Plane_range::iterator it = planes.begin();
  while (it != planes.end())
  {
    boost::shared_ptr<CGALShape> plane = *it;

    // Iterate through point indices assigned to each detected shape
    std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();

    while (index_it != (*it)->indices_of_assigned_points().end())
    {
      // Retrieve point.
      const CGalOrientedPoint& p = *(points.begin() + (*index_it));
      //const CGalOrientedPoint& p = *(points.begin() + (*index_it));

      double detectedPoint[3] = {CGAL::to_double(p.first.x()),CGAL::to_double(p.first.y()),CGAL::to_double(p.first.z())};
      auto locatedID = pointLocator->FindClosestPoint(detectedPoint);

      double inputPoint[2] = {0.0};
      output->GetPoint(locatedID,inputPoint);

      // TODO : Expose Point Search Tolerance as an advanced Property 
      // Squared the exposed property 
      if (vtkMath::Distance2BetweenPoints(detectedPoint,inputPoint) < 1.0e-6)
      {
        regionsArray->SetValue(locatedID, regionIndex);
         // Set Euclidean distance between point and shape
        distancesArray->SetValue(locatedID, CGAL::sqrt((*it)->squared_distance(p.first)));
      }

      // Proceed with the next point
      index_it++;
    }

    // Proceed with the next detected shape
    it++;
    regionIndex++;
  }

  output->GetPointData()->AddArray(regionsArray); // TODO : Add option to change the name of the array
  regionsArray->Delete(); // TODO : Make it into a SmartPointer 

  // This array is here to help testing
  output->GetPointData()->AddArray(distancesArray);// TODO : Add option to change the name of the array
  distancesArray->Delete(); // TODO : Make it into a SmartPointer 

  return 1;
}

//----------------------------------------------------------------------------
/** @brief Convert a polydata to a CGAL compatible data structure
 *
 *  @param polyData                  input data structure
 *  @param orientedPoints            output data structure
 *  @tparam CGalKernel               must be a CGAL kernel compatible type
 *  @tparam CGalOrientedPointVector  must be a vector of pairs { Point_3, Vector_3 }
 *                                   using the corresponding CGAL kernel
 *
 *  @return bool Success (true) or failure (false)
 */
template<class CGalKernel, typename CGalOrientedPointVector>
bool stkCGALEfficientRANSAC::vtkPolyDataToOrientedPoints(
  vtkPolyData* polyData, CGalOrientedPointVector& orientedPoints)
{
  using Point = typename CGalKernel::Point_3;
  using Vector = typename CGalKernel::Vector_3;

  vtkDataArray* normals = polyData->GetPointData()->GetNormals();
  if (!normals)
  {
    return false;
  }

  // Extract points and normals
  double p[3], v[3];
  vtkIdType num_points = polyData->GetNumberOfPoints();
  for (vtkIdType i = 0; i < num_points; ++i)
  {
    polyData->GetPoint(i, p);
    normals->GetTuple(i, v);

    orientedPoints.push_back(std::make_pair(Point(p[0], p[1], p[2]), Vector(v[0], v[1], v[2])));
  }

  return true;
}
