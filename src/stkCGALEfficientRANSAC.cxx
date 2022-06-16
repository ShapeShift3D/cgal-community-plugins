#include <stkCGALEfficientRANSAC.h>

// -- VTK
#include <vtkCellData.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkBoundingBox.h>
#include <vtkStaticPointLocator.h>

// -- stkCGAL
#include <stkCGALUtilities.h>

// -- CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
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

  if (!this->PointNormalsArrayName.empty())
  {
    auto pointNormalsArray = input->GetPointData()->GetArray(this->PointNormalsArrayName.c_str());

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
    vtkErrorMacro("Point Normals Array Name is not selected");
    return 0;
  }

  switch (this->KernelValue)
  {
    case KernelEnum::EPIC:
    {
      return this->Detection<CGAL::Exact_predicates_inexact_constructions_kernel>(input, output);
    }
    case KernelEnum::Cartesian:
    {
      return this->Detection<CGAL::Cartesian<double> >(input, output);
    }
    case KernelEnum::Simple_cartesian:
    {
      return this->Detection<CGAL::Simple_cartesian<double> >(input, output);
    }
    default:
    {
      vtkErrorMacro("Wrong kernel value.");
      return 0;
    }
  }
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

  // Points with normals
  CGalOrientedPointVector points;
  if (!stkCGALUtilities::vtkPolyDataToOrientedPoints<CGalKernel, CGalOrientedPointVector>(
        input, points))
  {
    vtkErrorMacro("Failed to convert input mesh. Make sure to supplied points normals array is valid.");
    return 0;
  }

  vtkIdType numPoints = input->GetNumberOfPoints();

  // Instantiate shape detection engine
  Efficient_ransac ransac;

  // Provide input data
  ransac.set_input(points);

  // Register planar shapes via template method
  ransac.template add_shape_factory<CGALPlane>();

  // Set parameters for shape detection.
  typename Efficient_ransac::Parameters parameters;

  vtkBoundingBox boundingBox;
  boundingBox.SetBounds(input->GetBounds());
  
  double bbDiagonal = boundingBox.GetDiagonalLength();

  // Set probability to miss the largest primitive at each iteration
  parameters.probability = this->Probability;

  // Detect shapes with at least a certain amount of points

  if (this->MinPointsInputType == MinPointsInputTypes::MIN_POINTS_PERCENTAGE)
  {
    if (this->MinPoints < 0 || this->MinPoints > 100)
    {
      vtkErrorMacro("MinPoints Percentage should be between 0 and 100 percent");
      return 0;
    }
     parameters.min_points =  static_cast<int>((this->MinPoints/100.0)*numPoints); 
  }
  else if (this->MinPointsInputType == MinPointsInputTypes::MIN_POINTS_NUMBER)
  {
    parameters.min_points = this->MinPoints;
  }
 
  // Set maximum Euclidean distance between a point and a shape
  if(this->EpsilonInputType == EpsilonInputTypes::EPSILON_PERCENTAGE)
  {
    if (this->Epsilon < 0 || this->Epsilon > 100)
    {
      vtkErrorMacro("Epsilon Percentage should be between 0 and 100 percent");
      return 0;
    }
    parameters.epsilon = (this->Epsilon/100.0)*bbDiagonal;
  }
  else if (this->EpsilonInputType == EpsilonInputTypes::EPSILON_VALUE)
  {
    parameters.epsilon = this->Epsilon;
  }

  // Set maximum Euclidean distance between points to be clustered
  if(this->ClusterEpsilonInputType == ClusterEpsilonInputTypes::CLUSTER_EPSILON_PERCENTAGE)
  {
    if (this->ClusterEpsilon < 0 || this->ClusterEpsilon > 100)
    {
      vtkErrorMacro("ClusterEpsilon Percentage should be between 0 and 100 percent");
      return 0;
    }
    parameters.cluster_epsilon = (this->ClusterEpsilon/100.0)*bbDiagonal;
  }
  else if (this->ClusterEpsilonInputType == ClusterEpsilonInputTypes::CLUSTER_EPSILON_VALUE)
  {
    parameters.cluster_epsilon = this->ClusterEpsilon;
  }



  // Set maximum normal deviation.
  // MaxNormalDeviation < dot(surface_normal, point_normal);
  parameters.normal_threshold = this->MaxNormalThreshold;
  
  // Build internal data structures
  ransac.preprocess();

  // Efficient_ransac::planes() provides
  // an iterator range to the detected planes.
  // TODO : Add to following to Doc 
  //  Depending on the
  //     chosen probability for the detection, the planes are ordered
  //     with decreasing size.
  Efficient_ransac::Plane_range planes = ransac.planes();

  if (this->NumberOfRuns == 0 )
  {
    vtkErrorMacro("Efficient RANSAC not executed becayse Number of Runs is set to 0");
    return 0;
  }

  // Perform detection several times and choose result with the highest coverage
  FT best_coverage = 0;
  for (std::size_t i = 0; i < this->NumberOfRuns; ++i)
  {
    // Detect shapes
    ransac.detect(parameters);

    Efficient_ransac::Plane_range ithRunPlanes = ransac.planes();

    // Compute coverage, i.e. ratio of the points assigned to a shape
    FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());

    // Print number of assigned shapes and unassigned points
    vtkWarningMacro(<< "Run #" << i << " | "
                    <<ithRunPlanes.end() - ithRunPlanes.begin() << " planes "
                    << " | " << coverage << " coverage");

    vtkWarningMacro("Probabilty=" << parameters.probability << ", MinPoints="
                                  << parameters.min_points << ", epsilon=" << parameters.epsilon
                                  << ", cluster_epsilon=" << parameters.cluster_epsilon
                                  << ", normal_threshold=" << parameters.normal_threshold);

    // Choose result with the highest coverage
    if (coverage > best_coverage)
    {
      best_coverage = coverage;
      planes = ithRunPlanes;
    }
  }

  // Print number of detected shapes
  vtkWarningMacro(<< planes.end() -planes.begin() << " planes detected.");

  auto regionsArray = vtkSmartPointer<vtkIntArray>::New();
  regionsArray->SetName(this->RegionsArrayName.c_str());
  regionsArray->SetNumberOfComponents(1);
  regionsArray->SetNumberOfTuples(numPoints);
  regionsArray->Fill(-1);

  auto distancesArray = vtkSmartPointer<vtkFloatArray>::New();
  if (this->CalculateDistanceFromPlane)
  {
    distancesArray->SetName(this->DistanceFromPlaneArrayName.c_str());
    distancesArray->SetNumberOfComponents(1);
    distancesArray->SetNumberOfTuples(numPoints);
    distancesArray->Fill(-1);
  }

  auto pointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();
  pointLocator->SetDataSet(input);
  pointLocator->BuildLocator();

  int regionIndex = 0;
  typename Efficient_ransac::Plane_range::iterator it = planes.begin();
  while (it != planes.end())
  {
    boost::shared_ptr<CGALShape> plane = *it;

    // Iterate through point indices assigned to each detected shape
    std::vector<std::size_t>::const_iterator index_it = plane->indices_of_assigned_points().begin();

    while (index_it != plane->indices_of_assigned_points().end())
    {
      // Retrieve point.
      const CGalOrientedPoint& p = *(points.begin() + (*index_it));

      double detectedPoint[3] = {CGAL::to_double(p.first.x()),CGAL::to_double(p.first.y()),CGAL::to_double(p.first.z())};
      auto locatedID = pointLocator->FindClosestPoint(detectedPoint);

      double inputPoint[2] = {0.0};
      output->GetPoint(locatedID,inputPoint);

      auto squaredSearchTolerance = std::pow(this->PointSearchTolerance,2);
      if (vtkMath::Distance2BetweenPoints(detectedPoint,inputPoint) < squaredSearchTolerance)
      {
        regionsArray->SetValue(locatedID, regionIndex);
         // Set Euclidean distance between point and shape
        if(this->CalculateDistanceFromPlane)
        {
          auto distance = CGAL::sqrt(plane->squared_distance(p.first));
          distancesArray->SetValue(locatedID, static_cast<float>(distance));
        }
      }
      
      // Proceed with the next point
      index_it++;
    }

    // Proceed with the next detected shape
    it++;
    regionIndex++;
  }

  output->GetPointData()->AddArray(regionsArray);

  // This array is here to help testing
  if (this->CalculateDistanceFromPlane)
  {
    output->GetPointData()->AddArray(distancesArray);      
  }

  return 1;
}

