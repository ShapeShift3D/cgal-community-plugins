/**
 * @class stkCGALEfficientRANSAC
 * @brief Detects plane shaped regions within a vtkPolyData
 * 
 * This filter uses the CGAL efficient RANSAC method.
 * It will append a point data array "regions" filled with the detected
 * region indices or -1 if the point is not assigned to a region.
 * 
 * 
 * In addition, a point data array "distances" filled with the distances to the
 * corresponding plane will be added to the output point data.
 *
 * TODO:
 *      - Wrap detect() using callbacks so the user can see the progress
 *        (see https://doc.cgal.org/latest/Shape_detection/index.html#Shape_detection_RANSACExample_with_callback)
 *      - Add the choice of the kernel as a parameter (see stkCGALRegionGrowing::KernelValue)
 *
 * @warning The input dataset must contain point notmals.
 * 
 * @sa
 * stkCGALRegionGrowing
*/

#ifndef stkCGALEfficientRANSAC_h
#define stkCGALEfficientRANSAC_h

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class STKCGAL_EXPORT stkCGALEfficientRANSAC : public vtkPolyDataAlgorithm
{
public:
  static stkCGALEfficientRANSAC* New();
  vtkTypeMacro(stkCGALEfficientRANSAC, vtkPolyDataAlgorithm);

  //@{
  /**
  * Set/Get UserDefinedParameters. If not checked, the algorithm will
  * evaluate defaultvalues based on the mesh parameters (size, ...).
  * Otherwise the user has the ability define detection parameters.
  */
  vtkSetMacro(UserDefinedParameters, vtkTypeBool);
  vtkGetMacro(UserDefinedParameters, vtkTypeBool);
  vtkBooleanMacro(UserDefinedParameters, vtkTypeBool);
  //@}

  //@{
  /**
  * Set/Get NumberOfIterations. The algorithm will perform detection several times
  * and choose result with the highest coverage (i.e. ratio of the points assigned to a shape).
  */
  vtkGetMacro(NumberOfIterations, int);
  vtkSetClampMacro(NumberOfIterations, int, 1, VTK_INT_MAX);

  //@{
  /**
  * Set/Get MinPoints. The minimum number of points in a shape.
  */
  vtkGetMacro(MinPoints, int);
  vtkSetClampMacro(MinPoints, int, 0, VTK_INT_MAX);
  //@}

  //@{
  /**
  * Set/Get Probability. The probability to miss the largest primitive at each iteration.
  * Probability to control search endurance. A lower probability provides a higher reliability
  * and determinism at the cost of longer running time due to a higher search endurance.
  */
  vtkGetMacro(Probability, double);
  vtkSetClampMacro(Probability, double, 0.0, 1.0);
  //@}

  //@{
  /**
  * Set/Get Epsilon. The maximum acceptable Euclidean distance between a point and a shape.
  */
  vtkGetMacro(Epsilon, double);
  vtkSetClampMacro(Epsilon, double, -1, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get ClusterEpsilon. The maximum acceptable Euclidean distance between points,
  * which are assumed to be neighbors.
  */
  vtkGetMacro(ClusterEpsilon, double);
  vtkSetClampMacro(ClusterEpsilon, double, -1, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get MaxNormalDeviation. The maximum threshold on the dot product between the estimated
  * shape's normal and the point's normal, that is the cosine of the angle (cos(25deg) = 0.9). 
  */
  vtkGetMacro(MaxNormalDeviation, double);
  vtkSetClampMacro(MaxNormalDeviation, double, 0.0, 1.0);
  //@}

protected:
  stkCGALEfficientRANSAC();
  ~stkCGALEfficientRANSAC() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  template <class CGalKernel>
  int Detection(vtkPolyData* input, vtkPolyData* output);

  template <class CGalKernel, typename CGalOrientedPointVector>
  static bool vtkPolyDataToOrientedPoints(vtkPolyData* polyData, CGalOrientedPointVector& orientedPoints);

  vtkTypeBool UserDefinedParameters;

  int NumberOfIterations;
  int MinPoints;

  double Probability;
  double Epsilon;
  double ClusterEpsilon;
  double MaxNormalDeviation;

private:
  stkCGALEfficientRANSAC(const stkCGALEfficientRANSAC&) = delete;
  void operator=(const stkCGALEfficientRANSAC&) = delete;
};
#endif
