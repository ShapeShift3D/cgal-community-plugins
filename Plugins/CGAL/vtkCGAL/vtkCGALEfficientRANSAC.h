#ifndef vtkCGALEfficientRANSAC_h
#define vtkCGALEfficientRANSAC_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALEfficientRANSAC : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALEfficientRANSAC* New();
  vtkTypeMacro(vtkCGALEfficientRANSAC, vtkPolyDataAlgorithm);

  //@{
  /**
  * Set/Get NumberOfIterations
  */
  vtkGetMacro(NumberOfIterations, int);
  vtkSetClampMacro(NumberOfIterations, int, 1, VTK_INT_MAX);

  //@{
  /**
  * Set/Get UseParameters
  */
  vtkGetMacro(UseParameters, int);
  vtkSetClampMacro(UseParameters, int, 0, 1);
  //@}

  //@{
  /**
  * Set/Get MinPoints
  */
  vtkGetMacro(MinPoints, int);
  vtkSetClampMacro(MinPoints, int, 0, VTK_INT_MAX);
  //@}

  //@{
  /**
  * Set/Get Probability
  */
  vtkGetMacro(Probability, double);
  vtkSetClampMacro(Probability, double, 0.0, 1.0);
  //@}

  //@{
  /**
  * Set/Get Epsilon
  */
  vtkGetMacro(Epsilon, double);
  vtkSetClampMacro(Epsilon, double, 0, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get ClusterEpsilon
  */
  vtkGetMacro(ClusterEpsilon, double);
  vtkSetClampMacro(ClusterEpsilon, double, 0, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get MaxNormalDeviation
  */
  vtkGetMacro(MaxNormalDeviation, double);
  vtkSetClampMacro(MaxNormalDeviation, double, 0.0, 1.0);
  //@}

protected:
  vtkCGALEfficientRANSAC();
  ~vtkCGALEfficientRANSAC() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  template <class CGalKernel>
  int Detection(vtkPolyData* input, vtkPolyData* output);

  template <class CGalKernel, typename CGalOrientedPointVector>
  static bool vtkPolyDataToOrientedPoints(vtkPolyData* polyData, CGalOrientedPointVector& orientedPoints);

  int NumberOfIterations;
  int UseParameters;
  int MinPoints;

  double Probability;
  double Epsilon;
  double ClusterEpsilon;
  double MaxNormalDeviation;

private:
  vtkCGALEfficientRANSAC(const vtkCGALEfficientRANSAC&) = delete;
  void operator=(const vtkCGALEfficientRANSAC&) = delete;
};
#endif
