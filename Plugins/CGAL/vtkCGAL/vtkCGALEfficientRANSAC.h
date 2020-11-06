#ifndef vtkCGALEfficientRANSAC_h
#define vtkCGALEfficientRANSAC_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALEfficientRANSAC : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALEfficientRANSAC* New();
  vtkTypeMacro(vtkCGALEfficientRANSAC, vtkPolyDataAlgorithm);

  vtkGetMacro(NumberOfIterations, int);
  vtkSetClampMacro(NumberOfIterations, int, 1, VTK_INT_MAX);

  vtkGetMacro(UseParameters, int);
  vtkSetClampMacro(UseParameters, int, 0, 1);

  vtkGetMacro(MinPoints, int);
  vtkSetClampMacro(MinPoints, int, 0, VTK_INT_MAX);

  vtkGetMacro(Probability, double);
  vtkSetClampMacro(Probability, double, 0.0, 1.0);

  vtkGetMacro(Epsilon, double);
  vtkSetClampMacro(Epsilon, double, 0, VTK_DOUBLE_MAX);

  vtkGetMacro(ClusterEpsilon, double);
  vtkSetClampMacro(ClusterEpsilon, double, 0, VTK_DOUBLE_MAX);

  vtkGetMacro(MaxNormalDeviation, double);
  vtkSetClampMacro(MaxNormalDeviation, double, 0.0, 1.0);

protected:
  vtkCGALEfficientRANSAC();
  ~vtkCGALEfficientRANSAC() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

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
