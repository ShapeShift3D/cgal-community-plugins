#ifndef vtkCGALRegionGrowing_h
#define vtkCGALRegionGrowing_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALRegionGrowing : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALRegionGrowing* New();
  vtkTypeMacro(vtkCGALRegionGrowing, vtkPolyDataAlgorithm);

  vtkGetMacro(MaxDistanceToPlane, double);
  vtkSetClampMacro(MaxDistanceToPlane, double, 0.0, VTK_DOUBLE_MAX);

  vtkGetMacro(MaxAcceptedAngle, double);
  vtkSetMacro(MaxAcceptedAngle, double);

  vtkGetMacro(MinRegionSize, int);
  vtkSetClampMacro(MinRegionSize, int, 0, VTK_INT_MAX);

protected:
  vtkCGALRegionGrowing();
  ~vtkCGALRegionGrowing() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  double MaxDistanceToPlane;
  double MaxAcceptedAngle;
  int MinRegionSize;

private:
  vtkCGALRegionGrowing(const vtkCGALRegionGrowing&) = delete;
  void operator=(const vtkCGALRegionGrowing&) = delete;
};
#endif
