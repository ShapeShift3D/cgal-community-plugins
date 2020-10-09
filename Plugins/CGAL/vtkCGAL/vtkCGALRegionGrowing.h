#ifndef vtkCGALRegionGrowing_h
#define vtkCGALRegionGrowing_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALRegionGrowing : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALRegionGrowing* New();
  vtkTypeMacro(vtkCGALRegionGrowing, vtkPolyDataAlgorithm);

  vtkGetMacro(MaxDistanceToPlane, int);
  vtkSetMacro(MaxDistanceToPlane, int);

  vtkGetMacro(MaxAcceptedAngle, int);
  vtkSetMacro(MaxAcceptedAngle, int);

  vtkGetMacro(MinRegionSize, int);
  vtkSetMacro(MinRegionSize, int);

protected:
  vtkCGALRegionGrowing();
  ~vtkCGALRegionGrowing() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  int MaxDistanceToPlane;
  int MaxAcceptedAngle;
  int MinRegionSize;

private:
  vtkCGALRegionGrowing(const vtkCGALRegionGrowing&) = delete;
  void operator=(const vtkCGALRegionGrowing&) = delete;
};
#endif
