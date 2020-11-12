#ifndef vtkCGALRegionGrowing_h
#define vtkCGALRegionGrowing_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALRegionGrowing : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALRegionGrowing* New();
  vtkTypeMacro(vtkCGALRegionGrowing, vtkPolyDataAlgorithm);

  //@{
  /**
  * Set/Get KernelEnum
  */
  typedef enum {
    EPEC = 1,
    EPEC_SQRT,
    EPEC_ROOT,
    EPEC_ROOT_OF,
    EPIC,
    Cartesian,
    Simple_cartesian,
    Homogeneous,        /* Not implemented */
    Simple_homogeneous  /* Not implemented */
  } KernelEnum;

  vtkGetMacro(KernelValue, int);
  vtkSetMacro(KernelValue, int);

  virtual void SetKernelValueToEPEC(void)               { this->SetKernelValue(KernelEnum::EPEC); }
  virtual void SetKernelValueToEPEC_SQRT(void)          { this->SetKernelValue(KernelEnum::EPEC_SQRT); }
  virtual void SetKernelValueToEPEC_ROOT(void)          { this->SetKernelValue(KernelEnum::EPEC_ROOT); }
  virtual void SetKernelValueToEPEC_ROOT_OF(void)       { this->SetKernelValue(KernelEnum::EPEC_ROOT_OF); }
  virtual void SetKernelValueToEPIC(void)               { this->SetKernelValue(KernelEnum::EPIC); }
  virtual void SetKernelValueToCartesian(void)          { this->SetKernelValue(KernelEnum::Cartesian); }
  virtual void SetKernelValueToSimpleCartesian(void)    { this->SetKernelValue(KernelEnum::Simple_cartesian); }
  virtual void SetKernelValueToHomogeneous(void)        { this->SetKernelValue(KernelEnum::Homogeneous); }
  virtual void SetKernelValueToSimpleHomogeneous(void)  { this->SetKernelValue(KernelEnum::Simple_homogeneous); }
  //@}

  //@{
  /**
  * Set/Get MaxDistanceToPlane
  */
  vtkGetMacro(MaxDistanceToPlane, double);
  vtkSetClampMacro(MaxDistanceToPlane, double, 0.0, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get MaxAcceptedAngle
  */
  vtkGetMacro(MaxAcceptedAngle, double);
  vtkSetMacro(MaxAcceptedAngle, double);
  //@}

  //@{
  /**
  * Set/Get MinRegionSize
  */
  vtkGetMacro(MinRegionSize, int);
  vtkSetClampMacro(MinRegionSize, int, 0, VTK_INT_MAX);
  //@}

protected:
  vtkCGALRegionGrowing();
  ~vtkCGALRegionGrowing() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  template <class CGalKernel>
  int Detection(vtkPolyData* input, vtkPolyData* output);

  template <typename MeshType>
  static bool vtkPolyDataToPolygonMesh(vtkPolyData* poly_data, MeshType& tmesh);

  int KernelValue;
  double MaxDistanceToPlane;
  double MaxAcceptedAngle;
  int MinRegionSize;

private:
  vtkCGALRegionGrowing(const vtkCGALRegionGrowing&) = delete;
  void operator=(const vtkCGALRegionGrowing&) = delete;
};
#endif
