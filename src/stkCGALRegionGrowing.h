/**
 * @class stkCGALRegionGrowing
 * @brief Detects plane shaped regions within a vtkPolyData
 * 
 * This filter uses the CGAL region growing method.
 * It will append a cell data array "regions" filled with the detected
 * region indices or -1 if the cell is not assigned to a region.
 * 
 * TODO:
 *      - Add extra kernels if applicable (Homogeneous and Simple_homogeneous)
 *
 * @sa
 * stkCGALEfficientRANSAC
*/

#ifndef stkCGALRegionGrowing_h
#define stkCGALRegionGrowing_h

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class STKCGAL_EXPORT stkCGALRegionGrowing : public vtkPolyDataAlgorithm
{
public:
  static stkCGALRegionGrowing* New();
  vtkTypeMacro(stkCGALRegionGrowing, vtkPolyDataAlgorithm);

  //@{
  /**
  * Set/Get KernelEnum. Depending on the CGAL kernel chosen,
  * the compute time may vary a lot and so the output regions.
  */
  typedef enum {
    EPEC = 1,             // Exact_predicates_exact_constructions_kernel
    EPEC_SQRT,            // Exact_predicates_exact_constructions_kernel_with_sqrt
    EPEC_KTH_ROOT,        // Exact_predicates_exact_constructions_kernel_with_kth_root
    EPEC_ROOT_OF,         // Exact_predicates_exact_constructions_kernel_with_root_of
    EPIC,                 // Exact_predicates_inexact_constructions_kernel
    Cartesian,
    Simple_cartesian
    //Homogeneous,        /* Not implemented */
    //Simple_homogeneous  /* Not implemented */
  } KernelEnum;

  vtkGetMacro(KernelValue, int);
  vtkSetMacro(KernelValue, int);

  virtual void SetKernelValueToEPEC(void)               { this->SetKernelValue(KernelEnum::EPEC); }
  virtual void SetKernelValueToEPEC_SQRT(void)          { this->SetKernelValue(KernelEnum::EPEC_SQRT); }
  virtual void SetKernelValueToEPEC_KTH_ROOT(void)      { this->SetKernelValue(KernelEnum::EPEC_KTH_ROOT); }
  virtual void SetKernelValueToEPEC_ROOT_OF(void)       { this->SetKernelValue(KernelEnum::EPEC_ROOT_OF); }
  virtual void SetKernelValueToEPIC(void)               { this->SetKernelValue(KernelEnum::EPIC); }
  virtual void SetKernelValueToCartesian(void)          { this->SetKernelValue(KernelEnum::Cartesian); }
  virtual void SetKernelValueToSimpleCartesian(void)    { this->SetKernelValue(KernelEnum::Simple_cartesian); }
  //virtual void SetKernelValueToHomogeneous(void)        { this->SetKernelValue(KernelEnum::Homogeneous); }
  //virtual void SetKernelValueToSimpleHomogeneous(void)  { this->SetKernelValue(KernelEnum::Simple_homogeneous); }
  //@}

  //@{
  /**
  * Set/Get MinRegionSize. The minimum number of mesh faces a region must have.
  */
  vtkGetMacro(MinRegionSize, int);
  vtkSetClampMacro(MinRegionSize, int, 0, VTK_INT_MAX);
  //@}

  //@{
  /**
  * Set/Get MaxDistanceToPlane. The maximum distance from the furthest face vertex to a plane.
  */
  vtkGetMacro(MaxDistanceToPlane, double);
  vtkSetClampMacro(MaxDistanceToPlane, double, 0.0, VTK_DOUBLE_MAX);
  //@}

  //@{
  /**
  * Set/Get MaxAcceptedAngle. The maximum accepted angle between the face normal and the normal of a plane.
  */
  vtkGetMacro(MaxAcceptedAngle, double);
  vtkSetMacro(MaxAcceptedAngle, double);
  //@}

protected:
  stkCGALRegionGrowing();
  ~stkCGALRegionGrowing() {}

  int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

  template <class CGalKernel>
  int Detection(vtkPolyData *, vtkPolyData *);

  template <typename MeshType>
  static bool vtkPolyDataToPolygonMesh(vtkPolyData *, MeshType&);

  int KernelValue;
  int MinRegionSize;
  double MaxDistanceToPlane;
  double MaxAcceptedAngle;

private:
  stkCGALRegionGrowing(const stkCGALRegionGrowing&) = delete;
  void operator=(const stkCGALRegionGrowing&) = delete;
};
#endif
