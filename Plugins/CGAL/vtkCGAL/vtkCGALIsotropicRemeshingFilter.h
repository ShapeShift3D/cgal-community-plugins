#ifndef vtkCGALIsotropicRemeshingFilter_h
#define vtkCGALIsotropicRemeshingFilter_h

#include <vtkCGALModule.h>
#include <vtkGeometryFilter.h>
#include <vtkFiltersCoreModule.h>

class VTKCGAL_EXPORT vtkCGALIsotropicRemeshingFilter : public vtkGeometryFilter
{
public:
  static vtkCGALIsotropicRemeshingFilter *New();
  vtkTypeMacro(vtkCGALIsotropicRemeshingFilter, vtkGeometryFilter);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkSetClampMacro(TargetEdgeLength, double, 0.0, VTK_DOUBLE_MAX);
  vtkGetMacro(TargetEdgeLength, double);

  vtkGetMacro(TargetEdgeLengthInfo, double);

  vtkSetClampMacro(NumberOfIterations, int, 1, VTK_INT_MAX);
  vtkGetMacro(NumberOfIterations, int);

  vtkSetMacro(PreserveBorderEdges, vtkTypeBool);
  vtkGetMacro(PreserveBorderEdges, vtkTypeBool);
  vtkBooleanMacro(PreserveBorderEdges, vtkTypeBool);

  // Performs the isotropic remeshing algorithm and fills the output object here.
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

protected:
  vtkCGALIsotropicRemeshingFilter();
  ~vtkCGALIsotropicRemeshingFilter(){}

  // Computes the bbox's diagonal length to set the default target edge length.
  int RequestInformation(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  double TargetEdgeLength;
  double TargetEdgeLengthInfo;

  int NumberOfIterations;
  vtkTypeBool PreserveBorderEdges;

  vtkCGALIsotropicRemeshingFilter(const vtkCGALIsotropicRemeshingFilter&) = delete;
  void operator=(const vtkCGALIsotropicRemeshingFilter&) = delete;
};
#endif
