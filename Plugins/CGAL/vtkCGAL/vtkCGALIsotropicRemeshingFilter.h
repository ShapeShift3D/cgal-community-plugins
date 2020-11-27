#ifndef vtkCGALIsotropicRemeshingFilter_h
#define vtkCGALIsotropicRemeshingFilter_h
// Gives access to macros for communication with the UI
#include "vtkFiltersCoreModule.h" 
#include "vtkGeometryFilter.h"
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALIsotropicRemeshingFilter : public vtkGeometryFilter
{
public:
  // VTK requirements
  static vtkCGALIsotropicRemeshingFilter* New();
  vtkTypeMacro(vtkCGALIsotropicRemeshingFilter, vtkGeometryFilter);
  // Prints the values of the specific data
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Communicate with the UI
  vtkSetMacro(Length, double);
  vtkGetMacro(Length, double);
  vtkSetMacro(LengthInfo, double);
  vtkGetMacro(LengthInfo, double);
  vtkSetMacro(MainIterations, int);
  vtkGetMacro(MainIterations, int);

  // Pipeline functions:
  // Performs the isotropic remeshing algorithm and fills the output object here.
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *)override;
  // Specifies the type of the input objects
  int FillInputPortInformation(int, vtkInformation *info)override;
  // Specifies the type of the output object.
  int FillOutputPortInformation(int, vtkInformation *info)override;

protected:
  vtkCGALIsotropicRemeshingFilter();
  ~vtkCGALIsotropicRemeshingFilter(){}

  // Computes the bbox's diagonal length to set the default target edge length.
  int RequestInformation(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  // Data set by the UI and used by the algorithm
  double Length;
  double LengthInfo;
  int MainIterations;

  // needed but not implemented
  vtkCGALIsotropicRemeshingFilter(const vtkCGALIsotropicRemeshingFilter&);
  void operator=(const vtkCGALIsotropicRemeshingFilter&);
};
#endif
