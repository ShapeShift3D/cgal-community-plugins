#ifndef vtkCGALBoolean2DMesher_h
#define vtkCGALBoolean2DMesher_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALBoolean2DMesher : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALBoolean2DMesher* New();
  vtkTypeMacro(vtkCGALBoolean2DMesher, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();

  //vtkGetMacro(Mode, int);
  //vtkSetMacro(Mode, int);

protected:
  vtkCGALBoolean2DMesher();
  ~vtkCGALBoolean2DMesher(){}
  //int Mode;
  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

private:
  // needed but not implemented
  vtkCGALBoolean2DMesher(const vtkCGALBoolean2DMesher&) = delete;
  void operator=(const vtkCGALBoolean2DMesher&) = delete;
};
#endif
