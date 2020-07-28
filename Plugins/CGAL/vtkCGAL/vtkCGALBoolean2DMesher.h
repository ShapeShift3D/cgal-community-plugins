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

  enum OperationModes {
      JOIN = 1,
      INTERSECTION,
      DIFFERENCE,
      SYMMETRIC_DIFFERENCE
  };

  vtkGetMacro(OperationMode, int);
  vtkSetMacro(OperationMode, int);

  void SetOperationModeToJoin() { OperationMode = OperationModes::JOIN; }
  void SetOperationModeToIntersection() { OperationMode = OperationModes::INTERSECTION; }
  void SetOperationModeToDifference() { OperationMode = OperationModes::DIFFERENCE; }
  void SetOperationModeToSymmetricDifference() { OperationMode = OperationModes::SYMMETRIC_DIFFERENCE; }


protected:
  vtkCGALBoolean2DMesher();
  ~vtkCGALBoolean2DMesher(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int OperationMode;

private:
  // needed but not implemented
  vtkCGALBoolean2DMesher(const vtkCGALBoolean2DMesher&) = delete;
  void operator=(const vtkCGALBoolean2DMesher&) = delete;
};
#endif
