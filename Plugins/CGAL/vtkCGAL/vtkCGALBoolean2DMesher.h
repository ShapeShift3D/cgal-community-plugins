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
  
  vtkPolyData* GetInputPolyLineSetA();
  vtkPolyData* GetInputPolyLineSetB();

  enum OperationModes {
      JOIN = 1,
      INTERSECTION,
      DIFFERENCE,
      DIFFERENCE2,
      SYMMETRIC_DIFFERENCE,
      COMPLEMENT,
      NAND,
      XOR
  };

  vtkGetMacro(OperationMode, int);
  vtkSetMacro(OperationMode, int);

  void SetOperationModeToJoin() { OperationMode = OperationModes::JOIN; }
  void SetOperationModeToIntersection() { OperationMode = OperationModes::INTERSECTION; }
  void SetOperationModeToDifference() { OperationMode = OperationModes::DIFFERENCE; }
  void SetOperationModeToDifference2() { OperationMode = OperationModes::DIFFERENCE2; }
  void SetOperationModeToSymmetricDifference() { OperationMode = OperationModes::SYMMETRIC_DIFFERENCE; }
  void SetOperationModeToComplement() { OperationMode = OperationModes::COMPLEMENT; }
  void SetOperationModeToNAND() { OperationMode = OperationModes::NAND; }
  void SetOperationModeToXOR() { OperationMode = OperationModes::XOR; }

  enum Inputs {
      A = 1,
      B
  };

  vtkGetMacro(ComplementOf, int);
  vtkSetMacro(ComplementOf, int);

  void SetComplementOfA() { ComplementOf = Inputs::A; }
  void SetComplementOfB() { ComplementOf = Inputs::B; }

protected:
  vtkCGALBoolean2DMesher();
  ~vtkCGALBoolean2DMesher(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int OperationMode;
  int ComplementOf;

private:
  // needed but not implemented
  vtkCGALBoolean2DMesher(const vtkCGALBoolean2DMesher&) = delete;
  void operator=(const vtkCGALBoolean2DMesher&) = delete;

  bool DebugMode;
};
#endif
