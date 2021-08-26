#ifndef stkCGALBoolean2DMesher_h
#define stkCGALBoolean2DMesher_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALBoolean2DMesher : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static stkCGALBoolean2DMesher* New();
  vtkTypeMacro(stkCGALBoolean2DMesher, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputPolyLineSetA();
  vtkPolyData* GetInputPolyLineSetB();

  enum OperationModes {
      ADD = 1,
      INTERSECT,
      A_MINUS_B,
      B_MINUS_A,
      EXCLUDE_OVERLAP,
      COMPLEMENT,
      INTERSECT_COMPLEMENT,
      EXCLUSIVE_ADD
  };

  //@{
  /**
  * This property indicates which operation mode will be used.
  */
  vtkGetMacro(OperationMode, int);
  vtkSetMacro(OperationMode, int);
  //@}

  void SetOperationModeToAdd() { OperationMode = OperationModes::ADD; }
  void SetOperationModeToIntersect() { OperationMode = OperationModes::INTERSECT; }
  void SetOperationModeToAMinusB() { OperationMode = OperationModes::A_MINUS_B; }
  void SetOperationModeToBMinusA() { OperationMode = OperationModes::B_MINUS_A; }
  void SetOperationModeToExcludeOverlap() { OperationMode = OperationModes::EXCLUDE_OVERLAP; }
  void SetOperationModeToComplement() { OperationMode = OperationModes::COMPLEMENT; }
  void SetOperationModeToIntersectComplement() { OperationMode = OperationModes::INTERSECT_COMPLEMENT; }
  void SetOperationModeToExclusiveAdd() { OperationMode = OperationModes::EXCLUSIVE_ADD; }

  enum Inputs {
      A = 1,
      B
  };

  //@{
  /**
  * Apply a complement on an input
  */
  vtkGetMacro(ComplementOf, int);
  vtkSetMacro(ComplementOf, int);
  //@}

  void SetComplementOfA() { ComplementOf = Inputs::A; }
  void SetComplementOfB() { ComplementOf = Inputs::B; }

  enum Planes {
      XY = 1,
      YZ,
      XZ
  };

  vtkGetMacro(Plane, int);
  vtkSetMacro(Plane, int);

  void SetPlaneToXY() { Plane = Planes::XY; }
  void SetPlaneToYZ() { Plane = Planes::YZ; }
  void SetPlaneToXZ() { Plane = Planes::XZ; }

  //@{
  /**
  * Name of the Polygon With Holes ID array.
  */
  vtkSetMacro(PwhIdArrayName, std::string);
  vtkGetMacro(PwhIdArrayName, std::string);
  //@}

  //@{
  /**
  * If true, every polygon will be composed of one cell.
  */
  vtkGetMacro(OneCell, bool);
  vtkSetMacro(OneCell, bool);
  vtkBooleanMacro(OneCell, bool);
  //@}

protected:
  stkCGALBoolean2DMesher();
  ~stkCGALBoolean2DMesher(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int OperationMode;
  int ComplementOf;
  int Plane;
  std::string PwhIdArrayName;
  bool OneCell;
  bool DebugMode;

private:
  // needed but not implemented
  stkCGALBoolean2DMesher(const stkCGALBoolean2DMesher&) = delete;
  void operator=(const stkCGALBoolean2DMesher&) = delete;

};
#endif
