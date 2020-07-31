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
  
  vtkPolyData* GetInputPolyLineA();
  vtkPolyData* GetInputPolyLineB();

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

  enum PolygonOrientations {
      CLOCKWISE = 1,
      COUNTERCLOCKWISE
  };

  //@{
  /**
  * Inverts the PolyLine A direction
  */
  vtkGetMacro(InvertPolyLineAOrientation, bool);
  vtkSetMacro(InvertPolyLineAOrientation, bool);
  vtkBooleanMacro(InvertPolyLineAOrientation, bool);
  //@}

  //@{
  /**
  * Forces the PolyLine A orientation
  */
  vtkGetMacro(ForcePolyLineAOrientation, bool);
  vtkSetMacro(ForcePolyLineAOrientation, bool);
  vtkBooleanMacro(ForcePolyLineAOrientation, bool);
  //@}

  //@{
  /**
  * Choose the orientation to force
  */
  vtkGetMacro(PolyLineAOrientation, int);
  vtkSetMacro(PolyLineAOrientation, int);
  //@}

  void SetPolyLineAOrientationToClockwise() { PolyLineAOrientation = PolygonOrientations::CLOCKWISE; }
  void SetPolyLineAOrientationToCounterclockwise() { PolyLineAOrientation = PolygonOrientations::COUNTERCLOCKWISE; }

  //@{
  /**
  * Inverts the PolyLine B orientation
  */
  vtkGetMacro(InvertPolyLineBOrientation, bool);
  vtkSetMacro(InvertPolyLineBOrientation, bool);
  vtkBooleanMacro(InvertPolyLineBOrientation, bool);
  //@}

  //@{
  /**
  * Forces the PolyLine B orientation
  */
  vtkGetMacro(ForcePolyLineBOrientation, bool);
  vtkSetMacro(ForcePolyLineBOrientation, bool);
  vtkBooleanMacro(ForcePolyLineBOrientation, bool);
  //@}

  //@{
  /**
  * Choose the orientation to force
  */
  vtkGetMacro(PolyLineBOrientation, int);
  vtkSetMacro(PolyLineBOrientation, int);
  //@}

  void SetPolyLineBOrientationnToClockwise() { PolyLineBOrientation = PolygonOrientations::CLOCKWISE; }
  void SetPolyLineBOrientationToCounterclockwise() { PolyLineBOrientation = PolygonOrientations::COUNTERCLOCKWISE; }

protected:
  vtkCGALBoolean2DMesher();
  ~vtkCGALBoolean2DMesher(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int OperationMode;

  // PolyLine A
  bool InvertPolyLineAOrientation;
  bool ForcePolyLineAOrientation;
  int PolyLineAOrientation;

  // PolyLine B
  bool InvertPolyLineBOrientation;
  bool ForcePolyLineBOrientation;
  int PolyLineBOrientation;

private:
  // needed but not implemented
  vtkCGALBoolean2DMesher(const vtkCGALBoolean2DMesher&) = delete;
  void operator=(const vtkCGALBoolean2DMesher&) = delete;

  bool DebugMode;
};
#endif
