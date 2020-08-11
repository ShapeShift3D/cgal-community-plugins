#ifndef vtkCGALPolygonOrientOperator_h
#define vtkCGALPolygonOrientOperator_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolygonOrientOperator : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolygonOrientOperator* New();
  vtkTypeMacro(vtkCGALPolygonOrientOperator, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputPolyLine();

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

  enum PolygonOrientations {
      CLOCKWISE = 1,
      COUNTERCLOCKWISE
  };

  //@{
  /**
  * Inverts the PolyLine A direction
  */
  vtkGetMacro(InvertPolyLineOrientation, bool);
  vtkSetMacro(InvertPolyLineOrientation, bool);
  vtkBooleanMacro(InvertPolyLineOrientation, bool);
  //@}

  //@{
  /**
  * Forces the PolyLine A orientation
  */
  vtkGetMacro(ForcePolyLineOrientation, bool);
  vtkSetMacro(ForcePolyLineOrientation, bool);
  vtkBooleanMacro(ForcePolyLineOrientation, bool);
  //@}

  //@{
  /**
  * Choose the orientation to force
  */
  vtkGetMacro(PolyLineOrientation, int);
  vtkSetMacro(PolyLineOrientation, int);
  //@}

  void SetPolyLineAOrientationToClockwise() { PolyLineOrientation = PolygonOrientations::CLOCKWISE; }
  void SetPolyLineAOrientationToCounterclockwise() { PolyLineOrientation = PolygonOrientations::COUNTERCLOCKWISE; }

protected:
  vtkCGALPolygonOrientOperator();
  ~vtkCGALPolygonOrientOperator(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;
  bool InvertPolyLineOrientation;
  bool ForcePolyLineOrientation;
  int PolyLineOrientation;

private:
  // needed but not implemented
  vtkCGALPolygonOrientOperator(const vtkCGALPolygonOrientOperator&) = delete;
  void operator=(const vtkCGALPolygonOrientOperator&) = delete;
};
#endif
