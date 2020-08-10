#ifndef vtkCGALPolygonSetOrientedSideClassifier_h
#define vtkCGALPolygonSetOrientedSideClassifier_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolygonSetOrientedSideClassifier : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolygonSetOrientedSideClassifier* New();
  vtkTypeMacro(vtkCGALPolygonSetOrientedSideClassifier, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputPointSet();
  vtkPolyData* GetInputPolyLineSet();

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

  enum OrientedSides {
      INSIDE = 1,
      OUTSIDE,
      BOUNDARY
  };

  vtkGetMacro(OrientedSide, int);
  vtkSetMacro(OrientedSide, int);

  void SetOrientedSideToInside() { OrientedSide = OrientedSides::INSIDE; }
  void SetOrientedSideToOutside() { OrientedSide = OrientedSides::OUTSIDE; }
  void SetOrientedSideToBoundary() { OrientedSide = OrientedSides::BOUNDARY; }

  //@{
  /**
  * Name of the array storing the oriented side results. 
  * -1 for outside, 0 for boundary and +1 for inside.
  */
  vtkSetMacro(OrientedSideArrayName, std::string);
  vtkGetMacro(OrientedSideArrayName, std::string);
  //@}

protected:
  vtkCGALPolygonSetOrientedSideClassifier();
  ~vtkCGALPolygonSetOrientedSideClassifier(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;
  std::string PwhIdArrayName;
  int OrientedSide;
  std::string OrientedSideArrayName;

private:
  // needed but not implemented
  vtkCGALPolygonSetOrientedSideClassifier(const vtkCGALPolygonSetOrientedSideClassifier&) = delete;
  void operator=(const vtkCGALPolygonSetOrientedSideClassifier&) = delete;

  bool DebugMode;
};
#endif
