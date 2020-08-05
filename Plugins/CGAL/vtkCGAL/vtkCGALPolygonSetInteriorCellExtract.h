#ifndef vtkCGALPolygonSetInteriorCellExtract_h
#define vtkCGALPolygonSetInteriorCellExtract_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolygonSetInteriorCellExtract : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolygonSetInteriorCellExtract* New();
  vtkTypeMacro(vtkCGALPolygonSetInteriorCellExtract, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputPolyLineSet();
  vtkPolyData* GetInputCells();

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

  enum Criteria {
      CENTROID = 1
  };

  //@{
  /**
  * This property indicates which operation mode will be used.
  */
  vtkGetMacro(Criterion, int);
  vtkSetMacro(Criterion, int);
  //@}

  void SetCriterionToCentroid() { Criterion = Criteria::CENTROID; }

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

protected:
  vtkCGALPolygonSetInteriorCellExtract();
  ~vtkCGALPolygonSetInteriorCellExtract(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;
  int Criterion;
  int OrientedSide;

private:
  // needed but not implemented
  vtkCGALPolygonSetInteriorCellExtract(const vtkCGALPolygonSetInteriorCellExtract&) = delete;
  void operator=(const vtkCGALPolygonSetInteriorCellExtract&) = delete;

  bool DebugMode;
};
#endif
