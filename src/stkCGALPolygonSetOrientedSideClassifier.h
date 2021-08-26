#ifndef stkCGALPolygonSetOrientedSideClassifier_h
#define stkCGALPolygonSetOrientedSideClassifier_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALPolygonSetOrientedSideClassifier : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static stkCGALPolygonSetOrientedSideClassifier* New();
  vtkTypeMacro(stkCGALPolygonSetOrientedSideClassifier, vtkPolyDataAlgorithm);
  
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


  //@{
  /**
  * Name of the array storing the oriented side results. 
  * -1 for outside, 0 for boundary and +1 for inside.
  */
  vtkSetMacro(OrientedSideArrayName, std::string);
  vtkGetMacro(OrientedSideArrayName, std::string);
  //@}

protected:
  stkCGALPolygonSetOrientedSideClassifier();
  ~stkCGALPolygonSetOrientedSideClassifier(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;
  std::string PwhIdArrayName;
  std::string OrientedSideArrayName;

private:
  // needed but not implemented
  stkCGALPolygonSetOrientedSideClassifier(const stkCGALPolygonSetOrientedSideClassifier&) = delete;
  void operator=(const stkCGALPolygonSetOrientedSideClassifier&) = delete;

  bool DebugMode;
};
#endif
