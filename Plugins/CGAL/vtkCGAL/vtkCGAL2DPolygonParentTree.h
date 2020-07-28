#ifndef vtkCGAL2DPolygonParentTree_h
#define vtkCGAL2DPolygonParentTree_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

#include <string>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGAL2DPolygonParentTree : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGAL2DPolygonParentTree* New();
  vtkTypeMacro(vtkCGAL2DPolygonParentTree, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMesh();

  vtkGetMacro(NodeIdArrayName, std::string);
  vtkSetMacro(NodeIdArrayName, std::string);

  vtkGetMacro(ParentRelationshipArrayName, std::string);
  vtkSetMacro(ParentRelationshipArrayName, std::string);

  enum Planes {
      XY = 1,
      YZ,
      XZ
  };

  vtkGetMacro(Plane, int);
  vtkSetMacro(Plane, int);

  void SetPlaneToJoin() { Plane = Planes::XY; }
  void SetPlaneToIntersection() { Plane = Planes::YZ; }
  void SetPlaneToDifference() { Plane = Planes::XZ; }


protected:
  vtkCGAL2DPolygonParentTree();
  ~vtkCGAL2DPolygonParentTree(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;

private:
  // needed but not implemented
  vtkCGAL2DPolygonParentTree(const vtkCGAL2DPolygonParentTree&) = delete;
  void operator=(const vtkCGAL2DPolygonParentTree&) = delete;

  std::string NodeIdArrayName;
  std::string ParentRelationshipArrayName;
  bool DebugMode;
};
#endif
