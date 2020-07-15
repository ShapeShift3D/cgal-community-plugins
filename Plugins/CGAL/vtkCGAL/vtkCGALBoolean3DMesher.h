#ifndef vtkCGALBoolean3DMesher_h
#define vtkCGALBoolean3DMesher_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALBoolean3DMesher : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALBoolean3DMesher* New();
  vtkTypeMacro(vtkCGALBoolean3DMesher, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

  Surface_Mesh* RunBooleanOperations(
	  Surface_Mesh& tm1,
	  Surface_Mesh& tm2,
	  Surface_Mesh& operation);

  enum Modes { UNION, INTERSECTION, TM1_MINUS_TM2, TM2_MINUS_TM1};

  vtkGetMacro(Mode, int);
  vtkSetMacro(Mode, int);

  void SetModeToUnion();
  void SetModeToIntersection();
  void SetModeToDifference();
  void SetModeToDifference2();

  //@{
  /**
  * Skip preconditions if true
  */
  vtkGetMacro(SkipPreconditions, bool);
  vtkSetMacro(SkipPreconditions, bool);
  vtkBooleanMacro(SkipPreconditions, bool);
  //@}

  //@{
  /**
  * Calculates surface intersection between both inputs.
  */
  vtkGetMacro(ComputeSurfaceIntersection, bool);
  vtkSetMacro(ComputeSurfaceIntersection, bool);
  vtkBooleanMacro(ComputeSurfaceIntersection, bool);
  //@}

protected:
  vtkCGALBoolean3DMesher();
  ~vtkCGALBoolean3DMesher(){}

  int Mode;
  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  bool SkipPreconditions;
  bool ComputeSurfaceIntersection;

private:
  // needed but not implemented
  vtkCGALBoolean3DMesher(const vtkCGALBoolean3DMesher&) = delete;
  void operator=(const vtkCGALBoolean3DMesher&) = delete;
};
#endif
