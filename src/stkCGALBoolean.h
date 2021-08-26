#ifndef stkCGALBoolean_h
#define stkCGALBoolean_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALBoolean : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static stkCGALBoolean* New();
  vtkTypeMacro(stkCGALBoolean, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();
 
  int FillOutputPortInformation(int, vtkInformation *info) override;

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

  Surface_Mesh* RunBooleanOperations(
	  Surface_Mesh& tm1,
	  Surface_Mesh& tm2,
	  Surface_Mesh& operation,
	  int Mode
	  );

  enum Modes { UNION, INTERSECTION, TM1_MINUS_TM2, TM2_MINUS_TM1};

  vtkGetMacro(Mode, int);
  vtkSetMacro(Mode, int);

  void SetModeToUnion();
  void SetModeToIntersection();
  void SetModeToDifference();
  void SetModeToDifference2();

protected:
  stkCGALBoolean();
  ~stkCGALBoolean(){}
  int Mode;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*)override;

private:
  // needed but not implemented
  stkCGALBoolean(const stkCGALBoolean&) = delete;
  void operator=(const stkCGALBoolean&) = delete;
};
#endif
